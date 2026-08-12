#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <opencv2/opencv.hpp>

#include "yq_dart_aim/common/types.hpp"
#include "yq_dart_aim/common/constants.hpp"
#include "yq_dart_aim/image_processor.hpp"
#include "yq_dart_aim/target_detector.hpp"
#include "yq_dart_aim/model_detector.hpp"
#include "yq_dart_aim/coordinate_calculator.hpp"
#include "yq_dart_aim/serial_manager.hpp"

using std::placeholders::_1;

class DartAimNode : public rclcpp::Node {
public:
  explicit DartAimNode(const rclcpp::NodeOptions & options) : Node("dart_aim_node", options) {
    // ==================== 参数声明 ====================
    declareParameters();

    // ==================== ROS 接口 ====================
    debug_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/dart_debug", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dart_debug/image", 10);
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dart_debug/mask", 10);
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dart_debug/image_compressed", 10);
    compressed_mask_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dart_debug/mask_compressed", 10);
    serial_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/serial", 10);
    serial_debug_pub_ = this->create_publisher<std_msgs::msg::String>("/serial_debug", 10);

    // 模型检测可视化话题
    model_debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/model_debug/image", 10);
    model_debug_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/model_debug/image_compressed", 10);

    // 转发压缩图像用于 bag 录制
    debug_record_mask_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/debug_record/mask_compressed", 10);
    debug_record_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/debug_record/image_compressed", 10);
    debug_record_mask_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart_debug/mask_compressed", 10,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        debug_record_mask_pub_->publish(*msg);
      });
    debug_record_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart_debug/image_compressed", 10,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        debug_record_image_pub_->publish(*msg);
      });

    // 图像订阅
    std::string topic = this->get_parameter("image_topic").as_string();
    createImageSubscription(topic);

    // 参数变更回调
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DartAimNode::onParamChange, this, std::placeholders::_1)
    );

    // ==================== 串口初始化 ====================
    updateCachedParams();
    serial_manager_.initialize(
      this->get_parameter("usart_port").as_string(),
      this->get_parameter("bound_rate").as_int()
    );
    serial_manager_.setLogCallback([this](yq_dart_aim::LogLevel level, const std::string& message) {
      switch (level) {
        case yq_dart_aim::LogLevel::DEBUG:
          RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
          break;
        case yq_dart_aim::LogLevel::INFO:
          RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
          break;
        case yq_dart_aim::LogLevel::WARN:
          RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
          break;
        case yq_dart_aim::LogLevel::ERROR:
          RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
          break;
      }
    });
    serial_manager_.setEnabled(use_serial_);
    if (use_serial_) {
      serial_manager_.startMonitoring();
    }

    // ==================== 模型检测器初始化 ====================
    use_model_ = this->get_parameter("use_model").as_bool();
    if (use_model_) {
      yq_dart_aim::ModelParams model_params;
      model_params.model_path = this->get_parameter("model_path").as_string();
      model_params.conf_threshold = static_cast<float>(this->get_parameter("conf_threshold").as_double());
      model_params.nms_threshold = static_cast<float>(this->get_parameter("nms_threshold").as_double());
      if (model_detector_.loadModel(model_params)) {
        RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_params.model_path.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", model_params.model_path.c_str());
        use_model_ = false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "DartAimNode started (p_err=%f, mode=%s)",
                calc_params_.p_err, use_model_ ? "model" : "hsv");
  }

  ~DartAimNode() override {
    serial_manager_.stopMonitoring();
  }

private:
  // ==================== 参数声明 ====================
  void declareParameters() {
    // HSV 阈值
    if (!this->has_parameter("h_min")) this->declare_parameter("h_min", yq_dart_aim::defaults::H_MIN);
    if (!this->has_parameter("s_min")) this->declare_parameter("s_min", yq_dart_aim::defaults::S_MIN);
    if (!this->has_parameter("v_min")) this->declare_parameter("v_min", yq_dart_aim::defaults::V_MIN);
    if (!this->has_parameter("h_max")) this->declare_parameter("h_max", yq_dart_aim::defaults::H_MAX);
    if (!this->has_parameter("s_max")) this->declare_parameter("s_max", yq_dart_aim::defaults::S_MAX);
    if (!this->has_parameter("v_max")) this->declare_parameter("v_max", yq_dart_aim::defaults::V_MAX);

    // 偏移
    if (!this->has_parameter("offset_y")) this->declare_parameter("offset_y", yq_dart_aim::defaults::OFFSET_Y);

    // 串口
    if (!this->has_parameter("bound_rate")) this->declare_parameter("bound_rate", yq_dart_aim::defaults::BAUD_RATE);
    if (!this->has_parameter("usart_port")) this->declare_parameter("usart_port", std::string("/dev/ttyACM0"));
    if (!this->has_parameter("use_serial")) this->declare_parameter("use_serial", yq_dart_aim::defaults::USE_SERIAL);

    // 调试输出
    if (!this->has_parameter("publish_debug_image")) this->declare_parameter("publish_debug_image", yq_dart_aim::defaults::PUBLISH_DEBUG_IMAGE);
    if (!this->has_parameter("publish_compressed")) this->declare_parameter("publish_compressed", yq_dart_aim::defaults::PUBLISH_COMPRESSED);
    if (!this->has_parameter("publish_compressed_mask")) this->declare_parameter("publish_compressed_mask", yq_dart_aim::defaults::PUBLISH_COMPRESSED_MASK);

    // 图像话题
    if (!this->has_parameter("image_topic")) this->declare_parameter("image_topic", std::string("/image_raw"));

    // 图像裁剪
    if (!this->has_parameter("crop_width")) this->declare_parameter("crop_width", yq_dart_aim::defaults::CROP_WIDTH);
    if (!this->has_parameter("crop_height")) this->declare_parameter("crop_height", yq_dart_aim::defaults::CROP_HEIGHT);

    // 目标筛选
    if (!this->has_parameter("circle_mask")) this->declare_parameter("circle_mask", yq_dart_aim::defaults::CIRCLE_MASK);
    if (!this->has_parameter("p_err")) this->declare_parameter("p_err", yq_dart_aim::defaults::P_ERR);
    if (!this->has_parameter("morph_kernel_size")) this->declare_parameter("morph_kernel_size", yq_dart_aim::defaults::MORPH_KERNEL_SIZE);
    if (!this->has_parameter("morph_dilate_kernel_size")) this->declare_parameter("morph_dilate_kernel_size", yq_dart_aim::defaults::MORPH_DILATE_KERNEL_SIZE);
    if (!this->has_parameter("max_area")) this->declare_parameter("max_area", yq_dart_aim::defaults::MAX_AREA);

    // 模型检测
    if (!this->has_parameter("use_model")) this->declare_parameter("use_model", false);
    if (!this->has_parameter("model_path")) this->declare_parameter("model_path", std::string(""));
    if (!this->has_parameter("conf_threshold")) this->declare_parameter("conf_threshold", 0.5);
    if (!this->has_parameter("nms_threshold")) this->declare_parameter("nms_threshold", 0.45);

    // 偏移查找表
    if (!this->has_parameter("offset_0_0")) this->declare_parameter("offset_0_0", 43.0);
    if (!this->has_parameter("offset_0_1")) this->declare_parameter("offset_0_1", 43.0);
    if (!this->has_parameter("offset_0_2")) this->declare_parameter("offset_0_2", 43.0);
    if (!this->has_parameter("offset_0_3")) this->declare_parameter("offset_0_3", 43.0);
    if (!this->has_parameter("offset_1_0")) this->declare_parameter("offset_1_0", 43.0);
    if (!this->has_parameter("offset_1_1")) this->declare_parameter("offset_1_1", 43.0);
    if (!this->has_parameter("offset_1_2")) this->declare_parameter("offset_1_2", 43.0);
    if (!this->has_parameter("offset_1_3")) this->declare_parameter("offset_1_3", 43.0);
    if (!this->has_parameter("offset_2_0")) this->declare_parameter("offset_2_0", 43.0);
    if (!this->has_parameter("offset_2_1")) this->declare_parameter("offset_2_1", 43.0);
    if (!this->has_parameter("offset_2_2")) this->declare_parameter("offset_2_2", 43.0);
    if (!this->has_parameter("offset_2_3")) this->declare_parameter("offset_2_3", 43.0);

    // 加载偏移查找表
    for (int t = 0; t < 3; t++) {
      for (int d = 0; d < 4; d++) {
        char name[16];
        snprintf(name, sizeof(name), "offset_%d_%d", t, d);
        offset_lookup_[t][d] = this->get_parameter(name).as_double();
      }
    }
  }

  // ==================== 缓存参数更新 ====================
  void updateCachedParams() {
    // 图像处理参数
    image_params_.crop_width = this->get_parameter("crop_width").as_int();
    image_params_.crop_height = this->get_parameter("crop_height").as_int();
    image_params_.h_min = this->get_parameter("h_min").as_int();
    image_params_.s_min = this->get_parameter("s_min").as_int();
    image_params_.v_min = this->get_parameter("v_min").as_int();
    image_params_.h_max = this->get_parameter("h_max").as_int();
    image_params_.s_max = this->get_parameter("s_max").as_int();
    image_params_.v_max = this->get_parameter("v_max").as_int();
    image_params_.morph_kernel_size = this->get_parameter("morph_kernel_size").as_int();
    image_params_.morph_dilate_kernel_size = this->get_parameter("morph_dilate_kernel_size").as_int();
    image_params_.circle_mask = this->get_parameter("circle_mask").as_bool();

    // 目标检测参数
    detect_params_.max_area = this->get_parameter("max_area").as_double();
    detect_params_.circle_mask = image_params_.circle_mask;

    // 坐标计算参数
    calc_params_.p_err = this->get_parameter("p_err").as_double();
    calc_params_.offset_y = this->get_parameter("offset_y").as_double();

    // 调试选项
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
    publish_compressed_mask_ = this->get_parameter("publish_compressed_mask").as_bool();
    use_serial_ = this->get_parameter("use_serial").as_bool();
  }

  // ==================== 参数变更回调 ====================
  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    for (const auto &p : params) {
      if (p.get_name() == "use_serial") {
        use_serial_ = p.as_bool();
        serial_manager_.setEnabled(use_serial_);
        if (use_serial_) serial_manager_.startMonitoring();
        continue;
      }
      if (p.get_name() == "bound_rate") {
        // 重新初始化串口
        serial_manager_.stopMonitoring();
        serial_manager_.initialize(
          this->get_parameter("usart_port").as_string(), p.as_int());
        if (use_serial_) serial_manager_.startMonitoring();
        continue;
      }
      if (p.get_name() == "image_topic") {
        createImageSubscription(p.as_string());
        RCLCPP_INFO(this->get_logger(), "image subscription changed to %s", p.as_string().c_str());
      } else if (p.get_name() == "circle_mask") {
        image_params_.circle_mask = p.as_bool();
        detect_params_.circle_mask = p.as_bool();
      } else if (p.get_name() == "crop_width") {
        image_params_.crop_width = p.as_int();
      } else if (p.get_name() == "crop_height") {
        image_params_.crop_height = p.as_int();
      } else if (p.get_name() == "h_min") {
        image_params_.h_min = p.as_int();
      } else if (p.get_name() == "s_min") {
        image_params_.s_min = p.as_int();
      } else if (p.get_name() == "v_min") {
        image_params_.v_min = p.as_int();
      } else if (p.get_name() == "h_max") {
        image_params_.h_max = p.as_int();
      } else if (p.get_name() == "s_max") {
        image_params_.s_max = p.as_int();
      } else if (p.get_name() == "v_max") {
        image_params_.v_max = p.as_int();
      } else if (p.get_name() == "offset_y") {
        calc_params_.offset_y = p.as_double();
      } else if (p.get_name() == "publish_debug_image") {
        publish_debug_image_ = p.as_bool();
      } else if (p.get_name() == "publish_compressed") {
        publish_compressed_ = p.as_bool();
      } else if (p.get_name() == "publish_compressed_mask") {
        publish_compressed_mask_ = p.as_bool();
      } else if (p.get_name() == "p_err") {
        calc_params_.p_err = p.as_double();
        RCLCPP_INFO(this->get_logger(), "p_err updated: %f", calc_params_.p_err);
      } else if (p.get_name() == "morph_kernel_size") {
        image_params_.morph_kernel_size = p.as_int();
      } else if (p.get_name() == "morph_dilate_kernel_size") {
        image_params_.morph_dilate_kernel_size = p.as_int();
      } else if (p.get_name() == "max_area") {
        detect_params_.max_area = p.as_double();
      } else if (p.get_name() == "use_model") {
        use_model_ = p.as_bool();
        RCLCPP_INFO(this->get_logger(), "use_model updated: %s", use_model_ ? "true" : "false");
      } else if (p.get_name() == "conf_threshold") {
        // 模型置信度阈值（需要重新加载模型生效）
        RCLCPP_INFO(this->get_logger(), "conf_threshold updated (reload model to apply)");
      } else if (p.get_name() == "nms_threshold") {
        RCLCPP_INFO(this->get_logger(), "nms_threshold updated (reload model to apply)");
      }
      if (p.get_name().rfind("offset_", 0) == 0) {
        int t = 0, d = 0;
        if (sscanf(p.get_name().c_str(), "offset_%d_%d", &t, &d) == 2 &&
            t >= 0 && t < 3 && d >= 0 && d < 4) {
          offset_lookup_[t][d] = p.as_double();
          RCLCPP_INFO(this->get_logger(), "offset_%d_%d updated: %.2f", t, d, offset_lookup_[t][d]);
        }
      }
    }
    return result;
  }

  // ==================== 图像订阅 ====================
  void createImageSubscription(const std::string &topic) {
    image_sub_.reset();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic, 10, std::bind(&DartAimNode::imageCallback, this, _1)
    );
  }

  // ==================== 图像回调（核心处理流程） ====================
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 使用 ImageProcessor::process() 进行图像处理
    cv::Mat mask = image_processor_.process(img, image_params_);

    // 灰度图（与 mask 同尺寸裁剪）用于亚像素加权质心
    cv::Mat gray_full;
    cv::cvtColor(img, gray_full, cv::COLOR_BGR2GRAY);
    cv::Mat gray = image_processor_.cropCenter(gray_full, image_params_.crop_width, image_params_.crop_height);

    int width = mask.cols;
    int height = mask.rows;
    int cx = width / 2;
    int cy = height / 2;

    // 更新偏移
    coord_calculator_.updateOffset(offset_lookup_, current_target_, current_dart_id_);

    // ==================== 目标检测（HSV / 模型 切换） ====================
    std::vector<yq_dart_aim::TargetInfo> targets;
    cv::Mat cropped_img = image_processor_.cropCenter(img, image_params_.crop_width, image_params_.crop_height);

    if (use_model_ && model_detector_.isReady()) {
      // 模型模式：异步提交裁剪图，获取最新结果
      model_detector_.submit(cropped_img);
      targets = model_detector_.getResults();

      // 发布模型检测可视化
      if (publish_debug_image_) {
        cv::Mat model_vis = cropped_img.clone();
        for (const auto& t : targets) {
          // 画检测框
          cv::Scalar color(0, 255, 0);
          cv::rectangle(model_vis, t.bbox, color, 2);
          // 画类别+置信度
          char label[64];
          std::string cls_name = (t.class_id >= 0 && t.class_id < static_cast<int>(model_class_names_.size()))
                                 ? model_class_names_[t.class_id] : "cls" + std::to_string(t.class_id);
          snprintf(label, sizeof(label), "%s %.0f%%", cls_name.c_str(), t.confidence * 100);
          cv::putText(model_vis, label, cv::Point(t.bbox.x, t.bbox.y - 5),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
          // 画中心点
          cv::circle(model_vis, t.center, 3, cv::Scalar(0, 0, 255), -1);
        }
        publishModelDebugImage(model_vis, msg->header);
      }
    } else {
      // HSV 模式
      targets = target_detector_.detect(mask, gray, detect_params_);
    }

    cv::Point2f target = target_detector_.selectTarget(targets, current_target_, !startup_done_);
    if (targets.size() >= 1) startup_done_ = true;

    // 使用 CoordinateCalculator::calculate() 进行坐标计算
    cv::Point center_img(cx + static_cast<int>(coord_calculator_.getCurrentOffsetX()),
                         cy + static_cast<int>(calc_params_.offset_y));

    if (target.x >= 0) {
      auto aim_result = coord_calculator_.calculate(target, center_img, calc_params_);

      // 连续 5 帧误差 < 1.5px 判定为已瞄准
      if (std::abs(aim_result.err_x) < yq_dart_aim::defaults::AIM_THRESHOLD) {
        if (aim_counter_ < 5) aim_counter_++;
      } else {
        aim_counter_ = 0;
      }
      bool aimed = (aim_counter_ >= 5);
      float aim_info = aimed ? 1.0f : 0.0f;

      // 死区：已瞄准且差值 < 0.5px 时不修正，发 0
      float send_err;
      if (aimed && std::abs(aim_result.err_x) < yq_dart_aim::defaults::AIM_DEAD_ZONE) {
        send_err = 0.0f;
      } else {
        send_err = static_cast<float>(aim_result.scaled_err_x);
      }

      // 调试图标注
      cv::circle(img, center_img, 6, cv::Scalar(255, 0, 0), 2);
      cv::circle(img, target, 4, cv::Scalar(0, 255, 0), -1);

      geometry_msgs::msg::Point pt;
      pt.x = aim_result.scaled_err_x; pt.y = aim_result.scaled_err_y; pt.z = 0;
      debug_pub_->publish(pt);

      serial_manager_.send(send_err, aim_info);
    } else {
      aim_counter_ = 0;
      serial_manager_.send(0.0f, 0.0f);
    }

    // 转发串口调试数据
    if (use_serial_) {
      int target_val, dart_id;
      float encoder_angle;
      serial_manager_.getReceivedData(target_val, dart_id, encoder_angle);
      current_target_ = target_val;
      current_dart_id_ = dart_id;

      std_msgs::msg::Float32MultiArray serial_msg;
      serial_msg.data = {static_cast<float>(target_val),
                         static_cast<float>(dart_id),
                         encoder_angle};
      serial_pub_->publish(serial_msg);

      // 发布串口 hex 调试
      std::string hex = serial_manager_.getLastRxHex();
      if (!hex.empty()) {
        std_msgs::msg::String debug_msg;
        debug_msg.data = hex;
        serial_debug_pub_->publish(debug_msg);
      }
    }

    // 发布调试图像
    if (publish_debug_image_) {
      publishDebugImages(img, mask, msg->header);
    }
  }

  // ==================== 调试图像发布 ====================
  void publishDebugImages(const cv::Mat& img, const cv::Mat& mask,
                          const std_msgs::msg::Header& header) {
    try {
      // 原始调试图像
      cv_bridge::CvImage out_img;
      out_img.header = header;
      out_img.encoding = "bgr8";
      out_img.image = img;
      debug_image_pub_->publish(*out_img.toImageMsg());

      // 压缩图像
      if (publish_compressed_) {
        try {
          std::vector<unsigned char> buf;
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
          if (cv::imencode(".jpg", img, buf, params)) {
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = header;
            comp_msg.format = "jpeg";
            comp_msg.data.assign(buf.begin(), buf.end());
            compressed_image_pub_->publish(comp_msg);
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "failed publish compressed image: %s", e.what());
        }
      }

      // 压缩 mask
      if (publish_compressed_mask_) {
        try {
          std::vector<unsigned char> mbuf;
          if (cv::imencode(".png", mask, mbuf)) {
            sensor_msgs::msg::CompressedImage mcomp;
            mcomp.header = header;
            mcomp.format = "png";
            mcomp.data.assign(mbuf.begin(), mbuf.end());
            compressed_mask_pub_->publish(mcomp);
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "failed publish compressed mask: %s", e.what());
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "failed publish debug images: %s", e.what());
    }
  }

  // ==================== 模型检测可视化发布 ====================
  void publishModelDebugImage(const cv::Mat& vis, const std_msgs::msg::Header& header) {
    try {
      cv_bridge::CvImage out_img;
      out_img.header = header;
      out_img.encoding = "bgr8";
      out_img.image = vis;
      model_debug_image_pub_->publish(*out_img.toImageMsg());

      if (publish_compressed_) {
        std::vector<unsigned char> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        if (cv::imencode(".jpg", vis, buf, params)) {
          sensor_msgs::msg::CompressedImage comp_msg;
          comp_msg.header = header;
          comp_msg.format = "jpeg";
          comp_msg.data.assign(buf.begin(), buf.end());
          model_debug_compressed_pub_->publish(comp_msg);
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "failed publish model debug image: %s", e.what());
    }
  }

  // ==================== 成员变量 ====================
  // 模块实例
  yq_dart_aim::ImageProcessor image_processor_;
  yq_dart_aim::TargetDetector target_detector_;
  yq_dart_aim::ModelDetector model_detector_;
  yq_dart_aim::CoordinateCalculator coord_calculator_;
  yq_dart_aim::SerialManager serial_manager_;

  // ROS 接口
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_mask_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr serial_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_debug_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // 模型检测话题
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr model_debug_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr model_debug_compressed_pub_;

  // 转发发布/订阅
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_record_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_record_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debug_record_mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debug_record_image_sub_;

  // 参数缓存
  yq_dart_aim::ImageProcessParams image_params_;
  yq_dart_aim::DetectParams detect_params_;
  yq_dart_aim::CalcParams calc_params_;
  yq_dart_aim::OffsetLookup offset_lookup_;

  // 状态
  int current_target_ = 0;
  int current_dart_id_ = 0;
  int aim_counter_ = 0;  // 连续瞄准帧计数
  bool startup_done_ = false;
  bool use_serial_ = true;
  bool use_model_ = false;
  bool publish_debug_image_ = true;
  bool publish_compressed_ = true;
  bool publish_compressed_mask_ = true;
  std::vector<std::string> model_class_names_ = {"outpost", "base"};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DartAimNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
