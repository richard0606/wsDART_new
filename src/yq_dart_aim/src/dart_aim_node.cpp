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
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>

#include "yq_dart_aim/common/types.hpp"
#include "yq_dart_aim/common/constants.hpp"
#include "yq_dart_aim/image_processor.hpp"
#include "yq_dart_aim/target_detector.hpp"
#include "yq_dart_aim/model_detector.hpp"
#include "yq_dart_aim/sg_filter.hpp"
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
    // 只发布压缩图像，不发布未压缩的 sensor_msgs::Image
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dart_debug/image_compressed", 10);
    compressed_mask_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dart_debug/mask_compressed", 10);
    serial_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/serial", 10);
    serial_debug_pub_ = this->create_publisher<std_msgs::msg::String>("/serial_debug", 10);

    // 模型检测可视化话题（同样只发压缩图）
    model_debug_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/model_debug/image_compressed", 10);

    // 转发压缩图像用于 bag 录制（仅在录制功能开启时才创建/发布话题）
    enable_record_ = this->get_parameter("enable_record").as_bool();
    if (enable_record_) {
      setupRecordTopics();
    }

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
      // 加载失败会明确报错并退回 HSV（不再静默降级）
      use_model_ = tryEnableModel();
    } else {
      RCLCPP_INFO(this->get_logger(), "模型模式未启用（use_model=false），使用 HSV");
    }

    RCLCPP_INFO(this->get_logger(), "DartAimNode started (p_err=%f, mode=%s)",
                calc_params_.p_err, use_model_ ? "model" : "hsv");

    // 启动调试发布线程
    debug_thread_ = std::thread(&DartAimNode::debugPublishLoop, this);
  }

  ~DartAimNode() override {
    // 停止调试发布线程
    stop_debug_.store(true);
    new_debug_frame_.store(true);  // 唤醒线程
    if (debug_thread_.joinable()) {
      debug_thread_.join();
    }
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
    if (!this->has_parameter("publish_compressed_mask")) this->declare_parameter("publish_compressed_mask", yq_dart_aim::defaults::PUBLISH_COMPRESSED_MASK);
    if (!this->has_parameter("enable_record")) this->declare_parameter("enable_record", yq_dart_aim::defaults::ENABLE_RECORD);

    // 图像话题
    if (!this->has_parameter("image_topic")) this->declare_parameter("image_topic", std::string("/image_raw"));

    // 图像裁剪
    if (!this->has_parameter("crop_width")) this->declare_parameter("crop_width", yq_dart_aim::defaults::CROP_WIDTH);
    if (!this->has_parameter("crop_height")) this->declare_parameter("crop_height", yq_dart_aim::defaults::CROP_HEIGHT);

    // 目标筛选
    if (!this->has_parameter("p_err")) this->declare_parameter("p_err", yq_dart_aim::defaults::P_ERR);
    if (!this->has_parameter("morph_kernel_size")) this->declare_parameter("morph_kernel_size", yq_dart_aim::defaults::MORPH_KERNEL_SIZE);
    if (!this->has_parameter("morph_dilate_kernel_size")) this->declare_parameter("morph_dilate_kernel_size", yq_dart_aim::defaults::MORPH_DILATE_KERNEL_SIZE);
    if (!this->has_parameter("max_area")) this->declare_parameter("max_area", yq_dart_aim::defaults::MAX_AREA);

    // 模型检测
    // model_path: BPU 上是切好后处理导出的 .bin；开发机(cv::dnn)上是同一个 .onnx
    if (!this->has_parameter("use_model")) this->declare_parameter("use_model", false);
    if (!this->has_parameter("model_path")) this->declare_parameter("model_path", std::string(""));
    if (!this->has_parameter("conf_threshold")) this->declare_parameter("conf_threshold", 0.25);
    if (!this->has_parameter("model_input_width")) this->declare_parameter("model_input_width", 768);
    if (!this->has_parameter("model_input_height")) this->declare_parameter("model_input_height", 576);
    if (!this->has_parameter("model_default_target")) this->declare_parameter("model_default_target", yq_dart_aim::defaults::MODEL_DEFAULT_TARGET);
    if (!this->has_parameter("model_result_max_age_ms")) this->declare_parameter("model_result_max_age_ms", yq_dart_aim::defaults::MODEL_MAX_RESULT_AGE_MS);

    // 偏移查找表（默认值与 config/params.yaml 一致）
    if (!this->has_parameter("offset_0_0")) this->declare_parameter("offset_0_0", 35.0);
    if (!this->has_parameter("offset_0_1")) this->declare_parameter("offset_0_1", 35.0);
    if (!this->has_parameter("offset_0_2")) this->declare_parameter("offset_0_2", 35.0);
    if (!this->has_parameter("offset_0_3")) this->declare_parameter("offset_0_3", 35.0);
    if (!this->has_parameter("offset_1_0")) this->declare_parameter("offset_1_0", 35.0);
    if (!this->has_parameter("offset_1_1")) this->declare_parameter("offset_1_1", 35.0);
    if (!this->has_parameter("offset_1_2")) this->declare_parameter("offset_1_2", 35.0);
    if (!this->has_parameter("offset_1_3")) this->declare_parameter("offset_1_3", 35.0);
    if (!this->has_parameter("offset_2_0")) this->declare_parameter("offset_2_0", 12.0);
    if (!this->has_parameter("offset_2_1")) this->declare_parameter("offset_2_1", 29.0);
    if (!this->has_parameter("offset_2_2")) this->declare_parameter("offset_2_2", 34.0);
    if (!this->has_parameter("offset_2_3")) this->declare_parameter("offset_2_3", 30.0);

    // 加载偏移查找表
    for (int t = 0; t < 3; t++) {
      for (int d = 0; d < 4; d++) {
        char name[16];
        snprintf(name, sizeof(name), "offset_%d_%d", t, d);
        offset_lookup_[t][d] = this->get_parameter(name).as_double();
      }
    }
  }

  // ==================== 模型参数 ====================
  yq_dart_aim::ModelParams buildModelParams() const {
    yq_dart_aim::ModelParams p;
    p.model_path = this->get_parameter("model_path").as_string();
    p.conf_threshold = static_cast<float>(this->get_parameter("conf_threshold").as_double());
    p.input_width = static_cast<int>(this->get_parameter("model_input_width").as_int());
    p.input_height = static_cast<int>(this->get_parameter("model_input_height").as_int());
    p.max_result_age_ms = static_cast<int>(this->get_parameter("model_result_max_age_ms").as_int());
    return p;
  }

  // 尝试进入模型模式。任何一步失败都明确报错并返回 false（由调用方退回 HSV）。
  bool tryEnableModel() {
    if (model_detector_.isReady()) {
      return true;  // 已经加载过
    }
    const yq_dart_aim::ModelParams mp = buildModelParams();
    if (mp.model_path.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "use_model=true 但 model_path 为空，已退回 HSV 模式"
                   "（请在 params.yaml 里指定模型路径：BPU 后端填 .bin，ORT 后端填切好的 .onnx）");
      return false;
    }
    if (!model_detector_.loadModel(mp)) {
      RCLCPP_ERROR(this->get_logger(),
                   "模型加载失败: %s，已退回 HSV 模式"
                   "（检查文件是否存在、格式是否与编译时选择的推理后端匹配）",
                   mp.model_path.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "模型模式已启用: %s (输入 %dx%d)",
                mp.model_path.c_str(), mp.input_width, mp.input_height);
    return true;
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

    // 目标检测参数
    detect_params_.max_area = this->get_parameter("max_area").as_double();

    // 坐标计算参数
    calc_params_.p_err = this->get_parameter("p_err").as_double();
    calc_params_.offset_y = this->get_parameter("offset_y").as_double();

    // 调试选项
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    publish_compressed_mask_ = this->get_parameter("publish_compressed_mask").as_bool();
    use_serial_ = this->get_parameter("use_serial").as_bool();
    model_default_target_ = clampTargetId(this->get_parameter("model_default_target").as_int());
  }

  static int clampTargetId(int v) {
    return std::max(0, std::min(yq_dart_aim::MAX_TARGET_ID, v));
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
      } else if (p.get_name() == "publish_compressed_mask") {
        publish_compressed_mask_ = p.as_bool();
      } else if (p.get_name() == "enable_record") {
        enable_record_ = p.as_bool();
        if (enable_record_) {
          setupRecordTopics();
        } else {
          teardownRecordTopics();
        }
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
        use_model_ = p.as_bool() ? tryEnableModel() : false;
        sg_filter_x_.reset();  // 切换模式时重置滤波器
        RCLCPP_INFO(this->get_logger(), "当前模式: %s", use_model_ ? "模型" : "HSV");
      } else if (p.get_name() == "conf_threshold") {
        RCLCPP_INFO(this->get_logger(), "conf_threshold updated (reload model to apply)");
      } else if (p.get_name() == "model_default_target") {
        model_default_target_ = clampTargetId(p.as_int());
        RCLCPP_INFO(this->get_logger(), "无串口提示时的兜底目标: %d (0=不瞄准, 1=前哨站, 2=基地)",
                    model_default_target_);
      } else if (p.get_name() == "model_result_max_age_ms") {
        model_detector_.setMaxResultAgeMs(p.as_int());
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

  // ==================== 录制转发话题 ====================
  // /dart_debug/* -> /debug_record/*，仅 enable_record=true 时创建，
  // 避免不开录制时白白占用话题和转发带宽
  void setupRecordTopics() {
    if (debug_record_image_pub_) return;  // 已创建
    debug_record_mask_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/debug_record/mask_compressed", 10);
    debug_record_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/debug_record/image_compressed", 10);
    debug_record_mask_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart_debug/mask_compressed", 10,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (debug_record_mask_pub_) debug_record_mask_pub_->publish(*msg);
      });
    debug_record_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart_debug/image_compressed", 10,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (debug_record_image_pub_) debug_record_image_pub_->publish(*msg);
      });
    RCLCPP_INFO(this->get_logger(), "录制话题已开启 (/debug_record/*)");
  }

  void teardownRecordTopics() {
    if (!debug_record_image_pub_) return;  // 本来就未创建
    debug_record_mask_sub_.reset();
    debug_record_image_sub_.reset();
    debug_record_mask_pub_.reset();
    debug_record_image_pub_.reset();
    RCLCPP_INFO(this->get_logger(), "录制话题已关闭 (/debug_record/*)");
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
    // 保持 cv_bridge 共享指针存活，确保 ROI 视图数据有效
    auto cv_image = cv_bridge::toCvShare(msg, "bgr8");

    // 裁剪中心区域（ROI 视图，零拷贝）
    cv::Mat cropped = image_processor_.cropCenter(
        cv_image->image, image_params_.crop_width, image_params_.crop_height);

    // 图像中心（按裁剪区域尺寸算，两种模式一致）
    int cx = cropped.cols / 2;
    int cy = cropped.rows / 2;

    // 更新偏移
    coord_calculator_.updateOffset(offset_lookup_, current_target_, current_dart_id_);

    // ==================== 目标检测（HSV / 模型 切换） ====================
    // 两种模式的预处理完全独立：模型模式只把裁剪图交给模型自己处理，
    // 不再做 HSV 阈值/灰度那套传统 CV 预处理（结果也用不上）
    std::vector<yq_dart_aim::TargetInfo> targets;
    cv::Mat mask;  // 仅 HSV 模式产生；模型模式为空

    if (use_model_ && model_detector_.isReady()) {
      // 模型模式：异步提交裁剪图，获取最新结果
      model_detector_.submit(cropped);
      targets = model_detector_.getResults();

      // 模型检测可视化（提交给调试线程）
      if (publish_debug_image_) {
        cv::Mat model_vis = cropped.clone();
        for (const auto& t : targets) {
          cv::Scalar color(0, 255, 0);
          cv::rectangle(model_vis, t.bbox, color, 2);
          char label[64];
          std::string cls_name =
              (t.class_id >= 0 && t.class_id < yq_dart_aim::model_classes::kNumClasses)
                  ? yq_dart_aim::model_classes::kNames[t.class_id]
                  : "cls" + std::to_string(t.class_id);
          snprintf(label, sizeof(label), "%s %.0f%%", cls_name.c_str(), t.confidence * 100);
          cv::putText(model_vis, label, cv::Point(t.bbox.x, t.bbox.y - 5),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
          cv::circle(model_vis, t.center, 3, cv::Scalar(0, 0, 255), -1);
        }
        publishModelDebugImage(model_vis, msg->header);
      }
    } else {
      // HSV 模式：裁剪图 -> 二值化 -> 形态学，灰度图用于亚像素加权质心
      mask = image_processor_.processCropped(cropped, image_params_);
      cv::Mat gray;
      cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
      targets = target_detector_.detect(mask, gray, detect_params_);
    }

    // ==================== 目标选择 ====================
    // 下位机的 enemy 字段指定打哪个目标：1=前哨站(s0_o6)、2=基地(s1_o7)、0=无目标
    // 模型类别 id 与 enemy 不是同一个编号体系，必须查表映射，不能直接比
    // 下位机没给过任何有效包时（串口没接/没上电/被禁用），用兜底目标，
    // 否则模型模式会因为拿不到 enemy 而永远不瞄准
    cv::Point2f target(-1, -1);
    if (use_model_ && model_detector_.isReady()) {
      const bool serial_has_hint = serial_manager_.hasValidData();
      const int want_target = serial_has_hint ? current_target_ : model_default_target_;
      const int want_class = yq_dart_aim::ModelDetector::classIdForTarget(want_target);
      if (want_class >= 0) {
        const uint64_t seq = model_detector_.resultSeq();
        for (const auto& t : targets) {
          if (t.class_id == want_class) {
            // 异步推理下同一批结果会被多帧重复读到，时序滤波只在出现新结果时
            // 推入样本，否则等效于把同一个坐标重复喂 6 次（2Hz 检测 @13fps 图像）
            if (seq != last_result_seq_) {
              filtered_x_ = sg_filter_x_.push(t.center.x);
              last_result_seq_ = seq;
            }
            target = cv::Point2f(filtered_x_, t.center.y);
            break;
          }
        }
      }
      if (target.x < 0) {
        sg_filter_x_.reset();  // 目标丢失/被要求不瞄准，重置滤波器
        last_result_seq_ = 0;
      }
    } else {
      target = target_detector_.selectTarget(targets, current_target_);
    }

    // 坐标计算：瞄准中心 = 图像中心 + 偏移补偿
    // （全流程只有这一处施加偏移，CoordinateCalculator::calculate() 内部不再叠加）
    cv::Point center_img(cx + static_cast<int>(coord_calculator_.getCurrentOffsetX()),
                         cy + static_cast<int>(calc_params_.offset_y));

    if (target.x >= 0) {
      auto aim_result = coord_calculator_.calculate(target, center_img, calc_params_);

      if (std::abs(aim_result.err_x) < yq_dart_aim::defaults::AIM_THRESHOLD) {
        if (aim_counter_ < 5) aim_counter_++;
      } else {
        aim_counter_ = 0;
      }
      bool aimed = (aim_counter_ >= 5);
      float aim_info = aimed ? 1.0f : 0.0f;

      float send_err;
      if (aimed && std::abs(aim_result.err_x) < yq_dart_aim::defaults::AIM_DEAD_ZONE) {
        send_err = 0.0f;
      } else {
        send_err = static_cast<float>(aim_result.scaled_err_x);
      }

      geometry_msgs::msg::Point pt;
      pt.x = aim_result.scaled_err_x; pt.y = aim_result.scaled_err_y; pt.z = 0;
      debug_pub_->publish(pt);

      serial_manager_.send(send_err, aim_info);
    } else {
      aim_counter_ = 0;
      serial_manager_.send(0.0f, 0.0f);
    }

    // 转发串口调试数据
    // 目标状态与 use_serial_ 解耦：即使不发布调试话题，也要用下位机的目标提示；
    // 从未收到过有效包时 current_target_ 保持 0，模型模式走兜底目标
    int target_val = current_target_;
    int dart_id = current_dart_id_;
    float encoder_angle = 0.0f;
    serial_manager_.getReceivedData(target_val, dart_id, encoder_angle);
    if (serial_manager_.hasValidData()) {
      current_target_ = target_val;
      current_dart_id_ = dart_id;
    }

    if (use_serial_) {
      std_msgs::msg::Float32MultiArray serial_msg;
      serial_msg.data = {static_cast<float>(current_target_),
                         static_cast<float>(current_dart_id_),
                         encoder_angle};
      serial_pub_->publish(serial_msg);

      std::string hex = serial_manager_.getLastRxHex();
      if (!hex.empty()) {
        std_msgs::msg::String debug_msg;
        debug_msg.data = hex;
        serial_debug_pub_->publish(debug_msg);
      }
    }

    // 提交调试图像到调试线程（非阻塞）
    if (publish_debug_image_) {
      cv::Mat debug_vis = cropped.clone();
      // 画调试图标注（十字瞄准线 + 目标点）
      if (target.x >= 0) {
        cv::circle(debug_vis, center_img, 6, cv::Scalar(255, 0, 0), 2);
        cv::circle(debug_vis, target, 4, cv::Scalar(0, 255, 0), -1);
      }
      std::lock_guard<std::mutex> lock(debug_mutex_);
      debug_img_ = std::move(debug_vis);
      debug_mask_ = mask;
      debug_header_ = msg->header;
      new_debug_frame_.store(true);
    }
  }

  // ==================== 调试图像发布 ====================
  void publishDebugImages(const cv::Mat& img, const cv::Mat& mask,
                          const std_msgs::msg::Header& header) {
    try {
      // 压缩图像（是否发布由 publish_debug_image 在回调侧统一控制）
      if (!img.empty()) {
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

      // 压缩 mask（模型模式下没有 mask，跳过）
      if (publish_compressed_mask_ && !mask.empty()) {
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
      if (!vis.empty()) {
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

  // ==================== 调试发布线程循环 ====================
  void debugPublishLoop() {
    while (!stop_debug_.load()) {
      while (!new_debug_frame_.load() && !stop_debug_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (stop_debug_.load()) break;

      cv::Mat img, mask;
      std_msgs::msg::Header header;
      {
        std::lock_guard<std::mutex> lock(debug_mutex_);
        img = std::move(debug_img_);
        mask = std::move(debug_mask_);
        header = debug_header_;
        new_debug_frame_.store(false);
      }
      if (!img.empty()) {
        publishDebugImages(img, mask, header);
      }
    }
  }

  // ==================== 成员变量 ====================
  // 模块实例
  yq_dart_aim::ImageProcessor image_processor_;
  yq_dart_aim::TargetDetector target_detector_;
  yq_dart_aim::ModelDetector model_detector_;
  yq_dart_aim::SGFilter7 sg_filter_x_;  // 模型模式下 X 坐标时序平滑
  yq_dart_aim::CoordinateCalculator coord_calculator_;
  yq_dart_aim::SerialManager serial_manager_;

  // 调试发布线程
  std::thread debug_thread_;
  std::mutex debug_mutex_;
  std::atomic<bool> stop_debug_{false};
  std::atomic<bool> new_debug_frame_{false};
  cv::Mat debug_img_;
  cv::Mat debug_mask_;
  std_msgs::msg::Header debug_header_;

  // ROS 接口
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_mask_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr serial_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_debug_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // 模型检测话题
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
  bool use_serial_ = true;
  bool use_model_ = false;
  bool publish_debug_image_ = true;
  bool publish_compressed_mask_ = true;
  bool enable_record_ = false;
  int model_default_target_ = yq_dart_aim::defaults::MODEL_DEFAULT_TARGET;
  uint64_t last_result_seq_ = 0;
  float filtered_x_ = 0.0f;
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
