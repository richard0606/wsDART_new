#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;  // 300ms 这类字面量

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include "yq_dart_aim/param_registry.hpp"
#include "yq_dart_aim/param_tuner.hpp"
#include "yq_dart_aim/yaml_writer.hpp"

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
    // 调参口也要从下位机的自动重连扫描里排除：
    // 否则 MCU 没插上时，scanAvailableACMPorts() 会把 USB-TTL(/dev/ttyACM1)
    // 当成下位机口打开，然后 send() 往里写二进制视觉协议 ——
    // 调参通道按行读 ASCII，会被二进制包打乱（node 里已有防呆，这里补上第二道）。
    serial_manager_.setExcludedPort(this->get_parameter("tune_port").as_string());
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
    // use_model_ 表示"用户是否要求模型模式"，加载失败不会把它改回 false，
    // 否则改好模型路径后不会再自动重试；实际是否在跑模型看 model_detector_.isReady()
    use_model_ = this->get_parameter("use_model").as_bool();
    if (use_model_) {
      // 加载失败会明确报错并退回 HSV（不再静默降级）
      tryEnableModel();
    } else {
      RCLCPP_INFO(this->get_logger(), "模型模式未启用（use_model=false），使用 HSV");
    }

    RCLCPP_INFO(this->get_logger(), "DartAimNode started (p_err=%f, mode=%s)",
                calc_params_.p_err, currentModeName());

    // 启动调试发布线程
    debug_thread_ = std::thread(&DartAimNode::debugPublishLoop, this);

    // 串口调参服务（独立于下位机通信的第二个串口，给非技术人员用）
    startParamTuner();
  }

  // ==================== 串口调参 ====================
  void startParamTuner() {
    const std::string port = this->get_parameter("tune_port").as_string();
    const int baud = static_cast<int>(this->get_parameter("tune_baud").as_int());
    if (port.empty()) {
      RCLCPP_INFO(this->get_logger(), "串口调参未启用（tune_port 为空）");
      return;
    }
    // 防呆：调参通道绝不能和下位机 MCU 用同一个串口。
    // 万一配错（两个设备路径写法不同但指向同一个设备），宁可不开调参，
    // 也不能往 MCU 视觉协议链路上发 ASCII/JSON。
    const std::string mcu_port = this->get_parameter("usart_port").as_string();
    if (!mcu_port.empty() && sameDevice(port, mcu_port)) {
      RCLCPP_ERROR(this->get_logger(),
                   "tune_port(%s) 和下位机 usart_port(%s) 是同一个设备，"
                   "为避免污染 MCU 视觉协议，调参服务未启动。请给调参换个口"
                   "（USB-TTL: /dev/ttyACM1；板载串口: /dev/ttyS1；或用 tcp:8766 走网络）",
                   port.c_str(), mcu_port.c_str());
      return;
    }
    // 相机参数在 hik_camera 节点上，调它们要走 ROS 服务代理
    cam_get_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(
        "/hik_camera/get_parameters");
    cam_set_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
        "/hik_camera/set_parameters");

    const bool started = tuner_.start(
        port, baud,
        [this](const std::string& name, yq_dart_aim::ParamValue* out) {
          return tunerGet(name, out);
        },
        [this](const std::string& name, const yq_dart_aim::ParamValue& v, std::string* err) {
          return tunerSet(name, v, err);
        },
        [this]() { return tunerStatus(); },
        [this](std::string* err) { return tunerSave(err); },
        [this]() { return listPresets(); },
        [this](const std::string& name, std::string* err) { return loadPreset(name, err); });

    if (started) {
      RCLCPP_INFO(this->get_logger(), "串口调参服务已启动: %s @ %d baud（PC 端用 tools/tune.py）",
                  port.c_str(), baud);
      // 命令必须在执行器线程里跑：tunerGet/tunerSet 会碰 rclcpp 的参数接口，
      // 而串口 I/O 在 ParamTuner 自己的线程上。若在那边直接执行，
      // 就会和图像回调、参数回调并发读写同一份状态（未定义行为）。
      tuner_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                             [this]() { drainTunerQueue(); });
    } else {
      RCLCPP_WARN(this->get_logger(), "串口调参启动失败: %s", port.c_str());
    }
  }

  // 从 I/O 线程取出待执行的调参命令，在执行器线程里处理，再把应答交回去
  void drainTunerQueue() {
    if (!tuner_timer_) return;
    std::string line;
    // 每轮最多处理若干条，避免连续灌命令时把这一轮定时器占满、饿死图像回调
    for (int i = 0; i < 8 && tuner_.popPendingCmd(&line); ++i) {
      tuner_.pushReply(tuner_.handleLine(line));
    }
  }

  // ---- 参数读写（tuner 的回调）----
  bool tunerGet(const std::string& name, yq_dart_aim::ParamValue* out) {
    const yq_dart_aim::ParamMeta* meta = yq_dart_aim::findParam(name);
    if (meta == nullptr) return false;

    if (meta->owner == yq_dart_aim::ParamOwner::kHikCamera) {
      return remoteGet(name, meta->type, out);
    }
    if (!this->has_parameter(name)) return false;
    const rclcpp::Parameter p = this->get_parameter(name);
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        out->type = yq_dart_aim::ParamValue::Type::kInt;
        out->i = p.as_int();
        return true;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        out->type = yq_dart_aim::ParamValue::Type::kDouble;
        out->d = p.as_double();
        return true;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        out->type = yq_dart_aim::ParamValue::Type::kBool;
        out->b = p.as_bool();
        return true;
      case rclcpp::ParameterType::PARAMETER_STRING:
        out->type = yq_dart_aim::ParamValue::Type::kString;
        out->s = p.as_string();
        return true;
      default:
        return false;
    }
  }

  bool tunerSet(const std::string& name, const yq_dart_aim::ParamValue& v, std::string* err) {
    const yq_dart_aim::ParamMeta* meta = yq_dart_aim::findParam(name);
    if (meta == nullptr) {
      *err = "没有这个参数";
      return false;
    }
    if (meta->owner == yq_dart_aim::ParamOwner::kHikCamera) {
      return remoteSet(name, v, meta->type, err);
    }
    if (!this->has_parameter(name)) {
      *err = "参数未声明";
      return false;
    }
    rclcpp::Parameter p;
    switch (v.type) {
      case yq_dart_aim::ParamValue::Type::kInt:
        p = rclcpp::Parameter(name, static_cast<int64_t>(v.i));
        break;
      case yq_dart_aim::ParamValue::Type::kDouble:
        p = rclcpp::Parameter(name, v.d);
        break;
      case yq_dart_aim::ParamValue::Type::kBool:
        p = rclcpp::Parameter(name, v.b);
        break;
      default:
        p = rclcpp::Parameter(name, v.s);
        break;
    }
    const auto result = this->set_parameter(p);
    if (!result.successful) {
      *err = result.reason.empty() ? "节点拒绝了该值" : result.reason;
      return false;
    }
    return true;
  }

  // ---- 跨节点：hik_camera 参数 ----
  // 相机节点名固定为 hik_camera（下面的 client 创建时就绑定了），
  // 所以不再传 node 参数进来 —— 留着反而会让人以为能传别的节点。
  bool remoteGet(const std::string& name, yq_dart_aim::ParamType type,
                 yq_dart_aim::ParamValue* out) {
    if (!cam_get_client_ || !cam_get_client_->wait_for_service(300ms)) return false;
    auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    req->names.push_back(name);
    auto fut = cam_get_client_->async_send_request(req);
    if (fut.wait_for(500ms) != std::future_status::ready) return false;
    auto resp = fut.get();
    if (!resp || resp->values.empty()) return false;
    const auto& pv = resp->values[0];
    switch (type) {
      case yq_dart_aim::ParamType::kInt:
        if (pv.type != rclcpp::ParameterType::PARAMETER_INTEGER) return false;
        out->type = yq_dart_aim::ParamValue::Type::kInt;
        out->i = pv.integer_value;
        return true;
      case yq_dart_aim::ParamType::kDouble:
        if (pv.type != rclcpp::ParameterType::PARAMETER_DOUBLE) return false;
        out->type = yq_dart_aim::ParamValue::Type::kDouble;
        out->d = pv.double_value;
        return true;
      case yq_dart_aim::ParamType::kBool:
        if (pv.type != rclcpp::ParameterType::PARAMETER_BOOL) return false;
        out->type = yq_dart_aim::ParamValue::Type::kBool;
        out->b = pv.bool_value;
        return true;
      default:
        if (pv.type != rclcpp::ParameterType::PARAMETER_STRING) return false;
        out->type = yq_dart_aim::ParamValue::Type::kString;
        out->s = pv.string_value;
        return true;
    }
  }

  bool remoteSet(const std::string& name, const yq_dart_aim::ParamValue& v,
                 yq_dart_aim::ParamType type, std::string* err) {
    if (!cam_set_client_ || !cam_set_client_->wait_for_service(300ms)) {
      *err = "相机节点(/hik_camera)不可达";
      return false;
    }
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::ParameterValue pv;
    switch (type) {
      case yq_dart_aim::ParamType::kInt:
        pv.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        pv.integer_value = static_cast<int64_t>(v.i);
        break;
      case yq_dart_aim::ParamType::kDouble:
        pv.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        pv.double_value = v.d;
        break;
      case yq_dart_aim::ParamType::kBool:
        pv.type = rclcpp::ParameterType::PARAMETER_BOOL;
        pv.bool_value = v.b;
        break;
      default:
        pv.type = rclcpp::ParameterType::PARAMETER_STRING;
        pv.string_value = v.s;
        break;
    }
    rcl_interfaces::msg::Parameter param;
    param.name = name;
    param.value = pv;
    req->parameters.push_back(param);
    auto fut = cam_set_client_->async_send_request(req);
    if (fut.wait_for(500ms) != std::future_status::ready) {
      *err = "相机节点响应超时";
      return false;
    }
    auto resp = fut.get();
    if (!resp || resp->results.empty()) {
      *err = "相机节点无响应";
      return false;
    }
    if (!resp->results[0].successful) {
      *err = resp->results[0].reason.empty() ? "相机节点拒绝了该值" : resp->results[0].reason;
      return false;
    }
    return true;
  }

  // ---- STATUS：给调参界面看的运行状态 ----
  std::map<std::string, std::string> tunerStatus() {
    std::map<std::string, std::string> m;
    m["mode"] = currentModeName();
    m["backend"] = yq_dart_aim::backendName(infer_backend_);
    m["target_class"] = className(currentWantClass());
    m["serial_connected"] = serial_manager_.hasValidData() ? "yes" : "no";
    m["last_detection_ms"] = std::to_string(
        static_cast<long long>(model_detector_.resultAgeMs()));
    std::lock_guard<std::mutex> lock(status_mutex_);
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f", status_err_x_);
    m["aim_err_x_px"] = buf;
    m["aiming"] = status_aiming_ ? "yes" : "no";
    return m;
  }

  // ---- SAVE：把当前值写回 yaml ----
  bool tunerSave(std::string* err) {
    const std::string root = "/root/dart_ws_on/src/yq_dart_aim/config/params.yaml";
    const std::string cam = "/root/dart_ws_on/src/ros2_hik_camera/config/camera_params.yaml";

    std::vector<std::pair<std::string, yq_dart_aim::YamlValue>> dart_items;
    std::vector<std::pair<std::string, yq_dart_aim::YamlValue>> cam_items;
    for (const auto& meta : yq_dart_aim::paramRegistry()) {
      yq_dart_aim::ParamValue v;
      if (!tunerGet(meta.name, &v)) continue;
      yq_dart_aim::YamlValue yv;
      switch (meta.type) {
        case yq_dart_aim::ParamType::kInt:
          yv.kind = yq_dart_aim::YamlValue::Kind::kInt;
          yv.num = static_cast<double>(v.i);
          break;
        case yq_dart_aim::ParamType::kDouble:
          yv.kind = yq_dart_aim::YamlValue::Kind::kDouble;
          yv.num = v.d;
          break;
        case yq_dart_aim::ParamType::kBool:
          yv.kind = yq_dart_aim::YamlValue::Kind::kBool;
          yv.flag = v.b;
          break;
        default:
          yv.kind = yq_dart_aim::YamlValue::Kind::kString;
          yv.str = v.s;
          break;
      }
      if (meta.owner == yq_dart_aim::ParamOwner::kHikCamera) {
        cam_items.emplace_back(meta.name, yv);
      } else {
        dart_items.emplace_back(meta.name, yv);
      }
    }
    std::string e1;
    if (!yq_dart_aim::updateYamlFile(root, "dart_aim_node", dart_items, &e1)) {
      *err = e1;
      return false;
    }
    std::string e2;
    if (!cam_items.empty() && !yq_dart_aim::updateYamlFile(cam, "hik_camera", cam_items, &e2)) {
      *err = e2;
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "串口调参 SAVE: 已写回 %zu 项到 params.yaml, %zu 项到 camera_params.yaml",
                dart_items.size(), cam_items.size());
    return true;
  }

  // ---- 预设 ----
  std::vector<std::string> listPresets() {
    std::vector<std::string> out;
    for (const auto& p : presetNames()) {
      out.push_back(p.stem().string());
    }
    return out;
  }

  std::vector<std::filesystem::path> presetNames() const {
    std::vector<std::filesystem::path> out;
    for (const auto& entry : std::filesystem::directory_iterator(presetDir())) {
      if (entry.path().extension() == ".yaml") out.push_back(entry.path());
    }
    std::sort(out.begin(), out.end());
    return out;
  }

  std::filesystem::path presetDir() const {
    return std::filesystem::path("/root/dart_ws_on/src/yq_dart_aim/config/presets");
  }

  bool loadPreset(const std::string& name, std::string* err) {
    const std::filesystem::path path = presetDir() / (name + ".yaml");
    if (!std::filesystem::exists(path)) {
      *err = "预设不存在: " + name;
      return false;
    }
    std::ifstream f(path);
    std::vector<std::pair<std::string, yq_dart_aim::YamlValue>> items;
    std::string line;
    while (std::getline(f, line)) {
      const std::string t = trim(line);
      if (t.empty() || t[0] == '#' || t[0] == '_') continue;
      const size_t colon = t.find(':');
      if (colon == std::string::npos) continue;
      const std::string key = trim(t.substr(0, colon));
      const std::string val = trim(t.substr(colon + 1));
      if (val.empty()) continue;
      const yq_dart_aim::ParamMeta* meta = yq_dart_aim::findParam(key);
      if (meta == nullptr) continue;  // 预设里的旧参数名直接忽略
      yq_dart_aim::ParamValue v;
      bool ok = false;
      switch (meta->type) {
        case yq_dart_aim::ParamType::kInt:
          v.type = yq_dart_aim::ParamValue::Type::kInt;
          v.i = std::strtoll(val.c_str(), nullptr, 10);
          break;
        case yq_dart_aim::ParamType::kDouble:
          v.type = yq_dart_aim::ParamValue::Type::kDouble;
          v.d = std::strtod(val.c_str(), nullptr);
          ok = true;
          break;
        case yq_dart_aim::ParamType::kBool:
          v.type = yq_dart_aim::ParamValue::Type::kBool;
          v.b = (val == "true" || val == "1");
          break;
        default:
          v.type = yq_dart_aim::ParamValue::Type::kString;
          v.s = val;
          break;
      }
      if (meta->type == yq_dart_aim::ParamType::kInt) ok = true;
      if (meta->type == yq_dart_aim::ParamType::kBool) ok = true;
      if (meta->type == yq_dart_aim::ParamType::kString) ok = true;
      if (!ok) continue;
      yq_dart_aim::YamlValue yv;
      yv.kind = (meta->type == yq_dart_aim::ParamType::kInt)
                    ? yq_dart_aim::YamlValue::Kind::kInt
                    : (meta->type == yq_dart_aim::ParamType::kDouble)
                          ? yq_dart_aim::YamlValue::Kind::kDouble
                          : (meta->type == yq_dart_aim::ParamType::kBool)
                                ? yq_dart_aim::YamlValue::Kind::kBool
                                : yq_dart_aim::YamlValue::Kind::kString;
      yv.num = (meta->type == yq_dart_aim::ParamType::kInt) ? static_cast<double>(v.i) : v.d;
      yv.flag = v.b;
      yv.str = v.s;
      items.emplace_back(key, yv);
      std::string e;
      tunerSet(key, v, &e);
    }
    RCLCPP_INFO(this->get_logger(), "串口调参: 已加载预设 %s（%zu 项，注意尚未落盘，需 SAVE）",
                name.c_str(), items.size());
    return true;
  }

  ~DartAimNode() override {
    tuner_.stop();
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
    // 调试图限速。JPEG 编码 1280x720 实测约 20ms/帧，跟着相机满帧率发的话
    // 单这一项就要占掉两个多核，图像回调会被拖垮、连带动慢瞄准。
    if (!this->has_parameter("debug_max_fps")) this->declare_parameter("debug_max_fps", 10);

    // 串口调参（PC 端用 tools/tune.py；端口为空则关闭）
    if (!this->has_parameter("tune_port")) this->declare_parameter("tune_port", yq_dart_aim::defaults::TUNE_PORT);
    if (!this->has_parameter("tune_baud")) this->declare_parameter("tune_baud", yq_dart_aim::defaults::TUNE_BAUD);

    // 图像话题
    if (!this->has_parameter("image_topic")) this->declare_parameter("image_topic", std::string("/image_raw"));
    // 相机端直出的 JPEG 话题；留空则退回订阅 raw /image_raw
    if (!this->has_parameter("image_topic_compressed")) this->declare_parameter("image_topic_compressed", std::string("/image_raw/jpeg"));

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
    if (!this->has_parameter("infer_backend")) this->declare_parameter("infer_backend", DART_DEFAULT_INFER_BACKEND);
    if (!this->has_parameter("model_path_cpu")) this->declare_parameter("model_path_cpu", std::string(""));
    if (!this->has_parameter("model_path_bpu")) this->declare_parameter("model_path_bpu", std::string(""));
    if (!this->has_parameter("conf_threshold")) this->declare_parameter("conf_threshold", 0.25);
    if (!this->has_parameter("model_nms_threshold")) this->declare_parameter("model_nms_threshold", 0.7);
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
  // 相对模型路径统一按 <包 share>/model/ 解析：
  //   params.yaml 里直接写 praysky_coord_576x768_cut.onnx 即可，
  //   RDK X5 与 x86 共用一套配置，不再写死 /root/dart_ws_on 这类绝对路径。
  static std::string resolveModelPath(const std::string& path) {
    if (path.empty() || path.front() == '/') {
      return path;
    }
    try {
      const std::string share = ament_index_cpp::get_package_share_directory("yq_dart_aim");
      return (std::filesystem::path(share) / "model" / path).string();
    } catch (const std::exception&) {
      return path;
    }
  }

  yq_dart_aim::ModelParams buildModelParams() const {
    yq_dart_aim::ModelParams p;
    p.backend = infer_backend_;
    // 分后端的模型路径：model_path_{cpu,bpu} 为空时回退到通用 model_path
    const std::string cpu_path = this->get_parameter("model_path_cpu").as_string();
    const std::string bpu_path = this->get_parameter("model_path_bpu").as_string();
    const std::string common_path = this->get_parameter("model_path").as_string();
    switch (p.backend) {
      case yq_dart_aim::InferBackend::kBpu:
        p.model_path = bpu_path.empty() ? common_path : bpu_path;
        break;
      default:
        p.model_path = cpu_path.empty() ? common_path : cpu_path;
        break;
    }
    p.model_path = resolveModelPath(p.model_path);
    p.conf_threshold = static_cast<float>(this->get_parameter("conf_threshold").as_double());
    p.nms_threshold = static_cast<float>(this->get_parameter("model_nms_threshold").as_double());
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
    if (!this->has_parameter("infer_backend")) return false;
    const std::string backend_text = this->get_parameter("infer_backend").as_string();
    if (!yq_dart_aim::parseInferBackendOrAuto(backend_text, infer_backend_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "infer_backend='%s' 无法识别或没有可用后端（可用: auto / CPU / BPU / OpenVINO / DNN）",
                   backend_text.c_str());
      return false;
    }
    const yq_dart_aim::ModelParams mp = buildModelParams();
    if (mp.model_path.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "use_model=true 但 %s 后端没有模型路径，已退回 HSV 模式"
                   "（CPU/OpenVINO 填 model_path_cpu 指向切好的 .onnx；BPU 填 model_path_bpu 指向量化 .bin）",
                   yq_dart_aim::backendName(mp.backend));
      return false;
    }
    if (!model_detector_.loadModel(mp)) {
      RCLCPP_ERROR(this->get_logger(),
                   "模型加载失败: 后端=%s 路径=%s，已退回 HSV 模式"
                   "（检查文件是否存在、格式是否与 infer_backend 匹配）",
                   yq_dart_aim::backendName(mp.backend), mp.model_path.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "模型模式已启用: 后端=%s 路径=%s (输入 %dx%d)",
                yq_dart_aim::backendName(mp.backend), mp.model_path.c_str(),
                mp.input_width, mp.input_height);
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
    debug_max_fps_ = std::max(1, static_cast<int>(this->get_parameter("debug_max_fps").as_int()));
    last_debug_time_ = 0.0;
    use_serial_ = this->get_parameter("use_serial").as_bool();
    model_default_target_ = clampTargetId(this->get_parameter("model_default_target").as_int());
  }

  static int clampTargetId(int v) {
    return std::max(0, std::min(yq_dart_aim::MAX_TARGET_ID, v));
  }

  // 模型相关参数改动后热重载。
  // 注意：on_set_parameters 回调执行期间 get_parameter() 读到的还是旧值
  // （新值要等回调返回后才提交），所以真正的重载交给一个 50ms 的一次性定时器，
  // 由执行器在回调结束之后触发。
  void scheduleModelReload(const char* what) {
    model_reload_reason_ = what;
    if (!model_reload_timer_) {
      model_reload_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        if (model_reload_timer_) model_reload_timer_->cancel();
        const std::string reason = model_reload_reason_;
        model_reload_reason_.clear();
        reloadModelIfActive(reason.c_str());
      });
    } else {
      model_reload_timer_->reset();
    }
  }

  const char* currentModeName() const {
    return (use_model_ && model_detector_.isReady()) ? "model" : "hsv";
  }

  // 当前"要打"的模型类别 id（-1 = 不瞄准）
  int currentWantClass() const {
    const bool serial_has_hint = serial_manager_.hasValidData();
    return yq_dart_aim::ModelDetector::classIdForTarget(
        serial_has_hint ? current_target_ : model_default_target_);
  }

  static std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
  }

  // 判断 tune_port 和 usart_port 是否可能是同一个设备（宁可多拦，不要误开 MCU 口）
  //
  // 真实情况很杂：软链接指向、/dev 下的别名、还没插上导致 realpath 失败……，
  // 所以这里用"保守判定"：字面相同、解析后相同、或 base 名相同（都算同一设备），
  // 另外只要 tune_port 看起来是个串口设备（/dev/tty*）且 usart_port 也是，就额外比对
  // 是否指向同一 tty 前缀。宁可误报（拒绝启动调参）也不要漏报（污染 MCU 链路）。
  static bool sameDevice(const std::string& a, const std::string& b) {
    if (a == b) return true;
    char ra[PATH_MAX] = {0};
    char rb[PATH_MAX] = {0};
    const bool oka = realpath(a.c_str(), ra) != nullptr;
    const bool okb = realpath(b.c_str(), rb) != nullptr;
    if (oka && okb) return std::string(ra) == std::string(rb);

    auto base = [](const std::string& p) {
      const size_t slash = p.find_last_of('/');
      return (slash == std::string::npos) ? p : p.substr(slash + 1);
    };
    // realpath 至少一边失败：先看 base 名
    if (base(a) == base(b)) return true;

    // 一边解析不出来时，如果它是软链接，看链接字面目标是不是另一个设备
    auto linkTarget = [](const std::string& p, std::string* out) {
      char buf[PATH_MAX] = {0};
      const ssize_t n = readlink(p.c_str(), buf, sizeof(buf) - 1);
      if (n <= 0) return false;
      *out = std::string(buf, static_cast<size_t>(n));
      return true;
    };
    std::string target;
    if (!oka && linkTarget(a, &target)) {
      // /tmp/fake_tune -> /dev/ttyACM0，target 的 base 名和 b 的 base 名一致
      if (base(target) == base(b)) return true;
    }
    if (!okb && linkTarget(b, &target)) {
      if (base(target) == base(a)) return true;
    }
    return false;
  }

  static std::string className(int class_id) {
    if (class_id < 0 || class_id >= yq_dart_aim::model_classes::kNumClasses) return "none";
    return yq_dart_aim::model_classes::kNames[class_id];
  }

  // 画一个带黑底的标签，保证在任何背景上都读得清
  static void drawLabel(cv::Mat& img, const std::string& text, cv::Point org, cv::Scalar color) {
    int baseline = 0;
    const double scale = 0.6;
    const int thickness = 2;
    cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
    const int x = std::max(0, std::min(org.x, img.cols - sz.width - 6));
    const int y = std::max(sz.height + 4, std::min(org.y, img.rows - 4));
    cv::rectangle(img, cv::Rect(x - 3, y - sz.height - 4, sz.width + 6, sz.height + 7),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness,
                cv::LINE_AA);
  }

  // 模型原生检测结果可视化：框和标签都直接来自模型输出
  // （box = 模型给的 xyxy，label = 模型的类别名 + 置信度），不做任何再加工
  void drawModelDetections(cv::Mat& vis,
                           const std::vector<yq_dart_aim::TargetInfo>& detections) const {
    for (const auto& t : detections) {
      const cv::Scalar color(0, 255, 0);
      cv::rectangle(vis, t.bbox, color, 2);
      cv::circle(vis, t.center, 3, cv::Scalar(0, 0, 255), -1);
      char label[64];
      snprintf(label, sizeof(label), "%s %.0f%%", className(t.class_id).c_str(),
               t.confidence * 100.0f);
      drawLabel(vis, label, cv::Point(t.bbox.x, t.bbox.y - 5), color);
    }
  }

  void reloadModelIfActive(const char* what) {
    if (!use_model_) {
      RCLCPP_INFO(this->get_logger(), "%s 已更新（当前非模型模式，未重载）", what);
      return;
    }
    model_detector_.unloadModel();
    sg_filter_x_.reset();
    last_result_seq_ = 0;
    tryEnableModel();
    RCLCPP_INFO(this->get_logger(), "模型重载(%s)结果: %s", what, currentModeName());
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
      } else if (p.get_name() == "debug_max_fps") {
        debug_max_fps_ = std::max(1, static_cast<int>(p.as_int()));
        RCLCPP_INFO(this->get_logger(), "调试图限速: %d fps", debug_max_fps_);
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
        use_model_ = p.as_bool();
        sg_filter_x_.reset();  // 切换模式时重置滤波器
        last_result_seq_ = 0;
        if (use_model_) {
          tryEnableModel();
        } else {
          model_detector_.unloadModel();
        }
        RCLCPP_INFO(this->get_logger(), "当前模式: %s",
                    use_model_ ? "模型" : "HSV");
      } else if (p.get_name() == "infer_backend") {
        yq_dart_aim::InferBackend parsed{};
        if (!yq_dart_aim::parseInferBackendOrAuto(p.as_string(), parsed)) {
          result.successful = false;
          result.reason = "infer_backend 只能是 auto / CPU / BPU / OpenVINO / DNN";
          continue;
        }
        scheduleModelReload("infer_backend");
      } else if (p.get_name() == "conf_threshold" || p.get_name() == "model_nms_threshold") {
        scheduleModelReload(p.get_name().c_str());
      } else if (p.get_name() == "model_path" ||
                 p.get_name() == "model_path_cpu" || p.get_name() == "model_path_bpu" ||
                 p.get_name() == "model_input_width" || p.get_name() == "model_input_height") {
        scheduleModelReload(p.get_name().c_str());
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
        image_compressed_sub_.reset();
        // QoS 必须和发布端匹配，否则一帧都收不到（而且是静默的，没有任何报错）。
        // hik_camera 侧 use_sensor_data_qos=true 时发布端是 BEST_EFFORT；
        // 而 create_subscription(topic, 10, ...) 默认是 RELIABLE，
        // RELIABLE 订阅端与 BEST_EFFORT 发布端不兼容 -> 图像话题永远不通。
        // 这里统一用 SensorDataQoS（KEEP_LAST + BEST_EFFORT），与 image_transport 的
        // 默认图像管线一致，且能同时兼容发布端的两种设置。
        // 深度取 1：图像来得比处理快时宁可丢旧的也不要堆积，积压只会让瞄准变迟钝。
        rclcpp::QoS qos(rclcpp::SensorDataQoS().keep_last(1));
        // 优先用相机端直出的 JPEG：1280x720 raw 每帧 2.76MB，Fast DDS 实测
        // 只能送出 2~6fps；JPEG 约 100KB，可以跟满相机帧率。
        const std::string compressed_topic =
            this->get_parameter("image_topic_compressed").as_string();
        if (!compressed_topic.empty()) {
            image_compressed_sub_ =
                this->create_subscription<sensor_msgs::msg::CompressedImage>(
                    compressed_topic, qos,
                    std::bind(&DartAimNode::imageCallbackCompressed, this, _1));
            RCLCPP_INFO(this->get_logger(), "图像源: %s (JPEG)", compressed_topic.c_str());
        } else {
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                topic, qos, std::bind(&DartAimNode::imageCallback, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "图像源: %s (raw)", topic.c_str());
        }
    }

  // ==================== 图像回调（核心处理流程） ====================
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 保持 cv_bridge 共享指针存活，确保 ROI 视图数据有效
    auto cv_image = cv_bridge::toCvShare(msg, "bgr8");
    processImage(cv_image->image, msg->header);
  }

  // 压缩图回调：相机端直接发 JPEG（约 100KB），避免 2.76MB raw 图
  // 在 Fast DDS 上实测只有 2~6fps 的吞吐。
  void imageCallbackCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv::Mat image = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if (image.empty()) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock, 2000, "JPEG 解码失败，丢帧");
      return;
    }
    processImage(image, msg->header);
  }

  void processImage(const cv::Mat& image, const std_msgs::msg::Header& header) {
    const rclcpp::Time now = header.stamp;

    // 裁剪中心区域（ROI 视图，零拷贝）
    cv::Mat cropped = image_processor_.cropCenter(
        image, image_params_.crop_width, image_params_.crop_height);

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

    // 调试图限速**每帧只能判一次**：debugDue() 会推进 last_debug_time_，
    // 在同一帧里判两次的话第二次必然是 false，会导致调试图永远发不出去。
    const bool debug_this_frame = publish_debug_image_ && debugDue(now);

    // 调试图底图：模型模式下先 clone 一次并画好检测框，
    // 后面的十字/目标点直接叠加在同一张上，避免每帧 clone 两次、编码两次。
    cv::Mat debug_base;

    if (use_model_ && model_detector_.isReady()) {
      // 模型模式：异步提交裁剪图，获取最新结果
      model_detector_.submit(cropped);
      targets = model_detector_.getResults();

      // 模型检测可视化（提交给调试线程）
      //
      // 注意：这里不能直接调 publishModelDebugImage。JPEG 编码 1280x720 实测
      // 约 20ms/帧，放在图像回调里同步做，等于每帧白烧 20ms CPU：
      // 相机 113fps 时需要 2.2 秒算力/秒（占 2.2 个核），物理上根本跑不满，
      // 回调被拖住还会连带拖慢瞄准。所以和普通调试图一样丢给调试线程去做，
      // 并且调试线程再限一次速率（见 debugPublishLoop）。
      if (debug_this_frame) {
        debug_base = cropped.clone();
        drawModelDetections(debug_base, targets);
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

      {
        std::lock_guard<std::mutex> sl(status_mutex_);
        status_err_x_ = aim_result.err_x;
        status_aiming_ = aimed;
      }

      serial_manager_.send(send_err, aim_info);
    } else {
      aim_counter_ = 0;
      serial_manager_.send(0.0f, 0.0f);
      std::lock_guard<std::mutex> sl(status_mutex_);
      status_err_x_ = 0.0;
      status_aiming_ = false;
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
    // 限速：只是给人看的可视化，不需要跟满相机帧率。相机能到 134fps，
    // 每帧都 clone + JPEG 编码纯属浪费（clone 2.7MB + 编码约 20ms）。
    if (debug_this_frame) {
      // 模型模式下 debug_base 已经画好检测框；HSV 模式为空，clone 原图
      cv::Mat debug_vis = debug_base.empty() ? cropped.clone() : std::move(debug_base);
      // 画调试图标注（十字瞄准线 + 目标点）
      if (target.x >= 0) {
        cv::circle(debug_vis, center_img, 6, cv::Scalar(255, 0, 0), 2);
        cv::circle(debug_vis, target, 4, cv::Scalar(0, 255, 0), -1);
      }
      std::lock_guard<std::mutex> lock(debug_mutex_);
      debug_img_ = std::move(debug_vis);
      debug_mask_ = mask;
      debug_header_ = header;
      new_debug_frame_.store(true);
    }
  }

  // 调试图限速：最多 debug_max_fps_ 帧/秒
  //
  // 注意：这里必须用单调时钟，不能用 msg->header.stamp。
  // 主回调本身会丢帧（相机 58fps、处理 27fps），相邻帧的相机时间戳
  // 可能 17ms / 34ms 交替；用 header.stamp 配合 33ms 周期会把大量帧误判掉
  // （实测限速 30fps 只发出 11fps）。单调时钟按真实经过时间判，行为稳定。
  bool debugDue(const rclcpp::Time& /*now*/) {
    const double period = 1.0 / static_cast<double>(std::max(1, debug_max_fps_));
    const double t = std::chrono::duration<double>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    // 留 5ms（且不超过周期的 20%）容差：主回调帧间隔与限速周期很接近时
    // （例如主回调 32fps、限速 30fps），严格比较会变成「每两帧放行一帧」，
    // 实测 30fps 只发出 11fps。容差让实际输出贴近主回调频率。
    const double tol = std::min(0.005, period * 0.2);
    if (last_debug_time_ > 0.0 && (t - last_debug_time_) < period - tol) return false;
    last_debug_time_ = t;
    return true;
  }

  // 调试图只给人看，编码前统一缩到 640 宽。
  // 1280x720 JPEG 编码在推理线程满载时会被挤到 20~30ms/张，
  // 缩到 640x360 后编码量降到 1/4，调试发布频率不再被编码拖住。
  static cv::Mat shrinkForDebug(const cv::Mat& img) {
    if (img.cols <= 640) return img;
    cv::Mat small;
    const int w = 640;
    const int h = std::max(1, img.rows * w / img.cols);
    cv::resize(img, small, cv::Size(w, h), 0, 0, cv::INTER_AREA);
    return small;
  }

  // ==================== 调试图像发布 ====================
  void publishDebugImages(const cv::Mat& img, const cv::Mat& mask,
                          const std_msgs::msg::Header& header) {
    try {
      // 压缩图像（是否发布由 publish_debug_image 在回调侧统一控制）
      if (!img.empty()) {
        try {
          std::vector<unsigned char> buf;
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
          if (cv::imencode(".jpg", shrinkForDebug(img), buf, params)) {
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = header;
            comp_msg.format = "jpeg";
            comp_msg.data.assign(buf.begin(), buf.end());
            compressed_image_pub_->publish(comp_msg);
            // 模型模式下 /model_debug 与 /dart_debug 是同一张图（框+十字都在上面），
            // 复用同一份 JPEG 编码，避免每帧编码两次。
            if (use_model_ && model_debug_compressed_pub_) {
              model_debug_compressed_pub_->publish(comp_msg);
            }
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "failed publish compressed image: %s", e.what());
        }
      }

      // 压缩 mask（模型模式下没有 mask，跳过）
      if (publish_compressed_mask_ && !mask.empty()) {
        try {
          std::vector<unsigned char> mbuf;
          if (cv::imencode(".png", shrinkForDebug(mask), mbuf)) {
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
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (cv::imencode(".jpg", shrinkForDebug(vis), buf, params)) {
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

      cv::Mat img, mask, model_img;
      std_msgs::msg::Header header, model_header;
      {
        std::lock_guard<std::mutex> lock(debug_mutex_);
        img = std::move(debug_img_);
        mask = std::move(debug_mask_);
        header = debug_header_;
        model_img = std::move(debug_model_img_);
        model_header = debug_model_header_;
        new_debug_frame_.store(false);
      }
      if (!img.empty()) {
        publishDebugImages(img, mask, header);
      }
      // 模型调试图的 JPEG 编码也放在这里做，不占用图像回调的时间
      if (!model_img.empty()) {
        publishModelDebugImage(model_img, model_header);
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
  // 模型调试图（模型模式用，与 debug_img_ 走同一个限速与编码线程）
  cv::Mat debug_model_img_;
  std_msgs::msg::Header debug_model_header_;
  // 调试图限速：相机能到 134fps，但调试图只是给人看的
  int debug_max_fps_ = 10;
  double last_debug_time_ = 0.0;

  // ROS 接口
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_mask_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_sub_;
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
  yq_dart_aim::InferBackend infer_backend_ = yq_dart_aim::InferBackend::kCpu;
  rclcpp::TimerBase::SharedPtr model_reload_timer_;
  std::string model_reload_reason_;
  uint64_t last_result_seq_ = 0;
  float filtered_x_ = 0.0f;

  // 串口调参
  yq_dart_aim::ParamTuner tuner_;
  rclcpp::TimerBase::SharedPtr tuner_timer_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr cam_get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr cam_set_client_;

  // 给 STATUS 用的运行状态快照
  std::mutex status_mutex_;
  double status_err_x_ = 0.0;
  bool status_aiming_ = false;
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
