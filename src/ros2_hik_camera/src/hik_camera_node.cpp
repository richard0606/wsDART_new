#include "MvCameraControl.h"
// ROS
#include <algorithm>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera {
    class HikCameraNode : public rclcpp::Node {
    public:
        explicit HikCameraNode(const rclcpp::NodeOptions &options) : Node("hik_camera", options) {
            RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

            MV_CC_DEVICE_INFO_LIST device_list;
            // enum device
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
            RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

            while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "No camera found!");
                RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
            }
            // 上面可能是因 rclcpp::ok() 变 false 而退出循环，此时 pDeviceInfo[0] 不可用
            if (!rclcpp::ok() || device_list.nDeviceNum == 0 || device_list.pDeviceInfo[0] == nullptr) {
                RCLCPP_FATAL(this->get_logger(), "No camera available, abort");
                rclcpp::shutdown();
                return;
            }

            if (MV_OK != MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0])) {
                RCLCPP_FATAL(this->get_logger(), "MV_CC_CreateHandle failed");
                camera_handle_ = nullptr;
                rclcpp::shutdown();
                return;
            }

            if (MV_OK != MV_CC_OpenDevice(camera_handle_)) {
                RCLCPP_FATAL(this->get_logger(), "MV_CC_OpenDevice failed (camera occupied?)");
                rclcpp::shutdown();
                return;
            }

            // Get camera infomation
            if (MV_OK != MV_CC_GetImageInfo(camera_handle_, &img_info_)) {
                RCLCPP_FATAL(this->get_logger(), "MV_CC_GetImageInfo failed");
                rclcpp::shutdown();
                return;
            }
            image_msg_.data.reserve(static_cast<size_t>(img_info_.nHeightMax) * img_info_.nWidthMax * 3);

            bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
            auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
            camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

            // 显式设置分辨率与像素格式（必须在 StartGrabbing 之前）
            configureImageFormat();
            declareParameters();

            if (MV_OK != MV_CC_StartGrabbing(camera_handle_)) {
                RCLCPP_FATAL(this->get_logger(), "MV_CC_StartGrabbing failed");
                rclcpp::shutdown();
                return;
            }

            // Load camera info
            camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
            camera_info_manager_ =
                    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
            auto camera_info_url =
                    this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
            if (camera_info_manager_->validateURL(camera_info_url)) {
                camera_info_manager_->loadCameraInfo(camera_info_url);
                camera_info_msg_ = camera_info_manager_->getCameraInfo();
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
            }

            params_callback_handle_ = this->add_on_set_parameters_callback(
                    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

            capture_thread_ = std::thread{[this]() -> void {
                MV_FRAME_OUT out_frame;

                RCLCPP_INFO(this->get_logger(), "Publishing image!");

                // prepare static fields（encoding 由 configureImageFormat 按 pixel_format 设定）
                image_msg_.header.frame_id = "camera_optical_frame";
                if (MV_OK != MV_CC_SetTriggerMode(camera_handle_, MV_TRIGGER_MODE_OFF)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to set trigger mode off");
                }

                while (rclcpp::ok()) {
                    nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
                    if (MV_OK == nRet) {
                        // set image dimensions and allocate buffer BEFORE conversion
                        image_msg_.height = out_frame.stFrameInfo.nHeight;
                        image_msg_.width = out_frame.stFrameInfo.nWidth;
                        image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
                        image_msg_.data.resize(static_cast<size_t>(image_msg_.width) * image_msg_.height * 3);

                        convert_param_.pDstBuffer = image_msg_.data.data();
                        convert_param_.nDstBufferSize = image_msg_.data.size();
                        convert_param_.pSrcData = out_frame.pBufAddr;
                        convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
                        convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
                        // 宽高按当前帧实际尺寸取值，避免与相机实际输出不一致
                        convert_param_.nWidth = out_frame.stFrameInfo.nWidth;
                        convert_param_.nHeight = out_frame.stFrameInfo.nHeight;

                        if (MV_OK != MV_CC_ConvertPixelType(camera_handle_, &convert_param_)) {
                            if (!convert_failed_) {
                                convert_failed_ = true;
                                RCLCPP_ERROR(this->get_logger(),
                                             "MV_CC_ConvertPixelType failed (src=%#x dst=%#x %ux%u), dropping frames",
                                             static_cast<unsigned int>(convert_param_.enSrcPixelType),
                                             static_cast<unsigned int>(convert_param_.enDstPixelType),
                                             convert_param_.nWidth, convert_param_.nHeight);
                            }
                            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                            continue;
                        }
                        convert_failed_ = false;

                        image_msg_.header.stamp = this->now();

                        camera_info_msg_.header = image_msg_.header;
                        camera_pub_.publish(image_msg_, camera_info_msg_);

                        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                        fail_conut_ = 0;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
                        int stop_ret = MV_CC_StopGrabbing(camera_handle_);
                        int start_ret = MV_CC_StartGrabbing(camera_handle_);
                        if (MV_OK != stop_ret || MV_OK != start_ret) {
                            RCLCPP_ERROR(this->get_logger(),
                                         "Failed to restart grabbing (stop=%#x start=%#x)", stop_ret, start_ret);
                        }
                        fail_conut_++;
                    }

                    if (fail_conut_ > 5) {
                        RCLCPP_FATAL(this->get_logger(), "Camera failed!");
                        rclcpp::shutdown();
                    }
                }
            }};
        }

        ~HikCameraNode() override {
            if (capture_thread_.joinable()) {
                capture_thread_.join();
            }
            if (camera_handle_) {
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_CloseDevice(camera_handle_);
                MV_CC_DestroyHandle(&camera_handle_);
            }
            RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
        }

    private:
        // 显式设置分辨率与像素格式
        // 不显式设置的话，输出格式取决于相机掉电前的状态：换相机/相机复位后分辨率变了，
        // 下游 dart_aim_node 的居中裁剪区域和按分辨率标定的偏移表就全偏了
        void configureImageFormat() {
            rcl_interfaces::msg::ParameterDescriptor desc;

            desc.description = "图像宽度，0 = 使用相机支持的最大值";
            const int cfg_width = this->declare_parameter("image_width", 0, desc);
            desc.description = "图像高度，0 = 使用相机支持的最大值";
            const int cfg_height = this->declare_parameter("image_height", 0, desc);

            MVCC_INTVALUE width_range{};
            MVCC_INTVALUE height_range{};
            if (MV_OK != MV_CC_GetIntValue(camera_handle_, "Width", &width_range) ||
                MV_OK != MV_CC_GetIntValue(camera_handle_, "Height", &height_range)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read Width/Height range, keep camera default");
                return;
            }

            unsigned int width = (cfg_width > 0) ? static_cast<unsigned int>(cfg_width) : width_range.nMax;
            unsigned int height = (cfg_height > 0) ? static_cast<unsigned int>(cfg_height) : height_range.nMax;
            width = std::min(std::max(width, width_range.nMin), width_range.nMax);
            height = std::min(std::max(height, height_range.nMin), height_range.nMax);

            if (MV_OK != MV_CC_SetIntValue(camera_handle_, "Width", width) ||
                MV_OK != MV_CC_SetIntValue(camera_handle_, "Height", height)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set image size %ux%u", width, height);
            }

            desc.description = "像素格式: RGB8 / BGR8";
            const std::string fmt = this->declare_parameter("pixel_format", std::string("RGB8"), desc);
            if (fmt == "BGR8") {
                convert_param_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                image_msg_.encoding = "bgr8";
            } else {
                if (fmt != "RGB8") {
                    RCLCPP_ERROR(this->get_logger(), "Unsupported pixel_format '%s', fallback to RGB8", fmt.c_str());
                }
                convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                image_msg_.encoding = "rgb8";
            }
            if (MV_OK != MV_CC_SetEnumValue(camera_handle_, "PixelFormat",
                                            static_cast<unsigned int>(convert_param_.enDstPixelType))) {
                RCLCPP_WARN(this->get_logger(), "Failed to set PixelFormat, keep camera default");
            }

            MVCC_INTVALUE width_now{};
            MVCC_INTVALUE height_now{};
            MV_CC_GetIntValue(camera_handle_, "Width", &width_now);
            MV_CC_GetIntValue(camera_handle_, "Height", &height_now);
            convert_param_.nWidth = width_now.nCurValue;
            convert_param_.nHeight = height_now.nCurValue;
            RCLCPP_INFO(this->get_logger(), "Image format: %ux%u %s",
                        width_now.nCurValue, height_now.nCurValue, image_msg_.encoding.c_str());
        }

        void declareParameters() {
            // Exposure time（整型参数，默认值保持整型字面量以维持参数类型）
            {
                rcl_interfaces::msg::ParameterDescriptor desc;
                MVCC_FLOATVALUE f_value{};
                desc.description = "Exposure time in microseconds";
                if (MV_OK == MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value)) {
                    desc.integer_range.resize(1);
                    desc.integer_range[0].step = 1;
                    desc.integer_range[0].from_value = f_value.fMin;
                    desc.integer_range[0].to_value = f_value.fMax;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to read ExposureTime range");
                }
                double exposure_time = this->declare_parameter("exposure_time", 5000, desc);
                if (MV_OK != MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set ExposureTime: %f", exposure_time);
                }
                RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);
            }

            // Gain（浮点参数）
            {
                rcl_interfaces::msg::ParameterDescriptor desc;
                MVCC_FLOATVALUE f_value{};
                desc.description = "Gain";
                if (MV_OK == MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value)) {
                    desc.integer_range.resize(1);
                    desc.integer_range[0].step = 1;
                    desc.integer_range[0].from_value = f_value.fMin;
                    desc.integer_range[0].to_value = f_value.fMax;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to read Gain range");
                }
                double gain = this->declare_parameter("gain", f_value.fCurValue, desc);
                if (MV_OK != MV_CC_SetFloatValue(camera_handle_, "Gain", gain)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set Gain: %f", gain);
                }
                RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
            }
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(
                const std::vector <rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param: parameters) {
                if (param.get_name() == "exposure_time") {
                    // exposure_time 声明为整型（默认值用的是整型字面量），这里按整型取值；
                    // 类型不匹配的下发会被 rclcpp 在进回调前就拒掉
                    int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
                    if (MV_OK != status) {
                        result.successful = false;
                        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
                    }
                } else if (param.get_name() == "gain") {
                    int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
                    if (MV_OK != status) {
                        result.successful = false;
                        result.reason = "Failed to set gain, status = " + std::to_string(status);
                    }
                } else {
                    result.successful = false;
                    result.reason = "Unknown parameter: " + param.get_name();
                }
            }
            return result;
        }

        // change CompressedImage to Image (we publish raw images, not compressed)
        sensor_msgs::msg::Image image_msg_;

        image_transport::CameraPublisher camera_pub_;

        int nRet = MV_OK;
        void *camera_handle_ = nullptr;  // 必须初始化：构造中途失败时析构函数会判空
        MV_IMAGE_BASIC_INFO img_info_;

        MV_CC_PIXEL_CONVERT_PARAM convert_param_;

        std::string camera_name_;
        std::unique_ptr <camera_info_manager::CameraInfoManager> camera_info_manager_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;

        int fail_conut_ = 0;
        bool convert_failed_ = false;  // 像素转换失败只记一次日志，避免刷屏
        std::thread capture_thread_;

        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    };
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
