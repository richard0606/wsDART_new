#include "MvCameraControl.h"
// ROS
#include <algorithm>
#include <cstring>
#include <string>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// OpenCV 放在 ROS 头之后：OpenCV 会带进 console_bridge/console.h，
// 放在前面会和 rclcpp 的头互相干扰（报 "expected unqualified-id before 'namespace'"）
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

    namespace hik_camera {
        // 像素格式值翻译成人能读的名字。
        // 判断"转换到底在不在相机上做"的关键：看到 Bayer* 就说明相机吐的是
        // 原始马赛克数据，去马赛克和上色是在 X5 的 CPU 上由 SDK 完成的。
        static const char *pixelFormatName(enum MvGvspPixelType t) {
            switch (t) {
                case PixelType_Gvsp_BayerGB8:   return "BayerGB8(原始马赛克)";
                case PixelType_Gvsp_BayerRG8:   return "BayerRG8(原始马赛克)";
                case PixelType_Gvsp_BayerBG8:   return "BayerBG8(原始马赛克)";
                case PixelType_Gvsp_BayerGR8:   return "BayerGR8(原始马赛克)";
                case PixelType_Gvsp_BayerGB10:  return "BayerGB10(原始马赛克)";
                case PixelType_Gvsp_BayerRG10:  return "BayerRG10(原始马赛克)";
                case PixelType_Gvsp_BayerBG10:  return "BayerBG10(原始马赛克)";
                case PixelType_Gvsp_BayerGR10:  return "BayerGR10(原始马赛克)";
                case PixelType_Gvsp_Mono8:      return "Mono8(灰度)";
                case PixelType_Gvsp_Mono16:     return "Mono16(灰度)";
                case PixelType_Gvsp_BGR8_Packed: return "BGR8(相机ISP已上色)";
                case PixelType_Gvsp_RGB8_Packed: return "RGB8(相机ISP已上色)";
                default:                        return "其它格式";
            }
        }

        // 把相机的像素格式映射到 OpenCV 的去马赛克/转换码。
        // 返回 -1 表示这个格式 OpenCV 处理不了（走 SDK 回退路径）。
        //
        // 【实测结论】OpenCV 与 GenICam 的 Bayer 命名规则不一致，名字相同并不代表
        // 排列相同，按名字硬对应会把 R/B 通道整体互换（红蓝颠倒，肉眼可见，且会
        // 让 HSV 阈值全部失效）。下表不是按名字推的，是用 SDK 自己的
        // MV_CC_ConvertPixelType 当基准实测出来的：
        //
        //   做法：合成一张 2x2 四格各填不同数值的"探针图"，同一张图分别喂给
        //         SDK 与各 OpenCV 码，读取 2x2 四格各自被判成 B/G/R 中的哪个，
        //         排列完全相同的即为正确映射（不受光照与图像内容影响）。
        //   结果（SDK 像素类型 -> OpenCV 码）：
        //         BayerGB8 -> COLOR_BayerGR2BGR   (原代码误写成 GB2BGR，红蓝反了)
        //         BayerGR8 -> COLOR_BayerGB2BGR
        //         BayerBG8 -> COLOR_BayerRG2BGR
        //         BayerRG8 -> COLOR_BayerBG2BGR
        //   规律：OpenCV 的名字正好是 SDK 名字里 G 与 B 对调，改这张表前
        //         务必用 tools/probe_bayer_map 重测一遍。
        static int opencvConvFor(enum MvGvspPixelType t) {
            switch (t) {
                case PixelType_Gvsp_BayerGB8: return cv::COLOR_BayerGR2BGR;
                case PixelType_Gvsp_BayerGR8: return cv::COLOR_BayerGB2BGR;
                case PixelType_Gvsp_BayerBG8: return cv::COLOR_BayerRG2BGR;
                case PixelType_Gvsp_BayerRG8: return cv::COLOR_BayerBG2BGR;
                case PixelType_Gvsp_Mono8:    return cv::COLOR_GRAY2BGR;
                default:                      return -1;
            }
        }

        class HikCameraNode : public rclcpp::Node {
    public:
        explicit HikCameraNode(const rclcpp::NodeOptions &options) : Node("hik_camera", options) {
            RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

            MV_CC_DEVICE_INFO_LIST device_list{};
            // enum device
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
            RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);
            // 打印相机型号：判断"转换能不能交给相机做"的关键前提。
            // 单色机型只有 Bayer/Mono 输出是正常的；彩色机型只出 Bayer 则说明
            // 它的 ISP 颜色通路没被启用或当前 SDK 版本访问不到。
            // 注意必须先判 nDeviceNum：没有相机时 pDeviceInfo[0] 是未定义的，
            // 直接解引用会读到野指针（之前这里就在判空之前访问了它）。
            if (device_list.nDeviceNum > 0 && device_list.pDeviceInfo[0] != nullptr) {
                const auto *dev = device_list.pDeviceInfo[0];
                if (dev->nTLayerType == MV_USB_DEVICE) {
                    RCLCPP_INFO(this->get_logger(), "Camera(USB3): model='%s' vendor='%s'",
                                dev->SpecialInfo.stUsb3VInfo.chModelName,
                                dev->SpecialInfo.stUsb3VInfo.chVendorName);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Camera(model='%s')",
                                dev->SpecialInfo.stGigEInfo.chModelName);
                }
            }

            while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "No camera found!");
                RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
            }
            // 上面可能是因 rclcpp::ok() 变 false 而退出循环，此时 pDeviceInfo[0] 不可用
            if (!rclcpp::ok() || device_list.nDeviceNum == 0 ||
                device_list.pDeviceInfo[0] == nullptr) {
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

            // Bayer->BGR 的转换后端：opencv(默认) / sdk，纯粹为了能 A/B 对照。
            // opencv 单核约 250 MPix/s，sdk 约 22 MPix/s/核，实测差 5 倍以上。
            {
                rcl_interfaces::msg::ParameterDescriptor d;
                d.description = "Bayer->BGR 转换后端: opencv(快,默认) / sdk(与旧行为一致,用于对照)";
                const std::string backend = this->declare_parameter("demosaic_backend", std::string("opencv"), d);
                use_opencv_demosaic_ = (backend != "sdk");

                // OpenCV 的线程池是自旋等待的：整机上 dart_aim_node 已经吃掉 4~5 核，
                // 这里再开 8 个线程只会互相空转，把 %CPU 抬高而墙钟时间并不变好。
                // 所以限制成 1 线程：转换本来就只要几毫秒，省下的核留给推理。
                d.description = "OpenCV 去马赛克使用的线程数（0=用 OpenCV 默认）";
                const int ocv_threads = this->declare_parameter("demosaic_threads", 1, d);
                if (ocv_threads > 0) cv::setNumThreads(ocv_threads);
                RCLCPP_INFO(this->get_logger(),
                            "Bayer->BGR 转换后端: %s, OpenCV 线程数: %d",
                            use_opencv_demosaic_ ? "opencv" : "sdk", cv::getNumThreads());
            }
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
                        const size_t need = static_cast<size_t>(image_msg_.width) * image_msg_.height * 3;
                        // 尺寸不变就别 resize：resize 变大时会把新增部分清零，
                        // 分辨率固定的情况下这每帧都是白做的 memset
                        if (image_msg_.data.size() != need) {
                            image_msg_.data.resize(need);
                        }

                        // 三条路径扁平化分发，避免深层嵌套：
                        //   1) 相机已输出目标格式 -> 直拷
                        //   2) Bayer/Mono 且 OpenCV 能处理 -> OpenCV 去马赛克
                        //   3) 其它 -> SDK 转换
                        const auto src_type = static_cast<enum MvGvspPixelType>(
                            out_frame.stFrameInfo.enPixelType);
                        const int cv_code = use_opencv_demosaic_ ? opencvConvFor(src_type) : -1;
                        bool frame_ok = true;

                        if (src_type == convert_param_.enDstPixelType) {
                            // 相机已经输出目标格式：只搬内存，不做任何像素运算
                            const size_t n = std::min(need,
                                                      static_cast<size_t>(out_frame.stFrameInfo.nFrameLen));
                            std::memcpy(image_msg_.data.data(), out_frame.pBufAddr, n);
                            if (!zero_copy_logged_) {
                                zero_copy_logged_ = true;
                                RCLCPP_INFO(this->get_logger(),
                                            "取帧走直拷路径：相机 ISP 已输出 %s，CPU 不做像素运算",
                                            image_msg_.encoding.c_str());
                            }
                        } else if (cv_code >= 0 &&
                                   out_frame.stFrameInfo.nWidth >= 4 &&
                                   out_frame.stFrameInfo.nHeight >= 4) {
                            // Bayer/Mono -> OpenCV 去马赛克（SIMD，比 SDK 快约 5 倍）
                            // 直接在 SDK 缓存和 ROS 消息上开视图，全程不额外分配内存
                            cv::Mat bayer(static_cast<int>(out_frame.stFrameInfo.nHeight),
                                          static_cast<int>(out_frame.stFrameInfo.nWidth),
                                          CV_8UC1, out_frame.pBufAddr,
                                          static_cast<size_t>(out_frame.stFrameInfo.nWidth));
                            cv::Mat bgr(image_msg_.height, image_msg_.width, CV_8UC3,
                                        image_msg_.data.data(),
                                        static_cast<size_t>(image_msg_.step));
                            const auto t0 = std::chrono::steady_clock::now();
                            if (cv_code == cv::COLOR_GRAY2BGR) {
                                cv::cvtColor(bayer, bgr, cv_code);   // 灰度没有马赛克排列
                            } else {
                                cv::demosaicing(bayer, bgr, cv_code);
                            }
                            // 注意：OpenCV 的去马赛核（COLOR_Bayer*2BGR / COLOR_GRAY2BGR）
                            // 产出的**永远是 BGR 通道顺序**。如果配置里要的是 RGB8，
                            // 这里的字节其实是 BGR，encoding 必须如实写 bgr8，
                            // 否则下游按 encoding 解释就会红蓝颠倒。
                            // 这里不改字节顺序（那要多一遍全图转换，正是要省掉的 CPU）；
                            // 需要 RGB8 的下游用 cv_bridge 转一次即可。
                            image_msg_.encoding = "bgr8";
                            if (!bgr_order_warned_) {
                                bgr_order_warned_ = true;
                                if (PixelType_Gvsp_RGB8_Packed == convert_param_.enDstPixelType) {
                                    RCLCPP_WARN(this->get_logger(),
                                                "相机不支持 ISP 直出 BGR8，已在 CPU 上用 OpenCV 去马赛克；"
                                                "OpenCV 只能产出 BGR，故实际 encoding 为 bgr8"
                                                "（pixel_format: RGB8 在这条路径上不生效，"
                                                "需要 RGB 顺序请在下游用 cv_bridge 转换）");
                                }
                            }
                            const double ms = std::chrono::duration<double, std::milli>(
                                                  std::chrono::steady_clock::now() - t0).count();
                            demosaic_ms_avg_ = demosaic_ms_avg_ * 0.9 + ms * 0.1;
                            if (!demosaic_logged_) {
                                demosaic_logged_ = true;
                                RCLCPP_INFO(this->get_logger(),
                                            "取帧走 OpenCV 路径 %s -> bgr8（单帧 %.2f ms，滑动平均 %.2f ms）",
                                            pixelFormatName(src_type), ms, demosaic_ms_avg_);
                            }
                        } else {
                            // 回退：OpenCV 处理不了的格式才用 SDK 转换
                            if (!convert_warned_) {
                                convert_warned_ = true;
                                RCLCPP_WARN(this->get_logger(),
                                            "相机输出 %s(0x%08x)，%s，走 SDK 转换（比 OpenCV 慢约 5 倍）",
                                            pixelFormatName(src_type),
                                            static_cast<unsigned int>(src_type),
                                            use_opencv_demosaic_ ? "OpenCV 处理不了" : "参数指定用 SDK");
                            }
                            convert_param_.pDstBuffer = image_msg_.data.data();
                            convert_param_.nDstBufferSize = image_msg_.data.size();
                            convert_param_.pSrcData = out_frame.pBufAddr;
                            convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
                            convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
                            convert_param_.nWidth = out_frame.stFrameInfo.nWidth;
                            convert_param_.nHeight = out_frame.stFrameInfo.nHeight;
                            if (MV_OK != MV_CC_ConvertPixelType(camera_handle_, &convert_param_)) {
                                frame_ok = false;
                                if (!convert_failed_) {
                                    convert_failed_ = true;
                                    RCLCPP_ERROR(this->get_logger(),
                                                 "MV_CC_ConvertPixelType failed (src=%#x dst=%#x %ux%u), dropping frames",
                                                 static_cast<unsigned int>(convert_param_.enSrcPixelType),
                                                 static_cast<unsigned int>(convert_param_.enDstPixelType),
                                                 convert_param_.nWidth, convert_param_.nHeight);
                                }
                            } else {
                                convert_failed_ = false;
                            }
                        }

                        if (!frame_ok) {
                            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                            continue;
                        }

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
        // 显式设置 ROI 与像素格式
        // 不显式设置的话，输出格式取决于相机掉电前的状态：换相机/相机复位后分辨率变了，
        // 下游 dart_aim_node 的居中裁剪区域和按分辨率标定的偏移表就全偏了
        void configureImageFormat() {
            rcl_interfaces::msg::ParameterDescriptor desc;

            desc.description = "图像宽度，0 = 使用相机支持的最大值";
            const int cfg_width = this->declare_parameter("image_width", 0, desc);
            desc.description = "图像高度，0 = 使用相机支持的最大值";
            const int cfg_height = this->declare_parameter("image_height", 0, desc);
            // ── ROI 上移：把「裁剪」交给相机做，而不是让 CPU 收到全幅再裁 ──
            // 下游 dart_aim_node 本来就只取中心 crop_height 行。以前这里是相机吐
            // 1280x1024 全幅，主机再裁成 1280x720，等于白白传了 304 行无用数据、
            // 再在 CPU 上对全幅做去马赛克。现在让相机只输出需要的那 720 行：
            //   - USB 传输量降 30%
            //   - 主机去马赛克的像素数降 30%（实测 4.47ms -> 3.06ms，单线程）
            //   - 帧率还从 113fps 涨到 134fps
            // ROI 坐标与原来的软裁剪完全一致（偏移量取偶数，保持 Bayer CFA 相位，
            // 已实测逐像素一致），所以标定好的偏移表和模型输入都不用重做。
            desc.description = "ROI 宽度，0 = 不限（用 image_width）";
            const int roi_width = this->declare_parameter("roi_width", 0, desc);
            desc.description = "ROI 高度，0 = 不限（用 image_height）。设成下游实际只用的高度即可省掉无用行的传输与去马赛克";
            const int roi_height = this->declare_parameter("roi_height", 0, desc);
            desc.description = "ROI 水平偏移(像素)，-1 = 自动居中";
            const int roi_offset_x = this->declare_parameter("roi_offset_x", -1, desc);
            desc.description = "ROI 垂直偏移(像素)，-1 = 自动居中。会自动向下取偶数以保持 Bayer 相位";
            const int roi_offset_y = this->declare_parameter("roi_offset_y", -1, desc);

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

            // ROI 只能比传感器可视区域小。先把偏移清零、再按全幅设尺寸，
            // 否则从「上次的 ROI 状态」直接改尺寸会被相机的取值范围挡住。
            MV_CC_SetIntValue(camera_handle_, "OffsetX", 0);
            MV_CC_SetIntValue(camera_handle_, "OffsetY", 0);
            MV_CC_SetIntValue(camera_handle_, "Width", width);
            MV_CC_SetIntValue(camera_handle_, "Height", height);

            if (roi_width > 0 || roi_height > 0) {
                unsigned int rw = (roi_width > 0) ? static_cast<unsigned int>(roi_width) : width;
                unsigned int rh = (roi_height > 0) ? static_cast<unsigned int>(roi_height) : height;
                // 相机自身会再夹一次范围，这里先夹一次保证居中计算用的是合法值
                rw = std::min(std::max(rw, width_range.nMin), width);
                rh = std::min(std::max(rh, height_range.nMin), height);

                // 居中偏移向下取偶：Bayer 的 CFA 相位由 (x+y)%2 决定，
                // 奇数偏移会让整幅图相位翻转、去马赛克码得跟着换。
                // 本机相机的 Offset 步进本身就是 4，但别依赖这个。
                unsigned int ox = (roi_offset_x >= 0) ? static_cast<unsigned int>(roi_offset_x)
                                                      : (width - rw) / 2;
                unsigned int oy = (roi_offset_y >= 0) ? static_cast<unsigned int>(roi_offset_y)
                                                      : (height - rh) / 2;
                ox -= (ox % 2);
                oy -= (oy % 2);

                if (MV_OK == MV_CC_SetIntValue(camera_handle_, "Width", rw) &&
                    MV_OK == MV_CC_SetIntValue(camera_handle_, "Height", rh) &&
                    MV_OK == MV_CC_SetIntValue(camera_handle_, "OffsetX", ox) &&
                    MV_OK == MV_CC_SetIntValue(camera_handle_, "OffsetY", oy)) {
                    RCLCPP_INFO(this->get_logger(),
                                "相机硬件 ROI: %ux%u @ (%u,%u)（相机只输出下游要用的区域，"
                                "省掉无用行的 USB 传输与 CPU 去马赛克）",
                                rw, rh, ox, oy);
                    width = rw;
                    height = rh;
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "设置硬件 ROI 失败，回退到全幅 %ux%u", width, height);
                }
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
            // 必须用专用的 SetPixelFormat，不能用 MV_CC_SetEnumValue("PixelFormat")。
            //
            // 实测（本机海康相机）：SetEnumValue("PixelFormat", BGR8) 返回 MV_OK，
            // 但回读 + 实际收到的帧都是 BayerGB8（0x0108000A），也就是设置根本没生效。
            // 相机会继续吐原始 Bayer 马赛克数据，SDK 只好在 X5 的 CPU 上做去马赛克 ->
            // BGR8，那个节点 CPU 一度占到 68%，而这些活本该由相机自带的 ISP 完成。
            //
            // SDK 文档也写明：要设的值必须在 Get 返回的 nSupportValue 列表里，否则无效。
            // 所以先问支持列表，再校验，再设。
            MVCC_ENUMVALUE fmt_support{};
            bool want_supported = false;
            if (MV_OK == MV_CC_GetPixelFormat(camera_handle_, &fmt_support)) {
                std::string supported;
                for (unsigned int k = 0; k < fmt_support.nSupportedNum; ++k) {
                    const unsigned int v = fmt_support.nSupportValue[k];
                    if (v == static_cast<unsigned int>(convert_param_.enDstPixelType)) want_supported = true;
                    {
                        char tmp[64];
                        snprintf(tmp, sizeof(tmp), " %s(0x%08x)",
                                 pixelFormatName(static_cast<enum MvGvspPixelType>(v)), v);
                        supported += tmp;
                    }
                }
                RCLCPP_INFO(this->get_logger(), "相机支持的像素格式:%s", supported.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "MV_CC_GetPixelFormat failed, will still try to set");
            }

            if (!want_supported && fmt_support.nSupportedNum > 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "相机不支持 %s 格式，无法把转换交给相机 ISP；"
                             "SDK 只能在 X5 的 CPU 上转换，CPU 占用会很高",
                             image_msg_.encoding.c_str());
            }

            int set_ret = MV_CC_SetPixelFormat(
                camera_handle_, static_cast<unsigned int>(convert_param_.enDstPixelType));
            if (MV_OK != set_ret) {
                RCLCPP_ERROR(this->get_logger(), "MV_CC_SetPixelFormat failed (%#x), keep camera default",
                             static_cast<unsigned int>(set_ret));
            }

            // 用专用接口回读，确认真的生效了（这一步是本问题的关键判据）
            MVCC_ENUMVALUE pixel_format_now{};
            if (MV_OK == MV_CC_GetPixelFormat(camera_handle_, &pixel_format_now)) {
                camera_output_pixel_type_ = static_cast<enum MvGvspPixelType>(pixel_format_now.nCurValue);
                if (camera_output_pixel_type_ == convert_param_.enDstPixelType) {
                    RCLCPP_INFO(this->get_logger(),
                                "相机 ISP 将直接输出 %s，取帧时不再做 CPU 像素转换",
                                image_msg_.encoding.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "相机实际输出 %s(0x%08x)，与目标 %s(0x%08x) 不一致，取帧时仍需 CPU 转换",
                                pixelFormatName(camera_output_pixel_type_),
                                static_cast<unsigned int>(camera_output_pixel_type_),
                                image_msg_.encoding.c_str(),
                                static_cast<unsigned int>(convert_param_.enDstPixelType));
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "MV_CC_GetPixelFormat failed, assume conversion may be needed");
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
        // 相机实际输出的像素格式（读回自相机），用于判断能不能跳过 CPU 转换
        enum MvGvspPixelType camera_output_pixel_type_ = PixelType_Gvsp_Undefined;
        bool zero_copy_logged_ = false;  // 直拷路径提示只记一次
        bool convert_warned_ = false;    // 回退路径警告只记一次
        bool demosaic_logged_ = false;   // OpenCV 转换路径提示只记一次
        bool bgr_order_warned_ = false;  // 「要 RGB8 但实际只能 BGR」提示只记一次
        bool use_opencv_demosaic_ = true;  // 转换后端: true=OpenCV, false=SDK
        double demosaic_ms_avg_ = 0.0;   // 去马赛克耗时滑动平均，用于观察余量
        std::thread capture_thread_;

        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    };
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
