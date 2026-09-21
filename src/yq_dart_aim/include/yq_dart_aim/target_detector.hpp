#pragma once

#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include "common/types.hpp"
#include "common/constants.hpp"
#include "detector_base.hpp"

namespace yq_dart_aim {

// 目标检测参数
struct DetectParams {
    double min_area = defaults::MIN_AREA;
    double max_area = defaults::MAX_AREA;
};

// HSV 阈值检测器（实现 DetectorBase 接口）
class TargetDetector : public DetectorBase {
public:
    TargetDetector() = default;

    // DetectorBase 接口（HSV 模式下 submit 为空操作，getResults 返回空）
    void submit(const cv::Mat& /*image*/) override {}
    std::vector<TargetInfo> getResults() override { return {}; }
    bool isReady() const override { return true; }
    std::string name() const override { return "hsv"; }

    // 从二值图中检测目标，gray 为灰度图用于亚像素加权质心
    std::vector<TargetInfo> detect(const cv::Mat& mask, const cv::Mat& gray, const DetectParams& params);

    // 根据当前目标类型选择最终目标
    // 说明：启动期的单目标歧义由本类内部状态管理（见 SINGLE_TARGET_WAIT_MS），
    // is_startup 参数当前未被使用
    cv::Point2f selectTarget(const std::vector<TargetInfo>& targets,
                             int current_target, bool is_startup);

private:
    // 启动阶段只看到一个目标时的等待时长：
    // 此时无法区分它是前哨站还是基地，先等一段时间看第二个目标会不会出现；
    // 等待期内不下发目标（返回 (-1,-1)），超时仍只有一个目标则直接按该目标处理
    static constexpr int SINGLE_TARGET_WAIT_MS = 1000;

    bool single_target_timeout_active_ = false;     // 启动等待窗口是否已开始计时
    bool startup_single_target_wait_done_ = false;  // 启动期单目标歧义是否已解除
    std::chrono::steady_clock::time_point single_target_start_;
};

}  // namespace yq_dart_aim
