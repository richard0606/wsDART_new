#pragma once

#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include "common/types.hpp"
#include "common/constants.hpp"

namespace yq_dart_aim {

// 目标检测参数
struct DetectParams {
    double min_area = defaults::MIN_AREA;
    double max_area = defaults::MAX_AREA;
    bool circle_mask = defaults::CIRCLE_MASK;
    float target_radius_px = defaults::TARGET_RADIUS_PX;
    float target_height_px = defaults::TARGET_HEIGHT_PX;
};

class TargetDetector {
public:
    TargetDetector() = default;

    // 从二值图中检测目标
    std::vector<TargetInfo> detect(const cv::Mat& mask, const DetectParams& params);

    // 根据当前目标类型选择最终目标
    cv::Point2f selectTarget(const std::vector<TargetInfo>& targets,
                             int current_target, bool is_startup);

private:
    bool single_target_timeout_active_ = false;
    bool startup_single_target_wait_done_ = false;
    std::chrono::steady_clock::time_point single_target_start_;
};

}  // namespace yq_dart_aim
