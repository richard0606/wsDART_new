#pragma once

#include <opencv2/core.hpp>
#include "common/types.hpp"
#include "common/constants.hpp"

namespace yq_dart_aim {

// 坐标计算参数
struct CalcParams {
    double p_err = defaults::P_ERR;
    double offset_x = defaults::OFFSET_X;
    double offset_y = defaults::OFFSET_Y;
    float aim_threshold = defaults::AIM_THRESHOLD;
};

// 瞄准结果
struct AimResult {
    double err_x = 0.0;        // 原始X误差
    double err_y = 0.0;        // 原始Y误差
    double scaled_err_x = 0.0; // 缩放后X误差
    double scaled_err_y = 0.0; // 缩放后Y误差
    float aim_information = 0.0f; // 瞄准信息
    bool valid = false;        // 是否有效
};

class CoordinateCalculator {
public:
    CoordinateCalculator() = default;

    // 计算瞄准误差
    AimResult calculate(const cv::Point2f& target,
                       const cv::Point& image_center,
                       const CalcParams& params);

    // 根据目标类型和飞镖编号更新偏移量
    void updateOffset(const OffsetLookup& lookup, int target_id, int dart_id);

    // 获取当前偏移量
    double getCurrentOffsetX() const { return current_offset_x_; }

private:
    double current_offset_x_ = defaults::OFFSET_X;
};

}  // namespace yq_dart_aim
