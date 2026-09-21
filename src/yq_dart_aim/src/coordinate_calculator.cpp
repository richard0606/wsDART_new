#include "yq_dart_aim/coordinate_calculator.hpp"
#include <cmath>

namespace yq_dart_aim {

AimResult CoordinateCalculator::calculate(const cv::Point2f& target,
                                           const cv::Point& image_center,
                                           const CalcParams& params) {
    AimResult result;

    if (target.x < 0) {
        result.valid = false;
        return result;
    }

    // 瞄准中心（图像中心 + 偏移补偿）已由调用方算好传入，这里不能再叠加一次，
    // 否则 offset_x / offset_y 会被放大 2 倍
    // 计算像素误差
    result.err_x = static_cast<double>(target.x - image_center.x);
    result.err_y = static_cast<double>(target.y - image_center.y);

    // 缩放误差
    result.scaled_err_x = result.err_x * params.p_err;
    result.scaled_err_y = result.err_y * params.p_err;

    // aim_information 由调用方根据连续帧逻辑判断
    result.aim_information = 0.0f;
    result.valid = true;

    return result;
}

void CoordinateCalculator::updateOffset(const OffsetLookup& lookup, int target_id, int dart_id) {
    // 限制 ID 范围
    int t = std::max(0, std::min(MAX_TARGET_ID, target_id));
    int d = std::max(0, std::min(MAX_DART_ID, dart_id));
    current_offset_x_ = lookup[t][d];
}

}  // namespace yq_dart_aim
