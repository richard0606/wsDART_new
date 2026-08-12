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

    // 计算中心点（图像中心 + 偏移补偿）
    cv::Point center_img(image_center.x + static_cast<int>(current_offset_x_),
                         image_center.y + static_cast<int>(params.offset_y));

    // 计算像素误差
    result.err_x = static_cast<double>(target.x - center_img.x);
    result.err_y = static_cast<double>(target.y - center_img.y);

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
