#include "yq_dart_aim/target_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace yq_dart_aim {

std::vector<TargetInfo> TargetDetector::detect(const cv::Mat& mask, const cv::Mat& gray, const DetectParams& params) {
    std::vector<TargetInfo> valid_contours;

    // 查找外轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        // 面积筛选
        if (area < params.min_area) continue;
        if (area > params.max_area) continue;

        // 灰度加权质心法（亚像素精度）
        cv::Rect rect = cv::boundingRect(c);
        double sum_w = 0.0;
        double sum_x = 0.0;
        double sum_y = 0.0;

        for (int y = rect.y; y < rect.y + rect.height; y++) {
            const uint8_t* mask_row = mask.ptr<uint8_t>(y);
            const uint8_t* gray_row = gray.ptr<uint8_t>(y);
            for (int x = rect.x; x < rect.x + rect.width; x++) {
                if (mask_row[x] == 0) continue;
                double w = static_cast<double>(gray_row[x]);
                sum_w += w;
                sum_x += w * x;
                sum_y += w * y;
            }
        }

        if (sum_w == 0) continue;
        float cx = static_cast<float>(std::round(sum_x / sum_w * 100.0) / 100.0);
        float cy = static_cast<float>(std::round(sum_y / sum_w * 100.0) / 100.0);
        cv::Point2f center(cx, cy);

        // 圆形匹配评分（可选）
        double score = std::numeric_limits<double>::max();
        if (params.circle_mask) {
            cv::Point2f enclosing_center;
            float enclosing_radius = 0.0f;
            cv::minEnclosingCircle(c, enclosing_center, enclosing_radius);
            double radius_diff = std::fabs(static_cast<double>(enclosing_radius - params.target_radius_px));
            double height_diff = std::fabs(static_cast<double>(rect.height - params.target_height_px));
            score = radius_diff + height_diff;
        }

        valid_contours.push_back({center, area, score});
    }

    return valid_contours;
}

cv::Point2f TargetDetector::selectTarget(const std::vector<TargetInfo>& targets,
                                          int current_target, bool /*is_startup*/) {
    cv::Point2f target(-1, -1);

    if (targets.size() >= 2) {
        // 多目标：按 x 坐标排序
        // target=1(前哨站) -> 最左，target=2(基地) -> 最右
        auto sorted = targets;
        std::sort(sorted.begin(), sorted.end(),
            [](const TargetInfo& a, const TargetInfo& b) {
                return a.center.x < b.center.x;
            });
        int idx = (current_target == 2) ? sorted.size() - 1 : 0;
        target = sorted[idx].center;
        single_target_timeout_active_ = false;  // 重置超时标志
    } else if (targets.size() == 1) {
        // 单目标：仅在启动时等待1秒
        if (!single_target_timeout_active_ && !startup_single_target_wait_done_) {
            // 启动时首次检测到单目标，开始等待
            single_target_timeout_active_ = true;
            startup_single_target_wait_done_ = true;
            single_target_start_ = std::chrono::steady_clock::now();
            target = cv::Point2f(-1, -1);  // 暂不发送数据
        } else {
            // 非启动阶段或超时已激活，直接当作基地处理
            target = targets[0].center;
            single_target_timeout_active_ = false;
        }
    } else {
        // 无目标
        single_target_timeout_active_ = false;
        target = cv::Point2f(-1, -1);
    }

    return target;
}

}  // namespace yq_dart_aim
