#include "yq_dart_aim/target_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

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

        TargetInfo info;
        info.center = center;
        info.area = area;
        info.class_id = -1;       // HSV 模式无类别
        info.confidence = 0.0f;   // HSV 模式无置信度
        info.bbox = rect;
        valid_contours.push_back(info);
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
        // 已经能看到两个目标，启动期的单目标歧义自然解除
        single_target_timeout_active_ = false;
        startup_single_target_wait_done_ = true;
    } else if (targets.size() == 1) {
        if (startup_single_target_wait_done_) {
            // 启动期已结束：单目标直接按该目标处理
            target = targets[0].center;
        } else {
            // 启动期只看到一个目标：等待 SINGLE_TARGET_WAIT_MS，
            // 给第二个目标（前哨站/基地的另一侧）出现留出时间
            if (!single_target_timeout_active_) {
                single_target_timeout_active_ = true;
                single_target_start_ = std::chrono::steady_clock::now();
            }
            // 计时只从首次检测到单目标开始，中途目标丢失也不重新计时，保证等待时长有界
            const auto waited_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - single_target_start_).count();
            if (waited_ms < SINGLE_TARGET_WAIT_MS) {
                target = cv::Point2f(-1, -1);  // 等待期内不下发数据
            } else {
                single_target_timeout_active_ = false;
                startup_single_target_wait_done_ = true;
                target = targets[0].center;
            }
        }
    } else {
        // 无目标：等待窗口保持计时状态，目标重新出现时不会重新计时
        target = cv::Point2f(-1, -1);
    }

    return target;
}

}  // namespace yq_dart_aim
