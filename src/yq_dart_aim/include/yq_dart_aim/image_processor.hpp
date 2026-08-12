#pragma once

#include <opencv2/core.hpp>
#include "common/types.hpp"
#include "common/constants.hpp"

namespace yq_dart_aim {

// 图像处理参数（从ROS参数缓存）
struct ImageProcessParams {
    int crop_width = defaults::CROP_WIDTH;
    int crop_height = defaults::CROP_HEIGHT;
    int h_min = defaults::H_MIN;
    int s_min = defaults::S_MIN;
    int v_min = defaults::V_MIN;
    int h_max = defaults::H_MAX;
    int s_max = defaults::S_MAX;
    int v_max = defaults::V_MAX;
    int morph_kernel_size = defaults::MORPH_KERNEL_SIZE;
    int morph_dilate_kernel_size = defaults::MORPH_DILATE_KERNEL_SIZE;
    bool circle_mask = defaults::CIRCLE_MASK;
};

class ImageProcessor {
public:
    ImageProcessor() = default;

    // 处理图像：裁剪 -> HSV -> 二值化 -> 形态学
    cv::Mat process(const cv::Mat& input, const ImageProcessParams& params);

    // 裁剪中心区域
    cv::Mat cropCenter(const cv::Mat& src, int width, int height);

private:
    // 形态学处理：开运算去噪 + 膨胀连接
    cv::Mat applyMorphology(const cv::Mat& mask, int open_size, int dilate_size);
};

}  // namespace yq_dart_aim
