#include "yq_dart_aim/image_processor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace yq_dart_aim {

cv::Mat ImageProcessor::cropCenter(const cv::Mat& src, int crop_width, int crop_height) {
    if (src.empty()) return src;
    if (crop_width <= 0 || crop_height <= 0) return src;
    int w = std::min(crop_width, src.cols);
    int h = std::min(crop_height, src.rows);
    int x = (src.cols - w) / 2;
    int y = (src.rows - h) / 2;
    cv::Rect roi(x, y, w, h);
    return src(roi).clone();
}

cv::Mat ImageProcessor::process(const cv::Mat& input, const ImageProcessParams& params) {
    // 裁剪中心区域
    cv::Mat img = cropCenter(input, params.crop_width, params.crop_height);

    // BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 阈值分割
    cv::Mat mask;
    cv::inRange(hsv,
                cv::Scalar(params.h_min, params.s_min, params.v_min),
                cv::Scalar(params.h_max, params.s_max, params.v_max),
                mask);

    // 形态学处理
    mask = applyMorphology(mask, params.morph_kernel_size, params.morph_dilate_kernel_size);

    return mask;
}

cv::Mat ImageProcessor::applyMorphology(const cv::Mat& mask, int open_size, int dilate_size) {
    cv::Mat result = mask;

    // 修正开运算核大小（确保 >=1 且为奇数）
    int ksize = open_size;
    if (ksize < 1) ksize = 1;
    if (ksize % 2 == 0) ksize += 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
    cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);

    // 修正膨胀核大小
    int dksize = dilate_size;
    if (dksize < 1) dksize = 1;
    if (dksize % 2 == 0) dksize += 1;
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dksize, dksize));
    cv::morphologyEx(result, result, cv::MORPH_DILATE, kernel_dilate);

    return result;
}

}  // namespace yq_dart_aim
