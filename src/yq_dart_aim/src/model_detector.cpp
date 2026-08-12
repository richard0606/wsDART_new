#include "yq_dart_aim/model_detector.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace yq_dart_aim {

ModelDetector::ModelDetector() = default;

ModelDetector::~ModelDetector() {
    stop_.store(true);
    new_frame_ready_.store(true);  // 唤醒推理线程以便退出
    if (infer_thread_.joinable()) {
        infer_thread_.join();
    }
}

bool ModelDetector::loadModel(const ModelParams& params) {
    params_ = params;

    if (params_.model_path.empty()) {
        return false;
    }

    try {
        // 根据文件扩展名选择加载方式
        std::string ext = params_.model_path.substr(params_.model_path.rfind('.'));
        if (ext == ".onnx") {
            net_ = cv::dnn::readNetFromONNX(params_.model_path);
        } else if (ext == ".caffemodel" || ext == ".prototxt") {
            // Caffe 模型需要 prototxt + caffemodel
            net_ = cv::dnn::readNet(params_.model_path);
        } else {
            // 通用方式（支持 .bin/.xml 等）
            net_ = cv::dnn::readNet(params_.model_path);
        }

        // 设置推理后端（CPU，后续可替换为 BPU）
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        model_loaded_ = true;

        // 启动推理线程
        stop_.store(false);
        infer_thread_ = std::thread(&ModelDetector::inferenceLoop, this);

        return true;
    } catch (const cv::Exception& e) {
        model_loaded_ = false;
        return false;
    }
}

void ModelDetector::setClassNames(const std::vector<std::string>& names) {
    class_names_ = names;
}

bool ModelDetector::isReady() const {
    return model_loaded_;
}

void ModelDetector::submit(const cv::Mat& image) {
    if (!model_loaded_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = image.clone();
    new_frame_ready_.store(true);
}

std::vector<TargetInfo> ModelDetector::getResults() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_results_;
}

void ModelDetector::inferenceLoop() {
    while (!stop_.load()) {
        // 等待新帧
        while (!new_frame_ready_.load() && !stop_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (stop_.load()) break;

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame = latest_frame_;
            new_frame_ready_.store(false);
        }

        if (frame.empty()) continue;

        try {
            // 预处理：resize + 归一化
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0,
                cv::Size(params_.input_width, params_.input_height),
                cv::Scalar(0, 0, 0), true, false);

            net_.setInput(blob);

            // 推理
            cv::Mat output = net_.forward();

            // 后处理
            auto results = postprocess(output, frame.cols, frame.rows);

            // NMS
            results = nms(results, params_.nms_threshold);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_results_ = std::move(results);
            }
        } catch (const cv::Exception& e) {
            // 推理失败，保留上一次结果
        }
    }
}

std::vector<TargetInfo> ModelDetector::postprocess(const cv::Mat& output, int img_w, int img_h) {
    std::vector<TargetInfo> results;

    // YOLOv5/v8 通用后处理
    // output shape: [1, num_detections, 5 + num_classes] 或 [1, num_classes + 4, num_detections]
    cv::Mat det;

    // 判断输出格式
    if (output.dims == 3) {
        int rows = output.size[1];
        int cols = output.size[2];

        // 判断是 [1, N, 85] 还是 [1, 85, N] 格式
        if (cols > rows) {
            // [1, N, 85] 格式（YOLOv5 ONNX）
            det = output.reshape(1, rows);
        } else {
            // [1, 85, N] 格式（YOLOv8 ONNX）→ 转置
            cv::Mat transposed;
            cv::transpose(output.reshape(1, output.size[1]), transposed);
            det = transposed;
        }
    } else {
        return results;
    }

    int num_detections = det.rows;
    int num_elements = det.cols;

    // 至少需要 5 列：x, y, w, h, conf
    if (num_elements < 5) return results;

    int num_classes = num_elements - 4;

    for (int i = 0; i < num_detections; i++) {
        const float* row = det.ptr<float>(i);

        // YOLOv5 格式：[cx, cy, w, h, conf, class_scores...]
        // 或 YOLOv8 格式：[cx, cy, w, h, class_scores...]
        float conf;
        int class_id = 0;

        if (num_classes == 1) {
            // YOLOv5: [cx, cy, w, h, objectness, class_score]
            conf = row[4];
        } else {
            // 找最大类别分数
            float max_class_score = 0;
            for (int c = 0; c < num_classes; c++) {
                if (row[4 + c] > max_class_score) {
                    max_class_score = row[4 + c];
                    class_id = c;
                }
            }
            conf = row[4] * max_class_score;  // YOLOv5: objectness * class_score
            // 如果是 YOLOv8 格式（无 objectness），直接用 class_score
            if (conf == 0) conf = max_class_score;
        }

        if (conf < params_.conf_threshold) continue;

        float cx = row[0];
        float cy = row[1];
        float w = row[2];
        float h = row[3];

        // 转换为像素坐标（输入已经是裁剪后的图，坐标直接可用）
        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;

        // 缩放到原图尺寸
        float scale_x = static_cast<float>(img_w) / params_.input_width;
        float scale_y = static_cast<float>(img_h) / params_.input_height;

        cv::Rect bbox(
            static_cast<int>(x1 * scale_x),
            static_cast<int>(y1 * scale_y),
            static_cast<int>(w * scale_x),
            static_cast<int>(h * scale_y)
        );

        // 边界裁剪
        bbox &= cv::Rect(0, 0, img_w, img_h);

        TargetInfo info;
        info.center = cv::Point2f(
            std::round((cx * scale_x) * 100.0f) / 100.0f,
            std::round((cy * scale_y) * 100.0f) / 100.0f
        );
        info.area = bbox.area();
        info.score = conf;
        info.class_id = class_id;
        info.confidence = conf;
        info.bbox = bbox;

        results.push_back(info);
    }

    return results;
}

std::vector<TargetInfo> ModelDetector::nms(std::vector<TargetInfo>& detections, float threshold) {
    if (detections.empty()) return {};

    // 按置信度降序排序
    std::sort(detections.begin(), detections.end(),
        [](const TargetInfo& a, const TargetInfo& b) {
            return a.confidence > b.confidence;
        });

    std::vector<bool> suppressed(detections.size(), false);
    std::vector<TargetInfo> result;

    for (size_t i = 0; i < detections.size(); i++) {
        if (suppressed[i]) continue;
        result.push_back(detections[i]);

        for (size_t j = i + 1; j < detections.size(); j++) {
            if (suppressed[j]) continue;
            if (detections[i].class_id != detections[j].class_id) continue;

            // IoU 计算
            cv::Rect inter = detections[i].bbox & detections[j].bbox;
            float inter_area = inter.area();
            float union_area = detections[i].area + detections[j].area - inter_area;
            float iou = (union_area > 0) ? inter_area / union_area : 0;

            if (iou > threshold) {
                suppressed[j] = true;
            }
        }
    }

    return result;
}

}  // namespace yq_dart_aim
