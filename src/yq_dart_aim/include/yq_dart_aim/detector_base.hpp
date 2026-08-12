#pragma once

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include "common/types.hpp"

namespace yq_dart_aim {

// 检测器统一接口
class DetectorBase {
public:
    virtual ~DetectorBase() = default;

    // 提交一帧图像用于检测（异步模式下为非阻塞入队）
    virtual void submit(const cv::Mat& image) = 0;

    // 获取最新检测结果（非阻塞，返回最近一次推理结果）
    virtual std::vector<TargetInfo> getResults() = 0;

    // 模型是否已加载就绪
    virtual bool isReady() const = 0;

    // 获取检测器类型名称
    virtual std::string name() const = 0;
};

}  // namespace yq_dart_aim
