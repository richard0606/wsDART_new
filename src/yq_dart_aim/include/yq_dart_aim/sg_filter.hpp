#pragma once

#include <array>
#include <cstddef>

namespace yq_dart_aim {

// Savitzky-Golay 滤波器（固定窗口 n=7, 多项式阶 p=2）
// 用于对连续帧的目标 X 坐标做时序平滑
// 系数来源：对 7 点 2 阶多项式最小二乘拟合的中心点平滑权重
// 公式：y_smooth = Σ(coeff[i] * x[i]) / denom
class SGFilter7 {
public:
    SGFilter7() { buffer_.fill(0.0f); }

    // 推入一个新值，返回平滑后的结果
    float push(float value) {
        // 移位：丢弃最旧的，追加最新的
        for (size_t i = 0; i < N - 1; i++) {
            buffer_[i] = buffer_[i + 1];
        }
        buffer_[N - 1] = value;

        count_++;
        if (count_ < N) {
            // 窗口未满，直接返回最新值
            return value;
        }

        // 应用 SG 系数：[-2, 3, 6, 7, 6, 3, -2] / 21
        float sum = 0.0f;
        for (size_t i = 0; i < N; i++) {
            sum += COEFF[i] * buffer_[i];
        }
        return sum / DENOM;
    }

    void reset() {
        buffer_.fill(0.0f);
        count_ = 0;
    }

    bool isReady() const { return count_ >= N; }

private:
    static constexpr size_t N = 7;
    static constexpr std::array<float, N> COEFF = {-2.0f, 3.0f, 6.0f, 7.0f, 6.0f, 3.0f, -2.0f};
    static constexpr float DENOM = 21.0f;

    std::array<float, N> buffer_;
    size_t count_ = 0;
};

}  // namespace yq_dart_aim
