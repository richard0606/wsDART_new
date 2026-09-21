#pragma once

#include <array>
#include <cstddef>

namespace yq_dart_aim {

// Savitzky-Golay 滤波器（固定窗口 n=7, 多项式阶 p=1, 取最新点）
// 用于对连续帧的目标 X 坐标做时序平滑
//
// 系数来源：对窗口内 7 个点做 1 阶（线性）最小二乘拟合 y = a + b*t (t = -3..3)，
// 取最新点 t = 3 处的拟合值 a + 3b，即
//     c_i = 1/7 + 3*t_i/28 = (4 + 3*t_i) / 28
//     => COEFF = [-5, -2, 1, 4, 7, 10, 13] / 28
// 特性：对匀速运动的目标零滞后（输出即当前帧的无偏估计），
//       噪声增益 Σc² = 364/784 ≈ 0.46
//
// 换算法的原因：原来用的是 7 点 2 阶拟合的「中心点」系数 [-2,3,6,7,6,3,-2]/21，
// 而滤波是因果的——窗口中心对应的是 3 帧前的采样，输出实际是 x[n-3]，
// 相当于给瞄准引入了约 3 帧（30fps 下约 100ms）的滞后。
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

        // 应用 SG 系数：[-5, -2, 1, 4, 7, 10, 13] / 28
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
    static constexpr std::array<float, N> COEFF = {-5.0f, -2.0f, 1.0f, 4.0f, 7.0f, 10.0f, 13.0f};
    static constexpr float DENOM = 28.0f;

    std::array<float, N> buffer_;
    size_t count_ = 0;
};

}  // namespace yq_dart_aim
