#pragma once

#include <string>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <functional>
#include "common/types.hpp"

namespace yq_dart_aim {

// 串口状态
struct SerialPortState {
    int fd = -1;
    std::vector<uint8_t> rx_buffer;
    std::chrono::steady_clock::time_point last_rx_time;
    bool active = false;
};

// 日志级别
enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR
};

class SerialManager {
public:
    // 日志回调函数类型
    using LogCallback = std::function<void(LogLevel level, const std::string& message)>;

    SerialManager();
    ~SerialManager();

    // 初始化串口
    void initialize(const std::string& port, int baud_rate);

    // 设置日志回调
    void setLogCallback(LogCallback callback);

    // 发送瞄准数据
    void send(float err_pix, float aim_information);

    // 获取接收到的数据（线程安全）
    bool getReceivedData(int& target, int& dart_id, float& encoder_angle);

    // 是否曾经收到过校验通过的包（用于判断下位机有没有给过目标提示）
    bool hasValidData() const { return has_valid_data_.load(); }

    // 启用/禁用串口
    void setEnabled(bool enabled);
    bool isEnabled() const;

    // 启动/停止监控线程
    void startMonitoring();
    void stopMonitoring();

    // 排除掉不属于下位机的设备（自动重连扫描时要跳过）。
    // 调参用的 USB-TTL 也是 /dev/ttyACM*，如果被当成下位机口打开，
    // send() 就会把 16 字节的二进制视觉协议写进调参通道，
    // 而 tune.py 在那边按行读 ASCII —— 两边都会花掉。
    void setExcludedPort(const std::string& path);

    // 获取串口调试信息发布器的 hex 字符串
    std::string getLastRxHex() const;

private:
    void log(LogLevel level, const std::string& message);
    void monitorThread();
    void readThread();
    bool openPort(const std::string& path);
    void closePort(const std::string& path);
    void parseData(SerialPortState& port, const std::string& port_name);
    std::vector<std::string> scanAvailableACMPorts();

    std::map<std::string, SerialPortState> serial_ports_;
    mutable std::mutex mutex_;
    std::atomic<bool> enabled_{true};
    std::atomic<bool> stop_{false};
    std::thread monitor_thread_;
    std::thread read_thread_;

    // 接收到的数据
    int current_target_ = 0;
    int current_dart_id_ = 0;
    float current_encoder_angle_ = 0.0f;
    std::string last_rx_hex_;
    std::atomic<bool> has_valid_data_{false};

    std::string configured_port_;
    int baud_rate_ = 115200;
    // 自动重连时要跳过的设备（调参口），见 setExcludedPort
    std::string excluded_port_;

    // 日志回调
    LogCallback log_callback_;
};

}  // namespace yq_dart_aim
