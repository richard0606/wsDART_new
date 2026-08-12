#pragma once

#include <string>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include "common/types.hpp"

namespace yq_dart_aim {

// 串口状态
struct SerialPortState {
    int fd = -1;
    std::vector<uint8_t> rx_buffer;
    std::chrono::steady_clock::time_point last_rx_time;
    bool active = false;
};

class SerialManager {
public:
    SerialManager();
    ~SerialManager();

    // 初始化串口
    void initialize(const std::string& port, int baud_rate);

    // 发送瞄准数据
    void send(float err_pix, float aim_information);

    // 获取接收到的数据（线程安全）
    bool getReceivedData(int& target, int& dart_id, float& encoder_angle);

    // 启用/禁用串口
    void setEnabled(bool enabled);
    bool isEnabled() const;

    // 启动/停止监控线程
    void startMonitoring();
    void stopMonitoring();

    // 获取串口调试信息发布器的 hex 字符串
    std::string getLastRxHex() const;

private:
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

    std::string configured_port_;
    int baud_rate_ = 115200;
};

}  // namespace yq_dart_aim
