#include "yq_dart_aim/serial_manager.hpp"
#include "yq_dart_aim/common/constants.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace yq_dart_aim {

static speed_t baudToSpeed(int baud) {
    switch (baud) {
        case 0: return B0;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
        default:
#ifdef B115200
            return B115200;
#else
            return B9600;
#endif
    }
}

SerialManager::SerialManager() = default;

void SerialManager::setLogCallback(LogCallback callback) {
    log_callback_ = std::move(callback);
}

void SerialManager::log(LogLevel level, const std::string& message) {
    if (log_callback_) {
        log_callback_(level, message);
    }
}

SerialManager::~SerialManager() {
    stopMonitoring();
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& kv : serial_ports_) {
        if (kv.second.fd >= 0) {
            close(kv.second.fd);
            kv.second.fd = -1;
        }
    }
    serial_ports_.clear();
}

void SerialManager::initialize(const std::string& port, int baud_rate) {
    configured_port_ = port;
    baud_rate_ = baud_rate;
}

void SerialManager::setEnabled(bool enabled) {
    enabled_.store(enabled);
    if (!enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& kv : serial_ports_) {
            if (kv.second.fd >= 0) {
                close(kv.second.fd);
                kv.second.fd = -1;
                kv.second.active = false;
            }
        }
    }
}

bool SerialManager::isEnabled() const {
    return enabled_.load();
}

void SerialManager::startMonitoring() {
    if (monitor_thread_.joinable() || read_thread_.joinable()) return;

    stop_.store(false);

    // 先尝试打开配置的串口
    {
        std::lock_guard<std::mutex> lock(mutex_);
        openPort(configured_port_);
    }

    // 串口监控线程（超时检测 + 自动重连）
    monitor_thread_ = std::thread([this]() {
        while (!stop_.load()) {
            if (enabled_.load()) {
                auto now = std::chrono::steady_clock::now();
                bool has_active_port = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    for (auto& kv : serial_ports_) {
                        if (kv.second.fd >= 0) {
                            has_active_port = true;
                            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                now - kv.second.last_rx_time).count();
                            if (elapsed > SERIAL_RX_TIMEOUT.count()) {
                                close(kv.second.fd);
                                kv.second.fd = -1;
                                kv.second.active = false;
                                has_active_port = false;
                            }
                        }
                    }
                }
                if (!has_active_port) {
                    auto available = scanAvailableACMPorts();
                    if (!available.empty()) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        for (const auto& port_path : available) {
                            if (openPort(port_path)) break;
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // 串口读取线程
    read_thread_ = std::thread([this]() {
        uint8_t buf[256];
        while (!stop_.load()) {
            std::vector<std::string> to_close;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (auto& kv : serial_ports_) {
                    SerialPortState& port = kv.second;
                    if (port.fd < 0) continue;
                    ssize_t n = read(port.fd, buf, sizeof(buf));
                    if (n > 0) {
                        // 记录 hex 串用于调试
                        char hex_str[512];
                        int offset = 0;
                        for (ssize_t i = 0; i < n && i < 256; i++) {
                            offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, "%02X ", buf[i]);
                        }
                        last_rx_hex_ = hex_str;

                        for (ssize_t i = 0; i < n; i++) {
                            port.rx_buffer.push_back(buf[i]);
                        }
                        parseData(port, kv.first);
                        port.last_rx_time = std::chrono::steady_clock::now();
                    } else if (n < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                        if (errno == EIO || errno == ENODEV || errno == EBADF) {
                            to_close.push_back(kv.first);
                        }
                    }
                }
            }
            for (const auto& path : to_close) {
                std::lock_guard<std::mutex> lock(mutex_);
                auto it = serial_ports_.find(path);
                if (it != serial_ports_.end()) {
                    if (it->second.fd >= 0) close(it->second.fd);
                    it->second.fd = -1;
                    it->second.active = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
}

void SerialManager::stopMonitoring() {
    stop_.store(true);
    if (monitor_thread_.joinable()) monitor_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
}

void SerialManager::send(float err_pix, float aim_information) {
    if (!enabled_.load()) return;
    std::lock_guard<std::mutex> lock(mutex_);

    DartAimPacket pkt{};
    pkt.err_of_pix = err_pix;
    pkt.keep_1 = 0.0f;
    pkt.aim_information = aim_information;
    pkt.keep_3 = 0.0f;

    unsigned char head = SERIAL_HEADER;
    unsigned char tail = SERIAL_TAIL;
    const unsigned char* payload = reinterpret_cast<const unsigned char*>(&pkt);
    size_t payload_size = sizeof(pkt);

    // 计算校验和
    uint8_t checksum = 0;
    for (size_t i = 0; i < payload_size; ++i) {
        checksum = static_cast<uint8_t>(checksum + payload[i]);
    }

    for (auto& kv : serial_ports_) {
        if (kv.second.fd < 0) continue;

        ssize_t w = write(kv.second.fd, &head, 1);
        if (w != 1) {
            log(LogLevel::DEBUG, "uart write head failed on " + kv.first);
            continue;
        }

        ssize_t written = 0;
        while (written < static_cast<ssize_t>(payload_size)) {
            w = write(kv.second.fd, payload + written, payload_size - written);
            if (w <= 0) {
                log(LogLevel::DEBUG, "uart write payload failed on " + kv.first);
                break;
            }
            written += w;
        }

        w = write(kv.second.fd, &checksum, 1);
        if (w != 1) {
            log(LogLevel::DEBUG, "uart write checksum failed on " + kv.first);
            continue;
        }

        w = write(kv.second.fd, &tail, 1);
        if (w != 1) {
            log(LogLevel::DEBUG, "uart write tail failed on " + kv.first);
        }
    }
}

bool SerialManager::getReceivedData(int& target, int& dart_id, float& encoder_angle) {
    std::lock_guard<std::mutex> lock(mutex_);
    target = current_target_;
    dart_id = current_dart_id_;
    encoder_angle = current_encoder_angle_;
    return true;
}

std::string SerialManager::getLastRxHex() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_rx_hex_;
}

bool SerialManager::openPort(const std::string& path) {
    auto it = serial_ports_.find(path);
    if (it != serial_ports_.end() && it->second.fd >= 0) return true;

    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    speed_t speed = baudToSpeed(baud_rate_);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10;
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return false;
    }

    SerialPortState state;
    state.fd = fd;
    state.active = true;
    state.last_rx_time = std::chrono::steady_clock::now();
    serial_ports_[path] = state;
    return true;
}

void SerialManager::closePort(const std::string& path) {
    auto it = serial_ports_.find(path);
    if (it != serial_ports_.end() && it->second.fd >= 0) {
        close(it->second.fd);
        it->second.fd = -1;
        it->second.active = false;
    }
}

std::vector<std::string> SerialManager::scanAvailableACMPorts() {
    std::vector<std::string> result;
    DIR* dir = opendir("/dev");
    if (!dir) return result;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0) {
            result.push_back(std::string("/dev/") + entry->d_name);
        }
    }
    closedir(dir);
    std::sort(result.begin(), result.end());
    return result;
}

void SerialManager::parseData(SerialPortState& port, const std::string& port_name) {
    // 接收包格式: 0xAA + VisionSend_s(12字节) + checksum(1字节) + 0x55
    constexpr size_t packet_len = 1 + sizeof(VisionSendPacket) + 1 + 1;
    size_t ri = 0;
    size_t buf_size = port.rx_buffer.size();

    while (ri + packet_len <= buf_size) {
        // 查找包头 0xAA
        size_t header_idx = SIZE_MAX;
        for (size_t i = ri; i < ri + packet_len && i < buf_size; ++i) {
            if (port.rx_buffer[i] == SERIAL_HEADER) {
                header_idx = i;
                break;
            }
        }

        if (header_idx == SIZE_MAX) break;

        // 跳过包头前的无效字节
        if (header_idx > ri) {
            size_t compact_sz = buf_size - header_idx;
            memmove(&port.rx_buffer[ri], &port.rx_buffer[header_idx], compact_sz);
            buf_size = ri + compact_sz;
        }

        if (ri + packet_len > buf_size) break;

        // 校验
        uint8_t checksum = 0;
        for (size_t i = 1; i < 1 + sizeof(VisionSendPacket); i++) {
            checksum += port.rx_buffer[ri + i];
        }
        size_t checksum_idx = ri + 1 + sizeof(VisionSendPacket);
        if (port.rx_buffer[checksum_idx] != checksum) {
            char msg[128];
            snprintf(msg, sizeof(msg), "serial checksum mismatch on %s: expected %02X got %02X",
                     port_name.c_str(), checksum, port.rx_buffer[checksum_idx]);
            log(LogLevel::WARN, msg);
            ++ri;
            continue;
        }
        size_t tail_idx = checksum_idx + 1;
        if (port.rx_buffer[tail_idx] != SERIAL_TAIL) {
            char msg[128];
            snprintf(msg, sizeof(msg), "serial tail mismatch on %s: expected %02X got %02X",
                     port_name.c_str(), SERIAL_TAIL, port.rx_buffer[tail_idx]);
            log(LogLevel::WARN, msg);
            ++ri;
            continue;
        }

        // 解析数据
        VisionSendPacket data;
        memcpy(&data, &port.rx_buffer[ri + 1], sizeof(data));
        current_target_ = data.enemy;
        current_dart_id_ = data.cur;
        current_encoder_angle_ = data.encoder_angle;

        ri += packet_len;
    }
    port.rx_buffer.erase(port.rx_buffer.begin(), port.rx_buffer.begin() + ri);
}

}  // namespace yq_dart_aim
