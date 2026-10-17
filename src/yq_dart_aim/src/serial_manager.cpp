#include "yq_dart_aim/serial_manager.hpp"
#include "yq_dart_aim/common/constants.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <dirent.h>
#include <limits.h>
#include <climits>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace yq_dart_aim {

namespace {
// 两个路径是否指向同一个设备。宁可误判成"是"（少开一个口）也不要误判成"不是"
// （把调参口当下位机口用，往里写二进制协议）。
// 比较顺序：字面相同 -> realpath 相同 -> 软链接目标 -> base 名相同。
bool sameDevicePath(const std::string& a, const std::string& b) {
    if (a.empty() || b.empty()) return false;
    if (a == b) return true;

    char ra[PATH_MAX] = {0};
    char rb[PATH_MAX] = {0};
    const bool oka = realpath(a.c_str(), ra) != nullptr;
    const bool okb = realpath(b.c_str(), rb) != nullptr;
    if (oka && okb) return std::string(ra) == std::string(rb);

    auto base = [](const std::string& p) {
        const size_t slash = p.find_last_of('/');
        return (slash == std::string::npos) ? p : p.substr(slash + 1);
    };
    if (base(a) == base(b)) return true;

    // realpath 至少一边失败（设备没插上）：如果是软链接，比链接目标
    auto linkTarget = [](const std::string& p, std::string* out) {
        char buf[PATH_MAX] = {0};
        const ssize_t n = readlink(p.c_str(), buf, sizeof(buf) - 1);
        if (n <= 0) return false;
        *out = std::string(buf, static_cast<size_t>(n));
        return true;
    };
    std::string target;
    if (!oka && linkTarget(a, &target) && base(target) == base(b)) return true;
    if (!okb && linkTarget(b, &target) && base(target) == base(a)) return true;
    return false;
}
}  // namespace

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

void SerialManager::setExcludedPort(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    excluded_port_ = path;
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
                    // 先探测可读性再读：VMIN=1/VTIME=10 时 VTIME 只在收到首字节后才开始计时，
                    // 对端静默时 read() 会无限期阻塞，而这里持着 mutex_，
                    // 会导致 send()/getReceivedData()/超时重连线程全部卡死
                    struct pollfd pfd{};
                    pfd.fd = port.fd;
                    pfd.events = POLLIN;
                    if (poll(&pfd, 1, 0) <= 0) continue;
                    if (!(pfd.revents & POLLIN)) continue;
                    // poll 已确认有数据，read 不会阻塞
                    ssize_t n = read(port.fd, buf, sizeof(buf));
                    if (n > 0) {
                        // 记录 hex 串用于调试（每字节 3 字符 "XX "，末尾需留 NUL）
                        char hex_str[3 * 256 + 1];
                        const int hex_cap = static_cast<int>(sizeof(hex_str)) - 1;
                        int offset = 0;
                        for (ssize_t i = 0; i < n && i < 256 && offset + 3 <= hex_cap; i++) {
                            offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, "%02X ", buf[i]);
                        }
                        hex_str[offset] = '\0';
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

    // 调参口绝不能当下位机口用（详见 setExcludedPort 的说明）。
    // 软链接/别名也要比，否则 /dev/serial/by-id/... 仍会绕过字面比较。
    if (!excluded_port_.empty() && sameDevicePath(path, excluded_port_)) {
        return false;
    }

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

    while (true) {
        const size_t buf_size = port.rx_buffer.size();
        if (buf_size < packet_len) break;   // 攒不够一整包，等下次

        // 在剩余数据里找包头 0xAA（要搜整个缓冲区，不只是首个 packet_len 字节：
        // 垃圾数据可能很长，只看窗口会漏掉窗口之后的包头）
        const auto header = std::find(port.rx_buffer.begin() + ri, port.rx_buffer.end(),
                                      SERIAL_HEADER);
        if (header == port.rx_buffer.end()) {
            // 整个缓冲区都没有包头：前面这些字节不可能是包的一部分。
            // 只保留最后 packet_len-1 个字节（它们可能是被切断的包头）。
            // 必须丢弃，否则对端不按协议发数据时 rx_buffer 会无限增长（内存泄漏）。
            if (port.rx_buffer.size() > packet_len - 1) {
                port.rx_buffer.erase(port.rx_buffer.begin(),
                                     port.rx_buffer.end() - (packet_len - 1));
            }
            return;
        }
        const size_t header_idx = static_cast<size_t>(header - port.rx_buffer.begin());

        // 跳过包头前的垃圾字节：直接推进 ri，最后统一 erase
        if (header_idx > ri) ri = header_idx;

        if (port.rx_buffer.size() - ri < packet_len) break;   // 包被切断，等下次

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

        // 合法性校验：enemy 只能是 0=无目标 / 1=前哨站 / 2=基地，cur 只能是 0~3
        // 越界按脏包丢弃（只记录，不更新状态，保留上一次的有效值）
        if (data.enemy < 0 || data.enemy > MAX_TARGET_ID ||
            data.cur < 0 || data.cur > MAX_DART_ID) {
            char msg[160];
            snprintf(msg, sizeof(msg), "serial payload out of range on %s: enemy=%d cur=%d (dropped)",
                     port_name.c_str(), data.enemy, data.cur);
            log(LogLevel::WARN, msg);
            ri += packet_len;
            continue;
        }

        current_target_ = data.enemy;
        current_dart_id_ = data.cur;
        current_encoder_angle_ = data.encoder_angle;
        has_valid_data_.store(true);

        ri += packet_len;
    }
    port.rx_buffer.erase(port.rx_buffer.begin(), port.rx_buffer.begin() + ri);
}

}  // namespace yq_dart_aim
