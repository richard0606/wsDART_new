#include "yq_dart_aim/param_tuner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <rclcpp/rclcpp.hpp>

#include "yq_dart_aim/param_registry.hpp"

namespace yq_dart_aim {

// ================================================================ ParamValue
namespace {

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

const char* typeName(ParamValue::Type t) {
    switch (t) {
        case ParamValue::Type::kInt: return "int";
        case ParamValue::Type::kDouble: return "double";
        case ParamValue::Type::kBool: return "bool";
        case ParamValue::Type::kString: return "string";
    }
    return "?";
}

ParamValue::Type metaTypeToValueType(ParamType t) {
    switch (t) {
        case ParamType::kInt: return ParamValue::Type::kInt;
        case ParamType::kDouble: return ParamValue::Type::kDouble;
        case ParamType::kBool: return ParamValue::Type::kBool;
        case ParamType::kString: return ParamValue::Type::kString;
    }
    return ParamValue::Type::kDouble;
}

std::string upperCase(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

std::string ParamValue::toJson() const {
    switch (type) {
        case Type::kInt: return std::to_string(i);
        case Type::kBool: return b ? "true" : "false";
        case Type::kString: return "\"" + jsonEscape(s) + "\"";
        case Type::kDouble:
        default: {
            char buf[64];
            snprintf(buf, sizeof(buf), "%g", d);
            return buf;
        }
    }
}

ParamValue ParamValue::fromString(const std::string& text, Type hint, bool* ok) {
    ParamValue v;
    v.type = hint;
    *ok = true;
    std::string t = text;
    // 去掉可能的引号
    if (t.size() >= 2 && (t.front() == '"' || t.front() == '\'') && t.back() == t.front()) {
        t = t.substr(1, t.size() - 2);
    }
    if (hint == Type::kString) {
        v.s = t;
        return v;
    }
    if (hint == Type::kBool) {
        std::string l = t;
        for (auto& c : l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (l == "true" || l == "1" || l == "on" || l == "yes") {
            v.b = true;
        } else if (l == "false" || l == "0" || l == "off" || l == "no") {
            v.b = false;
        } else {
            *ok = false;
        }
        return v;
    }
    char* end = nullptr;
    const double num = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || (end && *end != '\0')) {
        *ok = false;
        return v;
    }
    if (hint == Type::kInt) {
        v.i = static_cast<long long>(std::llround(num));
        v.d = static_cast<double>(v.i);
    } else {
        v.d = num;
    }
    return v;
}

namespace {
void setNonBlocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}
}  // namespace

// ================================================================ baud
int baudToConstant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return 0;
    }
}

// ================================================================ ParamTuner
ParamTuner::~ParamTuner() {
    stop();
}

bool ParamTuner::start(const std::string& port, int baud, Getter getter, Setter setter,
                        StatusProvider status, Saver saver, PresetLister list_presets,
                        PresetLoader load_preset) {
    if (port.empty()) return false;
    port_ = port;
    baud_ = baud;
    // "tcp:8766" / ":8766" -> TCP 模式；否则当串口设备路径
    const size_t colon = port.find(':');
    const std::string head = port.substr(0, colon);
    if (colon != std::string::npos && (head == "tcp" || head.empty())) {
        tcp_port_ = std::atoi(port.c_str() + colon + 1);
        if (tcp_port_ <= 0 || tcp_port_ > 65535) {
            RCLCPP_ERROR(rclcpp::get_logger("param_tuner"), "非法的 TCP 端口: %s", port.c_str());
            return false;
        }
    } else {
        device_ = port;
    }
    getter_ = std::move(getter);
    setter_ = std::move(setter);
    status_ = std::move(status);
    saver_ = std::move(saver);
    list_presets_ = std::move(list_presets);
    load_preset_ = std::move(load_preset);

    running_.store(true);
    quit_.store(false);
    thread_ = std::thread([this]() { loop(); });
    return true;
}

void ParamTuner::stop() {
    if (!running_.exchange(false)) return;
    quit_.store(true);
    if (thread_.joinable()) thread_.join();
    closePort();
}

bool ParamTuner::openPort() {
    // ── TCP 模式：tune_port 写成 tcp:PORT 或 :PORT ──
    if (tcp_port_ > 0) {
        if (tcp_listen_ >= 0) return true;
        const int lfd = socket(AF_INET, SOCK_STREAM, 0);
        if (lfd < 0) return false;
        int yes = 1;
        setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(tcp_port_));
        if (bind(lfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0 ||
            listen(lfd, 4) != 0) {
            ::close(lfd);
            return false;
        }
        setNonBlocking(lfd);
        tcp_listen_ = lfd;
        return true;
    }

    // ── 串口模式 ──
    if (fd_ >= 0) return true;
    const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return false;
    }
    cfmakeraw(&tty);
    const speed_t speed = baudToConstant(baud_);
    if (speed == 0) {
        ::close(fd);
        return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;  // 非阻塞读，靠 poll
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    return true;
}

void ParamTuner::closePort() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (tcp_listen_ >= 0) {
        ::close(tcp_listen_);
        tcp_listen_ = -1;
    }
    rx_buf_.clear();
}

void ParamTuner::feed(const char* data, size_t n) {
    rx_buf_.append(data, n);
    size_t pos;
    while ((pos = rx_buf_.find('\n')) != std::string::npos) {
        std::string line = rx_buf_.substr(0, pos);
        rx_buf_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        // 只入队，不在这里执行 —— 见 param_tuner.hpp 的线程模型说明
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_cmds_.push_back(line);
    }
    if (rx_buf_.size() > 4096) rx_buf_.clear();  // 防止半截命令堆积
}

bool ParamTuner::popPendingCmd(std::string* line) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (pending_cmds_.empty()) return false;
    *line = pending_cmds_.front();
    pending_cmds_.erase(pending_cmds_.begin());
    return true;
}

void ParamTuner::pushReply(std::string reply) {
    if (reply.empty()) return;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    replies_.push_back(std::move(reply));
}

// 把待发送的应答写回串口/TCP。必须在 I/O 线程调用（它直接操作 fd_）。
void ParamTuner::flushReplies() {
    std::vector<std::string> out;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (replies_.empty()) return;
        out.swap(replies_);
    }
    if (fd_ < 0) return;
    for (const auto& payload : out) {
        std::string buf = payload;
        buf.push_back('\n');
        size_t written = 0;
        while (written < buf.size()) {
            const ssize_t w = ::write(fd_, buf.data() + written, buf.size() - written);
            if (w <= 0) {
                // 非阻塞 socket/串口：EAGAIN 说明对端没跟上，这帧应答只能丢，
                // 总不能为了发一条状态把整个 I/O 循环卡死
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                break;
            }
            written += static_cast<size_t>(w);
        }
    }
}

void ParamTuner::loop() {
    char buf[4096];
    while (!quit_.load()) {
        if (!openPort()) {
            // 设备/端口可能还没就绪，每秒重试
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (tcp_port_ > 0) {
            // 接受连接：只保留最后一个客户端（调参场景一次只有一个）
            bool has_client = false;
            struct sockaddr_in peer{};
            socklen_t plen = sizeof(peer);
            const int cfd = accept(tcp_listen_, reinterpret_cast<struct sockaddr*>(&peer), &plen);
            if (cfd >= 0) {
                if (fd_ >= 0) ::close(fd_);
                fd_ = cfd;
                setNonBlocking(fd_);
                RCLCPP_INFO(rclcpp::get_logger("param_tuner"),
                            "调参客户端已连接: %s:%d", inet_ntoa(peer.sin_addr),
                            ntohs(peer.sin_port));
            } else if (fd_ < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (fd_ >= 0) {
                has_client = true;
                const ssize_t n = ::read(fd_, buf, sizeof(buf));
                if (n > 0) {
                    feed(buf, static_cast<size_t>(n));
                } else if (n == 0) {
                    // 对端断开
                    RCLCPP_INFO(rclcpp::get_logger("param_tuner"), "调参客户端已断开");
                    ::close(fd_);
                    fd_ = -1;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                // 命令由执行器线程处理完才会出现在这里
                flushReplies();
            }
            if (!has_client) std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // 串口模式
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            feed(buf, static_cast<size_t>(n));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        flushReplies();
    }
}

// ================================================================ 命令处理
std::string ParamTuner::handleLine(const std::string& line) {
    std::istringstream ss(line);
    std::string cmd;
    ss >> cmd;
    cmd = upperCase(cmd);

    if (cmd == "PING") {
        return "{\"ok\":true,\"proto\":1,\"port\":\"" + jsonEscape(port_) + "\"}";
    }
    if (cmd == "HELP" || cmd == "?") return cmdHelp();
    if (cmd == "LIST") return cmdList();
    if (cmd == "GET") {
        std::string name;
        ss >> name;
        if (name.empty()) return "{\"ok\":false,\"error\":\"用法: GET <参数名>\"}";
        return cmdGet(name);
    }
    if (cmd == "SET") {
        std::string name;
        ss >> name;
        if (name.empty()) {
            return "{\"ok\":false,\"error\":\"用法: SET <参数名> <值>\"}";
        }
        std::string rest;
        std::getline(ss, rest);
        // 去掉多余空格（string 值内部可能有空格，只 trim 两端）
        size_t b = rest.find_first_not_of(" \t");
        size_t e = rest.find_last_not_of(" \t");
        rest = (b == std::string::npos) ? "" : rest.substr(b, e - b + 1);
        if (rest.empty()) return "{\"ok\":false,\"error\":\"用法: SET <参数名> <值>\"}";
        return cmdSet(name, rest);
    }
    if (cmd == "STATUS") return cmdStatus();
    if (cmd == "PRESETS") return cmdPresets();
    if (cmd == "LOAD") {
        std::string name;
        ss >> name;
        if (name.empty()) return "{\"ok\":false,\"error\":\"用法: LOAD <预设名>\"}";
        return cmdLoad(name);
    }
    if (cmd == "SAVE") {
        std::string err;
        if (!saver_ || !saver_(&err)) {
            return "{\"ok\":false,\"error\":\"保存失败: " + jsonEscape(err) + "\"}";
        }
        return "{\"ok\":true,\"saved\":true}";
    }
    return "{\"ok\":false,\"error\":\"未知命令: " + jsonEscape(cmd) + "，发 HELP 看可用命令\"}";
}

std::string ParamTuner::cmdHelp() const {
    return
        "{\"ok\":true,\"commands\":["
        "\"PING  连通性检查\","
        "\"LIST  列出全部可调参数\","
        "\"GET <名>  读取参数\","
        "\"SET <名> <值>  修改参数(立即生效)\","
        "\"SAVE  把当前值写入 params.yaml(重启保留)\","
        "\"PRESETS  可用预设列表\","
        "\"LOAD <预设名>  加载预设\","
        "\"STATUS  运行状态(瞄准误差/目标/模式)\","
        "\"HELP  本帮助\"]}";
}

std::string ParamTuner::cmdList() const {
    std::string out = "{\"ok\":true,\"params\":[";
    bool first = true;
    for (const auto& p : paramRegistry()) {
        ParamValue v;
        if (getter_ && !getter_(p.name, &v)) continue;
        if (!first) out += ",";
        first = false;
        out += "{\"name\":\"" + jsonEscape(p.name) + "\",";
        out += "\"label\":\"" + jsonEscape(p.label) + "\",";
        out += "\"group\":\"" + jsonEscape(p.group) + "\",";
        out += "\"group_label\":\"" + jsonEscape(paramGroupLabel(p.group)) + "\",";
        out += "\"desc\":\"" + jsonEscape(p.desc) + "\",";
        out += "\"type\":\"" + std::string(typeName(metaTypeToValueType(p.type))) + "\",";
        out += "\"min\":" + std::to_string(static_cast<long long>(p.min)) + ",";
        out += "\"max\":" + std::to_string(static_cast<long long>(p.max)) + ",";
        out += "\"node\":\"" + std::string(p.owner == ParamOwner::kDartAim ? "dart_aim" : "hik_camera") + "\",";
        out += "\"value\":" + v.toJson() + "}";
    }
    out += "]}";
    return out;
}

std::string ParamTuner::cmdGet(const std::string& name) const {
    const ParamMeta* meta = findParam(name);
    if (!meta) return "{\"ok\":false,\"error\":\"没有这个参数: " + jsonEscape(name) + "\"}";
    ParamValue v;
    if (!getter_ || !getter_(name, &v)) {
        return "{\"ok\":false,\"error\":\"读取失败: " + jsonEscape(name) + "\"}";
    }
    std::string out = "{\"ok\":true,\"name\":\"" + jsonEscape(name) + "\",";
    out += "\"label\":\"" + jsonEscape(meta->label) + "\",";
    out += "\"type\":\"" + std::string(typeName(metaTypeToValueType(meta->type))) + "\",";
    out += "\"value\":" + v.toJson() + "}";
    return out;
}

std::string ParamTuner::cmdSet(const std::string& name, const std::string& value) {
    const ParamMeta* meta = findParam(name);
    if (!meta) return "{\"ok\":false,\"error\":\"没有这个参数: " + jsonEscape(name) + "\"}";

    bool ok = false;
    const ParamValue v = ParamValue::fromString(value, metaTypeToValueType(meta->type), &ok);
    if (!ok) {
        return "{\"ok\":false,\"error\":\"值格式不对: " + jsonEscape(value) + "（应为 " +
               typeName(metaTypeToValueType(meta->type)) + "）\"}";
    }
    if (meta->type == ParamType::kInt || meta->type == ParamType::kDouble) {
        const double num = (meta->type == ParamType::kInt) ? static_cast<double>(v.i) : v.d;
        if (num < meta->min || num > meta->max) {
            char buf[128];
            snprintf(buf, sizeof(buf), "超出范围: 允许 %.0f ~ %.0f，当前 %g",
                     meta->min, meta->max, num);
            return "{\"ok\":false,\"error\":\"" + jsonEscape(buf) + "\"}";
        }
    }
    std::string err;
    if (!setter_ || !setter_(name, v, &err)) {
        return "{\"ok\":false,\"error\":\"" + jsonEscape(err.empty() ? "设置失败" : err) + "\"}";
    }
    // 回读确认（有些参数需要重载/校验后才真正生效）
    ParamValue now;
    if (getter_ && getter_(name, &now)) {
        return "{\"ok\":true,\"name\":\"" + jsonEscape(name) + "\",\"value\":" + now.toJson() + "}";
    }
    return "{\"ok\":true,\"name\":\"" + jsonEscape(name) + "\"}";
}

std::string ParamTuner::cmdStatus() const {
    std::string out = "{\"ok\":true";
    if (status_) {
        for (const auto& kv : status_()) {
            out += ",\"" + jsonEscape(kv.first) + "\":\"" + jsonEscape(kv.second) + "\"";
        }
    }
    out += "}";
    return out;
}

std::string ParamTuner::cmdPresets() const {
    std::string out = "{\"ok\":true,\"presets\":[";
    if (list_presets_) {
        bool first = true;
        for (const auto& p : list_presets_()) {
            if (!first) out += ",";
            first = false;
            out += "\"" + jsonEscape(p) + "\"";
        }
    }
    out += "]}";
    return out;
}

std::string ParamTuner::cmdLoad(const std::string& name) {
    std::string err;
    if (!load_preset_ || !load_preset_(name, &err)) {
        return "{\"ok\":false,\"error\":\"" + jsonEscape(err.empty() ? "预设不存在" : err) + "\"}";
    }
    return "{\"ok\":true,\"preset\":\"" + jsonEscape(name) + "\"}";
}

}  // namespace yq_dart_aim
