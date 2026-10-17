#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace yq_dart_aim {

// 参数值的通用表示（够用来覆盖 int/double/bool/string）
struct ParamValue {
    enum class Type { kInt, kDouble, kBool, kString };
    Type type = Type::kDouble;
    long long i = 0;
    double d = 0.0;
    bool b = false;
    std::string s;

    std::string toJson() const;
    static ParamValue fromString(const std::string& text, Type hint, bool* ok);
};

// 串口调参服务
//
// 独立于下位机通信的第二个串口，给"非技术人员"用：
//   PC 端插一根 USB-TTL，连上后用 tools/tune.py 菜单操作，
//   不需要 ROS、不需要浏览器、不需要 SSH。
//
// 协议：ASCII 行命令（\n 结尾），应答也是一行 JSON。
//   PING              -> {"ok":true,"proto":1,...}
//   LIST              -> {"ok":true,"groups":[...],"params":[{...}]}
//   GET <name>        -> {"ok":true,"name":..,"value":..,"type":..}
//   SET <name> <val>  -> {"ok":true,...} 或 {"ok":false,"error":".."}
//   SAVE              -> 把当前值写回 params.yaml / camera_params.yaml
//   PRESETS           -> 可用预设列表
//   LOAD <name>       -> 加载预设
//   STATUS            -> 瞄准误差/目标/模式等运行状态
//   HELP              -> 命令列表
//
// 参数读写由外部注入回调（节点实现），本类只管串口与协议。
class ParamTuner {
public:
    // 读一个参数；name 不存在返回 false
    using Getter = std::function<bool(const std::string& name, ParamValue* out)>;
    // 写一个参数；返回 false 时 error 填原因
    using Setter = std::function<bool(const std::string& name, const ParamValue& value,
                                      std::string* error)>;
    // 运行状态（给 STATUS 用）
    using StatusProvider = std::function<std::map<std::string, std::string>()>;
    // 落盘：把当前所有参数写回 yaml
    using Saver = std::function<bool(std::string* error)>;
    // 加载预设
    using PresetLoader = std::function<bool(const std::string& name, std::string* error)>;
    using PresetLister = std::function<std::vector<std::string>()>;

    ~ParamTuner();

    // port 为空表示不启用；否则启动监听线程（设备不存在时每秒重试）
    //
    // port 支持两种写法：
    //   串口设备路径，如 /dev/ttyACM1、/dev/ttyS1
    //   "tcp:PORT" 或 ":PORT"，在所有网卡上监听 TCP（配合 USB 网络调参用）
    bool start(const std::string& port, int baud,
               Getter getter, Setter setter, StatusProvider status,
               Saver saver, PresetLister list_presets, PresetLoader load_preset);

    void stop();

    bool running() const { return running_.load(); }
    std::string port() const { return port_; }

    // ── 线程模型 ──
    // 串口/TCP 的读写在 ParamTuner 自己的 I/O 线程上做，但**命令绝不在那个线程上执行**。
    // 原因：getter/setter 会去碰 rclcpp（get_parameter/set_parameter、创建定时器与订阅），
    // 这些都不是线程安全的；rclcpp 只保证「在执行器线程内」调用才安全。
    // 在 I/O 线程里直接调会和图像回调、参数回调并发踩同一份状态（UB）。
    //
    // 所以这里拆成两段：
    //   I/O 线程：收字节 -> 切出整行 -> 塞进 pending_cmds_ 队列
    //   执行器线程：定时取走命令执行 -> 把应答塞进 replies_ 队列
    //   I/O 线程：把 replies_ 队列写回串口/TCP
    //
    // 节点侧用下面两个接口驱动这个回路（见 dart_aim_node.cpp 的 drainTunerQueue）。

    // 从 I/O 线程取一条待执行命令（队列空返回 false）。只能由执行器线程调用。
    bool popPendingCmd(std::string* line);

    // 执行器线程处理完命令后把应答交回 I/O 线程去发送
    void pushReply(std::string reply);

    // 供测试：直接喂一行命令，返回应答（不经过串口）
    std::string handleLine(const std::string& line);

private:
    void loop();
    bool openPort();
    void closePort();
    // 处理已收到的字节流：只负责切行并入队，不执行命令（见上面的线程模型说明）
    void feed(const char* data, size_t n);
    // 把待发送的应答写回串口/TCP（由 I/O 线程调用）
    void flushReplies();
    std::string cmdHelp() const;
    std::string cmdList() const;
    std::string cmdGet(const std::string& name) const;
    std::string cmdSet(const std::string& name, const std::string& value);
    std::string cmdStatus() const;
    std::string cmdPresets() const;
    std::string cmdLoad(const std::string& name);

    std::string port_;      // 原始配置值
    std::string device_;    // 实际设备路径（串口模式）
    int tcp_port_ = 0;       // >0 表示 TCP 模式
    int baud_ = 115200;
    int fd_ = -1;
    int tcp_listen_ = -1;   // TCP 监听 fd
    std::string rx_buf_;    // 行缓冲（跨多次读）

    // I/O 线程 <-> 执行器线程之间传递命令与应答
    std::vector<std::string> pending_cmds_;
    std::vector<std::string> replies_;
    std::mutex queue_mutex_;

    std::atomic<bool> running_{false};
    std::atomic<bool> quit_{false};
    std::thread thread_;

    Getter getter_;
    Setter setter_;
    StatusProvider status_;
    Saver saver_;
    PresetLister list_presets_;
    PresetLoader load_preset_;
};

// 串口波特率 -> 常量（B115200 等）。找不到返回 0。
int baudToConstant(int baud);

}  // namespace yq_dart_aim
