#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
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
    bool start(const std::string& port, int baud,
               Getter getter, Setter setter, StatusProvider status,
               Saver saver, PresetLister list_presets, PresetLoader load_preset);

    void stop();

    bool running() const { return running_.load(); }
    std::string port() const { return port_; }

    // 供测试：直接喂一行命令，返回应答（不经过串口）
    std::string handleLine(const std::string& line);

private:
    void loop();
    bool openPort();
    void closePort();
    std::string cmdHelp() const;
    std::string cmdList() const;
    std::string cmdGet(const std::string& name) const;
    std::string cmdSet(const std::string& name, const std::string& value);
    std::string cmdStatus() const;
    std::string cmdPresets() const;
    std::string cmdLoad(const std::string& name);

    std::string port_;
    int baud_ = 115200;
    int fd_ = -1;
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
