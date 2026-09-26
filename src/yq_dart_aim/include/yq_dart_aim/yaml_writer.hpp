#pragma once

#include <map>
#include <string>
#include <vector>

namespace yq_dart_aim {

// params.yaml / camera_params.yaml 的"外科手术式"写回：
// 只替换指定 key 那一行的值，其余内容（含注释、顺序、空行）原样保留。
//
// 为什么不用 YAML 库：这两个文件结构固定（两层缩进的扁平 map），
// 手写行处理反而更安全——不会像 yaml.dump 那样把注释和排版全部重排。
// 写入前会自动生成 .bak 备份。

// 一条待写入的键值。value 按类型格式化：
//   kInt/kDouble -> 数值；kBool -> true/false；kString -> 原样(需带引号则加)
struct YamlValue {
    enum class Kind { kInt, kDouble, kBool, kString } kind = Kind::kDouble;
    double num = 0.0;
    bool flag = false;
    std::string str;
};

// 把 keys 写回 yaml_path。
//   - key 已存在：只替换那一行的值
//   - key 不存在：追加到 section 指定的缩进块末尾
// 返回 true 表示文件写入成功；失败原因写入 error。
bool updateYamlFile(const std::string& yaml_path, const std::string& section,
                    const std::vector<std::pair<std::string, YamlValue>>& values,
                    std::string* error);

// 读一个键的原始值文本（去掉行尾空白）。找不到返回 false。
bool readYamlValue(const std::string& yaml_path, const std::string& section,
                   const std::string& key, std::string* out);

// 列出某 section 下所有 key（保持文件里的顺序）
std::vector<std::string> listYamlKeys(const std::string& yaml_path,
                                      const std::string& section);

}  // namespace yq_dart_aim
