#include "yq_dart_aim/yaml_writer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

namespace yq_dart_aim {

namespace {

std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// 行首缩进宽度（空格数；tab 记 1）。0 表示顶层行。
size_t indentOf(const std::string& line) {
    size_t n = 0;
    while (n < line.size() && (line[n] == ' ' || line[n] == '\t')) ++n;
    return n;
}

// 是否是"有内容的行"（非空、非纯注释）
bool isContentLine(const std::string& line) {
    const std::string t = trim(line);
    return !t.empty() && t[0] != '#';
}

std::vector<std::string> readLines(const std::string& path, bool* ok) {
    std::vector<std::string> lines;
    std::ifstream f(path);
    *ok = f.good();
    if (!f.good()) return lines;
    std::string line;
    while (std::getline(f, line)) lines.push_back(line);
    return lines;
}

std::string formatValue(const YamlValue& v) {
    char buf[64];
    switch (v.kind) {
        case YamlValue::Kind::kInt:
            snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(std::llround(v.num)));
            return buf;
        case YamlValue::Kind::kBool:
            return v.flag ? "true" : "false";
        case YamlValue::Kind::kString: {
            std::string s = v.str;
            bool need_quote = s.empty() || s.find_first_of(":#{}[],&*?|-<>=!%@`\"' \t") != std::string::npos;
            return need_quote ? "\"" + s + "\"" : s;
        }
        case YamlValue::Kind::kDouble:
        default:
            // 用 %.9g 保留原样精度；整数值的 double 写成 35（而不是 35.000000），
            // 需要 .0 的场合（如 max_area: 130.0）由调用方用 kDouble + 显式小数保证
            snprintf(buf, sizeof(buf), "%.9g", v.num);
            return buf;
    }
}

bool backupFile(const std::string& path) {
    struct stat st{};
    if (stat(path.c_str(), &st) != 0) return false;  // 文件不存在，无需备份
    std::string bak = path + ".bak";
    std::string cmd = "cp -f '" + path + "' '" + bak + "'";
    return system(cmd.c_str()) == 0;
}

}  // namespace

bool readYamlValue(const std::string& yaml_path, const std::string& section,
                   const std::string& key, std::string* out) {
    bool ok = false;
    auto lines = readLines(yaml_path, &ok);
    if (!ok) return false;

    bool in_section = false;
    const std::string section_prefix = section + ":";
    for (const auto& raw : lines) {
        if (!isContentLine(raw)) continue;
        const std::string line = trim(raw);
        if (indentOf(raw) == 0) {
            in_section = (line == section_prefix);
            continue;
        }
        if (!in_section) continue;
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        if (trim(line.substr(0, colon)) == key) {
            *out = trim(line.substr(colon + 1));
            return true;
        }
    }
    return false;
}

std::vector<std::string> listYamlKeys(const std::string& yaml_path,
                                      const std::string& section) {
    std::vector<std::string> keys;
    bool ok = false;
    auto lines = readLines(yaml_path, &ok);
    if (!ok) return keys;

    bool in_section = false;
    const std::string section_prefix = section + ":";
    for (const auto& raw : lines) {
        if (!isContentLine(raw)) continue;
        const std::string line = trim(raw);
        if (indentOf(raw) == 0) {
            in_section = (line == section_prefix);
            continue;
        }
        if (!in_section) continue;
        if (line.rfind("- ", 0) == 0) continue;  // 列表项
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        keys.push_back(trim(line.substr(0, colon)));
    }
    return keys;
}

// 把 values 里的 key 替换成新值；返回命中的下标，未命中的留 npos
size_t replaceKeyIn(std::vector<std::string>& lines, const std::string& section_prefix,
                    const std::string& section, const std::string& key,
                    const std::string& formatted) {
    bool in_section = false;
    for (size_t li = 0; li < lines.size(); ++li) {
        const std::string& raw = lines[li];
        if (!isContentLine(raw)) continue;
        const std::string line = trim(raw);
        const size_t ind = indentOf(raw);
        if (ind == 0) {
            // 顶层行：看是不是我们要的 section
            in_section = (line == section_prefix);
            continue;
        }
        if (!in_section) continue;
        if (line.rfind("- ", 0) == 0) continue;  // 列表项
        const size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        if (trim(line.substr(0, colon)) != key) continue;

        // 保留行尾注释（" # 说明"），只换值；缩进沿用原行
        std::string tail;
        const std::string value_part = line.substr(colon + 1);
        const size_t hash = value_part.find('#');
        if (hash != std::string::npos) tail = " " + trim(value_part.substr(hash));
        lines[li] = raw.substr(0, ind) + key + ": " + formatted + tail;
        return li;
    }
    return std::string::npos;
}

bool updateYamlFile(const std::string& yaml_path, const std::string& section,
                    const std::vector<std::pair<std::string, YamlValue>>& values,
                    std::string* error) {
    bool ok = false;
    auto lines = readLines(yaml_path, &ok);
    if (!ok) {
        if (error) *error = "打不开文件: " + yaml_path;
        return false;
    }
    if (values.empty()) return true;

    const std::string section_prefix = section + ":";

    // 1) 先全部尝试"就地替换"
    std::vector<std::string> missing;
    for (const auto& kv : values) {
        const std::string formatted = formatValue(kv.second);
        if (replaceKeyIn(lines, section_prefix, section, kv.first, formatted) ==
            std::string::npos) {
            missing.push_back(kv.first + ": " + formatted);
        }
    }

    // 2) 未命中的追加到 section 块末尾（= 下一个 0 缩进内容行之前；没有则文件末尾）
    if (!missing.empty()) {
        int insert_at = -1;
        bool in_section = false;
        for (size_t li = 0; li < lines.size(); ++li) {
            if (!isContentLine(lines[li])) continue;
            if (indentOf(lines[li]) == 0) {
                const std::string line = trim(lines[li]);
                if (in_section) {
                    insert_at = static_cast<int>(li);
                    break;
                }
                in_section = (line == section_prefix);
            }
        }
        if (insert_at < 0) insert_at = static_cast<int>(lines.size());
        for (const auto& m : missing) lines.insert(lines.begin() + insert_at++, "    " + m);
    }

    backupFile(yaml_path);
    std::ofstream out(yaml_path, std::ios::trunc);
    if (!out.good()) {
        if (error) *error = "写入失败: " + yaml_path;
        return false;
    }
    for (const auto& l : lines) out << l << "\n";
    out.close();
    return true;
}

}  // namespace yq_dart_aim
