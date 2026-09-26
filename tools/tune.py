#!/usr/bin/env python3
"""串口调参工具 —— PC 端

用法不需要记命令，直接跑就是菜单：

    python3 tools/tune.py

会自动找串口（优先 /dev/ttyACM1，其次第一个 ttyACM*），连上后出中文菜单：

    1) 查看全部参数      4) 加载预设
    2) 修改参数          5) 保存到板子（重启保留）
    3) 实时监控          6) 重新连接      q) 退出

也支持一行命令（方便写脚本/批量）：

    python3 tools/tune.py list                     列出全部参数
    python3 tools/tune.py get p_err                读一个参数
    python3 tools/tune.py set p_err 8.5            改一个参数（立即生效）
    python3 tools/tune.py save                      写入板子 params.yaml
    python3 tools/tune.py watch                     持续打印瞄准误差
    python3 tools/tune.py presets / load match_day
    python3 tools/tune.py --port /dev/ttyACM0 ...   指定串口

依赖：pyserial（pip install pyserial）。板子上也能跑（termios 兜底，无需 pyserial）。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

# ---------------------------------------------------------------- 串口封装
class Link:
    """串口连接：优先 pyserial，没有就用 termios 自己实现（Linux）"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.3):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.fd = None
        self._rx = b""

    def open(self):
        try:
            import serial  # noqa
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            return True
        except ImportError:
            pass
        except Exception as e:
            print(f"打开串口失败: {e}", file=sys.stderr)
            return False
        # termios 兜底（没有 pyserial 时）
        try:
            import fcntl
            import termios
        except ImportError:
            print("没有 pyserial 且不是 Linux，无法打开串口。请 pip install pyserial",
                  file=sys.stderr)
            return False
        try:
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0                      # iflag: raw
            attrs[1] = 0                      # oflag: raw
            attrs[2] = (termios.CLOCAL | termios.CREAD | termios.CS8)
            attrs[3] = 0                      # lflag: raw
            speed = getattr(termios, f"B{self.baud}", None)
            if speed is None:
                os.close(self.fd)
                self.fd = None
                print(f"不支持的波特率 {self.baud}", file=sys.stderr)
                return False
            attrs[4] = attrs[5] = speed       # ispeed/ospeed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            _ = fcntl
            return True
        except Exception as e:
            print(f"打开串口失败: {e}", file=sys.stderr)
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None
            return False

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None

    def _read_bytes(self) -> bytes:
        if self.ser is not None:
            try:
                return self.ser.read(4096) or b""
            except Exception:
                return b""
        if self.fd is not None:
            try:
                return os.read(self.fd, 4096)
            except BlockingIOError:
                return b""
            except OSError:
                return b""
        return b""

    def _write_bytes(self, data: bytes) -> None:
        if self.ser is not None:
            try:
                self.ser.write(data)
            except Exception:
                pass
        elif self.fd is not None:
            try:
                os.write(self.fd, data)
            except OSError:
                pass

    def cmd(self, line: str, timeout: float = 1.0) -> dict | None:
        """发一行命令，等一行 JSON 应答"""
        self._write_bytes((line + "\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            while b"\n" in self._rx:
                raw, _, self._rx = self._rx.partition(b"\n")
                text = raw.decode("utf-8", "replace").strip()
                if not text:
                    continue
                try:
                    return json.loads(text)
                except json.JSONDecodeError:
                    continue
            chunk = self._read_bytes()
            if chunk:
                self._rx += chunk
            else:
                time.sleep(0.02)
        return None


def autodetect_port() -> str | None:
    """优先 ttyACM1（约定的调参口），否则第一个 ttyACM*"""
    if os.path.exists("/dev/ttyACM1"):
        return "/dev/ttyACM1"
    devs = sorted(glob.glob("/dev/ttyACM*"))
    return devs[0] if devs else None


# ---------------------------------------------------------------- 显示
def fmt_value(v) -> str:
    if isinstance(v, bool):
        return "开" if v else "关"
    if isinstance(v, float) and v == int(v):
        return str(int(v))
    return str(v)


class TunerUI:
    def __init__(self, link: Link):
        self.link = link
        self.params: list[dict] = []
        self.meta: dict[str, dict] = {}
        self.dirty = False  # 改过参数但没保存

    # ---- 协议封装 ----
    def list_params(self) -> bool:
        r = self.link.cmd("LIST", timeout=2.0)
        if not r or not r.get("ok"):
            return False
        self.params = r.get("params", [])
        self.meta = {p["name"]: p for p in self.params}
        return True

    def refresh_values(self) -> None:
        self.list_params()

    # ---- 菜单 ----
    def show_all(self) -> None:
        self.refresh_values()
        if not self.params:
            print("（没有参数）")
            return
        group = None
        for p in self.params:
            if p["group_label"] != group:
                group = p["group_label"]
                print(f"\n【{group}】")
            desc = f"  # {p['desc']}" if p.get("desc") else ""
            rng = ""
            if p["type"] in ("int", "double"):
                rng = f"  [{p['min']}~{p['max']}]"
            print(f"  {p['label']:<18} = {fmt_value(p['value']):<10}"
                  f" ({p['name']}){rng}{desc}")

    def modify_param(self) -> None:
        self.refresh_values()
        if not self.params:
            print("（没有参数）")
            return
        key = input("输入参数名（不知道就看上面列表，或输入 q 返回）: ").strip()
        if key in ("q", "Q", ""):
            return
        p = self.meta.get(key)
        if p is None:
            # 允许用中文标签模糊匹配，方便非技术人员
            cands = [q for q in self.params if q["label"] == key or key in q["label"]]
            if len(cands) == 1:
                p = cands[0]
            elif len(cands) > 1:
                print("匹配到多个，请用参数名:", [c["name"] for c in cands])
                return
            else:
                print(f"没有名为 {key} 的参数")
                return
        tip = f"（{p['desc']}）" if p.get("desc") else ""
        rng = f" 范围 {p['min']}~{p['max']}" if p["type"] in ("int", "double") else ""
        cur = fmt_value(p["value"])
        if p["type"] == "bool":
            new = input(f"{p['label']} 当前={cur}，输入 y 开启 / n 关闭{rng}: ").strip().lower()
            if new not in ("y", "n"):
                print("输入无效，已取消")
                return
            val = (new == "y")
        else:
            val = input(f"{p['label']} 当前={cur}{rng}{tip}\n新值: ").strip()
            if val == "":
                print("已取消")
                return
            try:
                val = float(val) if p["type"] in ("int", "double") else val
            except ValueError:
                print("不是数字，已取消")
                return
        r = self.link.cmd(f"SET {p['name']} {val}")
        if r and r.get("ok"):
            print(f"✓ {p['label']} 已改为 {fmt_value(r.get('value', val))}（立即生效）")
            if not self.dirty:
                self.dirty = True
                print("  提醒：还没保存，选主菜单 5) 保存后重启才不会丢")
        else:
            print("✗ 失败:", (r or {}).get("error", "无响应"))

    def watch(self) -> None:
        print("每 0.5 秒刷新一次瞄准误差，按 Ctrl-C 返回菜单\n")
        print(f"{'时间':<10}{'模式':<8}{'目标类别':<14}{'瞄准误差(px)':<14}{'已瞄准':<8}")
        try:
            while True:
                r = self.link.cmd("STATUS", timeout=1.0)
                if r and r.get("ok"):
                    ts = time.strftime("%H:%M:%S")
                    mode = r.get("mode", "?")
                    tgt = r.get("target_class", "?")
                    err = r.get("aim_err_x_px", "?")
                    aim = "是" if r.get("aiming") == "yes" else "否"
                    print(f"{ts:<10}{mode:<8}{tgt:<14}{err:<14}{aim:<8}", end="\r")
                else:
                    print("  （无响应，检查接线/端口）", end="\r")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print()

    def load_preset(self) -> None:
        r = self.link.cmd("PRESETS")
        if not r or not r.get("ok"):
            print("读取预设失败")
            return
        names = r.get("presets", [])
        if not names:
            print("没有可用预设")
            return
        print("可用预设:", ", ".join(names))
        name = input("输入要加载的预设名（q 返回）: ").strip()
        if name in ("q", "", None):
            return
        res = self.link.cmd(f"LOAD {name}")
        if res and res.get("ok"):
            print(f"✓ 已加载预设 {name}（立即生效，记得再保存）")
            self.dirty = True
        else:
            print("✗ 失败:", (res or {}).get("error", "无响应"))

    def save(self) -> None:
        ans = input("确认把当前参数写入板子 params.yaml？重启后仍保留 (y/N): ").strip().lower()
        if ans != "y":
            print("已取消")
            return
        r = self.link.cmd("SAVE", timeout=3.0)
        if r and r.get("ok"):
            self.dirty = False
            print("✓ 已保存到板子（params.yaml / camera_params.yaml），原文件已备份为 .bak")
        else:
            print("✗ 保存失败:", (r or {}).get("error", "无响应"))

    def menu_loop(self) -> None:
        while True:
            print("\n" + "=" * 56)
            print("  飞镖视觉 串口调参")
            print("=" * 56)
            print("  1) 查看全部参数")
            print("  2) 修改参数")
            print("  3) 实时监控瞄准误差")
            print("  4) 加载预设")
            print("  5) 保存到板子（重启保留）")
            print("  6) 重新连接串口")
            print("  q) 退出")
            if self.dirty:
                print("  * 有修改未保存！选 5 保存，否则重启会丢")
            print("-" * 56)
            c = input("请选择: ").strip().lower()
            if c == "1":
                self.show_all()
            elif c == "2":
                self.modify_param()
            elif c == "3":
                self.watch()
            elif c == "4":
                self.load_preset()
            elif c == "5":
                self.save()
            elif c == "6":
                self.link.close()
                if not connect(self.link, self.port_wanted):
                    print("重连失败，仍在菜单中")
            elif c == "q":
                if self.dirty:
                    ans = input("有未保存修改，确认退出？(y/N): ").strip().lower()
                    if ans != "y":
                        continue
                print("再见")
                return
            else:
                print("请输入 1-6 或 q")


def connect(link: Link, port: str | None) -> bool:
    if port is None:
        print("没找到串口设备。请确认：")
        print("  1) USB-TTL 已插好（板子上应出现 /dev/ttyACM1）")
        print("  2) 板子上的参数 tune_port 与实际端口一致")
        return False
    link.port = port
    if not link.open():
        return False
    r = link.cmd("PING", timeout=2.0)
    if r and r.get("ok"):
        print(f"✓ 已连接 {port}")
        return True
    print(f"串口打开了但板子没响应（{port}）。检查是否插错口/波特率，或板子服务是否在跑。")
    link.close()
    return False


def cmdline_mode(link: Link, args) -> int:
    """一行命令模式"""
    act = args.action
    if act == "list":
        r = link.cmd("LIST", timeout=2.0)
        if not r or not r.get("ok"):
            print("读取失败")
            return 1
        for p in r.get("params", []):
            print(f"{p['name']:<26} = {fmt_value(p['value']):<10}  # {p['label']}")
        return 0
    if act == "get":
        r = link.cmd(f"GET {args.name}")
        print(json.dumps(r, ensure_ascii=False) if r else "无响应")
        return 0 if r and r.get("ok") else 1
    if act == "set":
        r = link.cmd(f"SET {args.name} {args.value}")
        if r and r.get("ok"):
            print(f"{args.name} = {fmt_value(r.get('value', args.value))}")
            return 0
        print("失败:", (r or {}).get("error", "无响应"))
        return 1
    if act == "save":
        r = link.cmd("SAVE", timeout=3.0)
        print("已保存" if r and r.get("ok") else f"失败: {(r or {}).get('error')}")
        return 0 if r and r.get("ok") else 1
    if act == "presets":
        r = link.cmd("PRESETS")
        print("预设:", ", ".join(r.get("presets", [])) if r and r.get("ok") else "读取失败")
        return 0
    if act == "load":
        r = link.cmd(f"LOAD {args.name}")
        print("已加载" if r and r.get("ok") else f"失败: {(r or {}).get('error')}")
        return 0 if r and r.get("ok") else 1
    if act == "status":
        r = link.cmd("STATUS")
        print(json.dumps(r, ensure_ascii=False, indent=2) if r else "无响应")
        return 0
    if act == "watch":
        print("Ctrl-C 结束")
        try:
            while True:
                r = link.cmd("STATUS", timeout=1.0)
                if r and r.get("ok"):
                    print(f"err_x={r.get('aim_err_x_px')}px  目标={r.get('target_class')}"
                          f"  瞄准={'是' if r.get('aiming') == 'yes' else '否'}"
                          f"  模式={r.get('mode')}", end="\r")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print()
        return 0
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="飞镖视觉 串口调参（菜单式）")
    ap.add_argument("--port", default=None, help="串口设备，默认自动找 /dev/ttyACM1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("action", nargs="?",
                    choices=["list", "get", "set", "save", "presets", "load", "status", "watch"],
                    help="不填则进入菜单")
    ap.add_argument("name", nargs="?", help="参数名或预设名")
    ap.add_argument("value", nargs="?", help="要设置的值")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    link = Link(port or "", args.baud)
    if not connect(link, port):
        return 1

    try:
        if args.action is None:
            ui = TunerUI(link)
            ui.port_wanted = port
            ui.refresh_values()
            ui.menu_loop()
            return 0
        return cmdline_mode(link, args)
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
