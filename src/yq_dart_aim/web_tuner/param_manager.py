"""
ROS2 参数管理器

在独立线程中运行 rclpy 节点，通过 ROS2 参数 API 读写其他节点的参数。
支持：
  - 读取参数当前值
  - 设置参数（即时生效）
  - 监听参数变更事件
  - 批量读写
"""

import threading
import time
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class ParamManager:
    """ROS2 参数管理器（独立线程运行）"""

    def __init__(self, on_param_changed: Callable[[str, str, Any], None] | None = None):
        """
        Args:
            on_param_changed: 回调函数 (node_name, param_name, new_value)
        """
        self._on_param_changed = on_param_changed
        self._node: Node | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._executor = None

        # 缓存参数值 {node_name: {param_name: value}}
        self._cache: dict[str, dict[str, Any]] = {}
        self._cache_lock = threading.Lock()

    def start(self):
        """启动 ROS2 参数管理线程"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """停止管理器"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _run(self):
        """ROS2 事件循环（在独立线程中运行）"""
        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass  # 已经初始化过

        self._node = Node("_web_tuner_param_manager")
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)

        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

        if self._node:
            self._node.destroy_node()
        self._running = False

    # ── 参数读写 ──────────────────────────────────────

    def get_param(self, node_name: str, param_name: str) -> Any | None:
        """读取指定节点的参数值"""
        if not self._node:
            return None

        client = self._node.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn(f'节点 {node_name} 不可达')
            return None

        request = GetParameters.Request()
        request.names = [param_name]
        future = client.call_async(request)

        # 等待结果
        timeout = time.time() + 3.0
        while not future.done() and time.time() < timeout:
            time.sleep(0.01)

        if not future.done():
            return None

        try:
            result = future.result()
            if result.values:
                value = self._extract_value(result.values[0])
                # 更新缓存
                with self._cache_lock:
                    if node_name not in self._cache:
                        self._cache[node_name] = {}
                    self._cache[node_name][param_name] = value
                return value
        except Exception:
            pass

        return None

    def get_all_params(self, node_name: str, param_names: list[str]) -> dict[str, Any]:
        """批量读取参数"""
        result = {}
        for name in param_names:
            value = self.get_param(node_name, name)
            if value is not None:
                result[name] = value
        return result

    def set_param(self, node_name: str, param_name: str, value: Any) -> bool:
        """设置指定节点的参数值，返回是否成功"""
        if not self._node:
            return False

        client = self._node.create_client(SetParameters, f'/{node_name}/set_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn(f'节点 {node_name} 不可达')
            return False

        request = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value = self._make_param_value(value)
        request.parameters = [param]

        future = client.call_async(request)

        timeout = time.time() + 3.0
        while not future.done() and time.time() < timeout:
            time.sleep(0.01)

        if not future.done():
            return False

        try:
            result = future.result()
            if result.results and result.results[0].successful:
                # 更新缓存
                with self._cache_lock:
                    if node_name not in self._cache:
                        self._cache[node_name] = {}
                    self._cache[node_name][param_name] = value
                # 触发回调
                if self._on_param_changed:
                    self._on_param_changed(node_name, param_name, value)
                return True
        except Exception:
            pass

        return False

    def set_params_batch(self, node_name: str, params: dict[str, Any]) -> dict[str, bool]:
        """批量设置参数，返回 {param_name: success}"""
        results = {}
        for name, value in params.items():
            results[name] = self.set_param(node_name, name, value)
        return results

    def get_cached_value(self, node_name: str, param_name: str) -> Any | None:
        """从缓存读取参数值（不发起 ROS2 请求）"""
        with self._cache_lock:
            return self._cache.get(node_name, {}).get(param_name)

    # ── 内部工具 ──────────────────────────────────────

    @staticmethod
    def _extract_value(pv: ParameterValue) -> Any:
        """从 ParameterValue 提取 Python 值"""
        t = pv.type
        if t == ParameterType.PARAMETER_BOOL:
            return pv.bool_value
        elif t == ParameterType.PARAMETER_INTEGER:
            return pv.integer_value
        elif t == ParameterType.PARAMETER_DOUBLE:
            return pv.double_value
        elif t == ParameterType.PARAMETER_STRING:
            return pv.string_value
        elif t == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(pv.byte_array_value)
        elif t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(pv.bool_array_value)
        elif t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(pv.integer_array_value)
        elif t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(pv.double_array_value)
        elif t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(pv.string_array_value)
        return None

    @staticmethod
    def _make_param_value(value: Any) -> ParameterValue:
        """将 Python 值转为 ParameterValue"""
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        return pv
