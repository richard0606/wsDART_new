#!/bin/bash

USER_NAME=${SUDO_USER:-$(whoami)}
HOME_DIR=$(eval echo "~$USER_NAME")
export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"
source /opt/ros/humble/setup.bash

# systemd 服务默认 KillMode=control-group，停止服务时系统会清理本 cgroup
# 内 rm_watch_dog.sh 及其拉起的全部子进程。
# 这里绝对不要用 pkill -f 匹配节点名/ros2 等通用字符串：
# 那会误杀命令行恰好包含这些词的其他进程（实测会杀掉 ssh 会话和编辑器）。
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
