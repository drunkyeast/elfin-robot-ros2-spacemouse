#!/bin/bash

# 简单的spacenav启动脚本
echo "启动spacenav..."

# 设置库路径
export LD_LIBRARY_PATH=/opt/ros/foxy/lib/spacenav:$LD_LIBRARY_PATH

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 启动spacenav节点
echo "正在启动spacenav节点..."
ros2 run spacenav spacenav_node
