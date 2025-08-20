#!/bin/bash

# SpaceMouse机械臂控制测试脚本
# 使用方法：./test_spacemouse_control.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  SpaceMouse机械臂控制测试${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误：未找到ROS2命令${NC}"
    echo "请确保ROS2环境已正确设置"
    exit 1
fi

echo -e "${GREEN}ROS2环境检查通过${NC}"

# 设置环境
echo "设置ROS2环境..."
source /opt/ros/foxy/setup.bash
export LD_LIBRARY_PATH=/opt/ros/foxy/lib/spacenav:$LD_LIBRARY_PATH

# 检查是否在catkin_ws目录
if [[ ! -d "src/elfin_robot_ros2" ]]; then
    echo -e "${YELLOW}切换到catkin_ws目录...${NC}"
    cd /home/ubuntu/catkin_ws
fi

# 源化工作空间
if [ -f "install/setup.bash" ]; then
    echo "源化工作空间..."
    source install/setup.bash
else
    echo -e "${YELLOW}警告：未找到install/setup.bash，请先编译工作空间${NC}"
fi

echo ""
echo -e "${YELLOW}⚠️  测试前准备检查：${NC}"
echo "1. 确保已启动机械臂仿真环境"
echo "2. 确保SpaceMouse已连接并驱动正常"
echo "3. 确保spacenavd服务正在运行"
echo "4. 机械臂周围空间安全"
echo ""

# 检查必要的topic
echo -e "${BLUE}检查ROS2 topic状态...${NC}"

# 检查spacenav topic
if ros2 topic list | grep -q "/spacenav/joy"; then
    echo -e "${GREEN}✓ SpaceMouse topic (/spacenav/joy) 存在${NC}"
else
    echo -e "${RED}✗ SpaceMouse topic (/spacenav/joy) 不存在${NC}"
    echo "请先启动spacenav节点：ros2 run spacenav spacenav_node"
    echo "或使用脚本：./start_spacenav_simple.sh"
fi

# 检查机械臂控制topic
if ros2 topic list | grep -q "/cart_goal"; then
    echo -e "${GREEN}✓ 机械臂控制topic (/cart_goal) 存在${NC}"
else
    echo -e "${YELLOW}? 机械臂控制topic (/cart_goal) 不存在${NC}"
    echo "请确保机械臂控制节点已启动"
fi

echo ""
echo -e "${BLUE}准备启动SpaceMouse控制节点...${NC}"
echo -n "按回车键继续，或按Ctrl+C取消: "
read

echo ""
echo -e "${GREEN}启动SpaceMouse机械臂控制节点...${NC}"
echo -e "${YELLOW}使用说明：${NC}"
echo "• 移动SpaceMouse控制机械臂末端位置"  
echo "• 平移：X轴(前后)、Y轴(左右)、Z轴(上下)"
echo "• 当前版本：仅支持位置控制，姿态保持固定"
echo "• 按Ctrl+C停止控制"
echo ""

# 启动控制节点
cd /home/ubuntu/catkin_ws
python3 myscripts/spacemouse_cartesian_control.py
