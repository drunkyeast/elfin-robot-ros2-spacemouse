#!/bin/bash

# 交互式机械臂控制脚本
# 使用方法：./interactive_arm_control.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示主菜单
show_main_menu() {
    clear
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}    机械臂交互式控制菜单${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${GREEN}请选择控制类型：${NC}"
    echo "1. 基础关节控制"
    echo "2. 笛卡尔空间控制"
    echo "3. 有趣的组合动作"
    echo "4. 退出"
    echo ""
    echo -n "请输入选择 (1-4): "
}

# 显示基础关节控制菜单
show_joint_menu() {
    clear
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}        基础关节控制${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${GREEN}请选择关节动作：${NC}"
    echo "1. 回到零位"
    echo "2. 关节1向前摆动45度"
    echo "3. 关节1向后摆动45度"
    echo "4. 关节2向左摆动30度"
    echo "5. 关节2向右摆动30度"
    echo "6. 关节3向上摆动30度"
    echo "7. 关节3向下摆动30度"
    echo "8. 返回主菜单"
    echo ""
    echo -n "请输入选择 (1-8): "
}

# 显示笛卡尔空间控制菜单
show_cartesian_menu() {
    clear
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}      笛卡尔空间控制${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${GREEN}请选择空间位置：${NC}"
    echo "1. 移动到前方位置 (x=0.4m, y=0.0m, z=0.4m)"
    echo "2. 移动到左侧位置 (x=0.4m, y=0.1m, z=0.4m)"
    echo "3. 移动到右侧位置 (x=0.4m, y=-0.1m, z=0.4m)"
    echo "4. 移动到高处位置 (x=0.4m, y=0.0m, z=0.5m)"
    echo "5. 返回主菜单"
    echo ""
    echo -n "请输入选择 (1-5): "
}

# 显示组合动作菜单
show_combination_menu() {
    clear
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}        有趣的组合动作${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${GREEN}请选择组合动作：${NC}"
    echo "1. 机械臂\"点头\"动作"
    echo "2. 机械臂\"挥手\"动作"
    echo "3. 机械臂\"鞠躬\"动作"
    echo "4. 返回主菜单"
    echo ""
    echo -n "请输入选择 (1-4): "
}

# 执行关节控制命令
execute_joint_command() {
    local choice=$1
    
    case $choice in
        1)
            echo -e "${YELLOW}即将执行：回到零位${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        2)
            echo -e "${YELLOW}即将执行：关节1向前摆动45度${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        3)
            echo -e "${YELLOW}即将执行：关节1向后摆动45度${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [-0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [-0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        4)
            echo -e "${YELLOW}即将执行：关节2向左摆动30度${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.524, 0.0, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.524, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        5)
            echo -e "${YELLOW}即将执行：关节2向右摆动30度${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, -0.524, 0.0, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, -0.524, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        6)
            echo -e "${YELLOW}即将执行：关节3向上摆动30度${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.524, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.524, 0.0, 0.0, 0.0]}"
            ;;
        7)
            echo -e "${YELLOW}即将执行：关节3向下摆动30度${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, -0.524, 0.0, 0.0, 0.0]}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, -0.524, 0.0, 0.0, 0.0]}"
            ;;
        8)
            return
            ;;
        *)
            echo -e "${RED}无效选择，请重新输入${NC}"
            sleep 2
            ;;
    esac
    
    echo ""
    echo -e "${GREEN}命令执行完成！${NC}"
    echo -n "按回车键继续..."
    read
}

# 执行笛卡尔空间控制命令
execute_cartesian_command() {
    local choice=$1
    
    case $choice in
        1)
            echo -e "${YELLOW}即将执行：移动到前方位置 (x=0.4m, y=0.0m, z=0.4m)${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: 0.0, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: 0.0, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
            ;;
        2)
            echo -e "${YELLOW}即将执行：移动到左侧位置 (x=0.4m, y=0.1m, z=0.4m)${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: 0.1, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: 0.1, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
            ;;
        3)
            echo -e "${YELLOW}即将执行：移动到右侧位置 (x=0.4m, y=-0.1m, z=0.4m)${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: -0.1, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: -0.1, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
            ;;
        4)
            echo -e "${YELLOW}即将执行：移动到高处位置 (x=0.4m, y=0.0m, z=0.5m)${NC}"
            echo ""
            echo -e "${GREEN}命令预览：${NC}"
            echo 'ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"'
            echo ""
            echo -n "按回车键执行命令，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行命令...${NC}"
            ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"elfin_base_link\"}, pose: {position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
            ;;
        5)
            return
            ;;
        *)
            echo -e "${RED}无效选择，请重新输入${NC}"
            sleep 2
            ;;
    esac
    
    echo ""
    echo -e "${GREEN}命令执行完成！${NC}"
    echo -n "按回车键继续..."
    read
}

# 执行组合动作命令
execute_combination_command() {
    local choice=$1
    
    case $choice in
        1)
            echo -e "${YELLOW}即将执行：机械臂\"点头\"动作${NC}"
            echo ""
            echo -e "${GREEN}动作说明：${NC}"
            echo "1. 关节1向前摆动到0.3弧度"
            echo "2. 等待2秒"
            echo "3. 回到零位"
            echo ""
            echo -n "按回车键开始执行，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行点头动作...${NC}"
            
            # 向前摆动
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            echo "等待2秒..."
            sleep 2
            
            # 回到零位
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        2)
            echo -e "${YELLOW}即将执行：机械臂\"挥手\"动作${NC}"
            echo ""
            echo -e "${GREEN}动作说明：${NC}"
            echo "1. 关节2向左摆动到0.4弧度"
            echo "2. 等待2秒"
            echo "3. 关节2向右摆动到-0.4弧度"
            echo "4. 等待2秒"
            echo "5. 回到中间位置"
            echo ""
            echo -n "按回车键开始执行，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行挥手动作...${NC}"
            
            # 向左摆动
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.4, 0.0, 0.0, 0.0, 0.0]}"
            echo "等待2秒..."
            sleep 2
            
            # 向右摆动
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, -0.4, 0.0, 0.0, 0.0, 0.0]}"
            echo "等待2秒..."
            sleep 2
            
            # 回到中间
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        3)
            echo -e "${YELLOW}即将执行：机械臂\"鞠躬\"动作${NC}"
            echo ""
            echo -e "${GREEN}动作说明：${NC}"
            echo "1. 关节1向前摆动到0.4弧度（鞠躬）"
            echo "2. 等待3秒"
            echo "3. 回到零位"
            echo ""
            echo -n "按回车键开始执行，或按Ctrl+C取消: "
            read
            echo -e "${GREEN}执行鞠躬动作...${NC}"
            
            # 向前摆动（鞠躬）
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.4, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            echo "等待3秒..."
            sleep 3
            
            # 回到零位
            ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, name: [\"elfin_joint1\", \"elfin_joint2\", \"elfin_joint3\", \"elfin_joint4\", \"elfin_joint5\", \"elfin_joint6\"], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
            ;;
        4)
            return
            ;;
        *)
            echo -e "${RED}无效选择，请重新输入${NC}"
            sleep 2
            ;;
    esac
    
    echo ""
    echo -e "${GREEN}组合动作执行完成！${NC}"
    echo -n "按回车键继续..."
    read
}

# 主程序循环
main() {
    while true; do
        show_main_menu
        read -r choice
        
        case $choice in
            1)
                while true; do
                    show_joint_menu
                    read -r joint_choice
                    if [ "$joint_choice" = "8" ]; then
                        break
                    fi
                    execute_joint_command "$joint_choice"
                done
                ;;
            2)
                while true; do
                    show_cartesian_menu
                    read -r cartesian_choice
                    if [ "$cartesian_choice" = "5" ]; then
                        break
                    fi
                    execute_cartesian_command "$cartesian_choice"
                done
                ;;
            3)
                while true; do
                    show_combination_menu
                    read -r combination_choice
                    if [ "$combination_choice" = "4" ]; then
                        break
                    fi
                    execute_combination_command "$combination_choice"
                done
                ;;
            4)
                echo -e "${GREEN}感谢使用机械臂控制脚本！${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}无效选择，请重新输入${NC}"
                sleep 2
                ;;
        esac
    done
}

# 检查ROS2环境
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误：未找到ROS2命令${NC}"
        echo "请确保ROS2环境已正确设置"
        exit 1
    fi
    
    echo -e "${GREEN}ROS2环境检查通过${NC}"
    echo ""
}

# 显示欢迎信息
show_welcome() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}  欢迎使用机械臂交互式控制脚本${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${YELLOW}⚠️  安全提示：${NC}"
    echo "1. 确保机械臂周围安全"
    echo "2. 在RViz中观察运动轨迹"
    echo "3. 从小幅度运动开始测试"
    echo "4. 随时可以按Ctrl+C停止"
    echo ""
    echo -e "${GREEN}功能说明：${NC}"
    echo "• 基础关节控制：控制单个关节的运动"
    echo "• 笛卡尔空间控制：控制末端执行器的空间位置"
    echo "• 有趣的组合动作：预设的机械臂表演动作"
    echo ""
    echo -n "按回车键开始..."
    read
}

# 程序入口
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    check_ros2
    show_welcome
    main
fi
