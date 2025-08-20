#!/bin/bash

# Elfin机械臂循环运动脚本
# 使用方法: ./elfin_cycle_movement.sh [间隔秒数]
# 例如: ./elfin_cycle_movement.sh 5

# 超参数配置
INTERVAL=${1:-5}  # 默认5秒间隔，可通过命令行参数修改

# 预定义的位置点 (x, y, z)
declare -a POSITIONS=(
    "0.4 0.1 0.3"    # 位置1: 右前方
    "0.4 -0.1 0.3"   # 位置2: 左前方  
    "0.3 0.0 0.4"    # 位置3: 正前方高位
    "0.5 0.0 0.2"    # 位置4: 正前方低位
    "0.35 0.15 0.35" # 位置5: 右侧中位
)

# 固定的姿态 (四元数)
ORIENTATION="x: 0.0, y: 0.0, z: 0.0, w: 1.0"

echo "=== Elfin机械臂循环运动脚本 ==="
echo "间隔时间: ${INTERVAL}秒"
echo "位置点数量: ${#POSITIONS[@]}"
echo "按 Ctrl+C 停止运动"
echo "=========================="

# 循环计数器
count=1

# 无限循环
while true; do
    for i in "${!POSITIONS[@]}"; do
        # 解析位置坐标
        pos=(${POSITIONS[$i]})
        x=${pos[0]}
        y=${pos[1]}
        z=${pos[2]}
        
        echo "[$(date '+%H:%M:%S')] 第${count}轮 - 移动到位置$((i+1)): (${x}, ${y}, ${z})"
        
        # 发送移动命令
        ros2 topic pub --once /cart_goal geometry_msgs/msg/PoseStamped "{
            header: {frame_id: 'elfin_base_link'}, 
            pose: {
                position: {x: ${x}, y: ${y}, z: ${z}}, 
                orientation: {${ORIENTATION}}
            }
        }" > /dev/null 2>&1
        
        # 等待指定时间
        sleep $INTERVAL
    done
    
    echo "--- 完成第${count}轮循环 ---"
    echo "等待10秒后开始下一轮..."
    sleep 10
    ((count++))
done
