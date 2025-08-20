#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import sys

class ElfinCycleMovement(Node):
    def __init__(self, interval=5):
        super().__init__('elfin_cycle_movement')
        
        # 创建发布者
        self.publisher = self.create_publisher(PoseStamped, '/cart_goal', 10)
        
        # 超参数
        self.interval = interval
        
        # 预定义位置点 (x, y, z)
        self.positions = [
            (0.4, 0.1, 0.3),    # 位置1: 右前方
            (0.4, -0.1, 0.3),   # 位置2: 左前方  
            (0.3, 0.0, 0.4),    # 位置3: 正前方高位
            (0.5, 0.0, 0.2),    # 位置4: 正前方低位
            (0.35, 0.15, 0.35), # 位置5: 右侧中位
        ]
        
        print(f"=== Elfin机械臂循环运动 (Python版) ===")
        print(f"间隔时间: {self.interval}秒")
        print(f"位置点数量: {len(self.positions)}")
        print("按 Ctrl+C 停止运动")
        print("=" * 40)

    def create_pose_msg(self, x, y, z):
        """创建PoseStamped消息"""
        msg = PoseStamped()
        msg.header.frame_id = 'elfin_base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        # 设置姿态 (无旋转)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        return msg

    def run_cycle(self):
        """运行循环运动"""
        count = 1
        
        try:
            while rclpy.ok():
                for i, (x, y, z) in enumerate(self.positions):
                    current_time = time.strftime('%H:%M:%S')
                    print(f"[{current_time}] 第{count}轮 - 移动到位置{i+1}: ({x}, {y}, {z})")
                    
                    # 创建并发送消息
                    pose_msg = self.create_pose_msg(x, y, z)
                    self.publisher.publish(pose_msg)
                    
                    # 等待
                    time.sleep(self.interval)
                
                print(f"--- 完成第{count}轮循环 ---")
                print("等待10秒后开始下一轮...")
                time.sleep(10)
                count += 1
                
        except KeyboardInterrupt:
            print("\n收到停止信号，退出程序...")

def main():
    # 解析命令行参数
    interval = 5  # 默认值
    if len(sys.argv) > 1:
        try:
            interval = float(sys.argv[1])
        except ValueError:
            print("错误: 间隔时间必须是数字")
            sys.exit(1)
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建节点并运行
    node = ElfinCycleMovement(interval)
    node.run_cycle()
    
    # 清理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
