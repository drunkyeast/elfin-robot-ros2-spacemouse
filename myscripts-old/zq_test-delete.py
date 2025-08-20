#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
import math
import time

class Arm3DMouseControl(Node):
    def __init__(self):
        super().__init__('arm_3d_mouse_control')
        
        # 创建发布者 - 发送目标位置到机械臂
        self.arm_publisher = self.create_publisher(
            PoseStamped, 
            '/cart_goal', 
            10
        )
        
        
        # 创建订阅者 - 接收3D鼠标输入
        self.mouse_subscriber = self.create_subscription(
            Joy,
            '/spacenav/joy',
            self.mouse_callback,
            10
        )
        
        # 当前机械臂末端位置和姿态
        self.current_position = Point(x=0.4, y=0.0, z=0.4)  # 初始位置
        self.current_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # 初始姿态
        
        # 控制参数
        self.scale_factor = 0.005  # 鼠标灵敏度：0.005米/单位
        self.orientation_scale = 0.01  # 姿态变化灵敏度：0.01弧度/单位
        
        # 位置阈值控制 - 只有变化超过阈值才发送命令
        self.position_threshold = 0.02  # 2cm阈值
        self.orientation_threshold = 0.05  # 0.05弧度阈值
        
        # 发送频率控制
        self.min_send_interval = 0.5  # 最小发送间隔：0.5秒
        self.last_send_time = 0.0
        
        # 位置限制（机械臂工作空间）
        self.position_limits = {
            'x': (0.2, 0.8),    # x方向范围
            'y': (-0.3, 0.3),   # y方向范围  
            'z': (0.1, 0.7)     # z方向范围
        }
        
        # 状态跟踪
        self.last_mouse_input = [0.0] * 6
        self.position_changed = False
        self.orientation_changed = False
        
        # 创建定时器，定期检查是否需要发送命令
        self.timer = self.create_timer(10, self.check_and_send)  # 10Hz检查
        
        self.get_logger().info('3D鼠标机械臂控制节点已启动')
        self.get_logger().info(f'位置灵敏度: {self.scale_factor} m/unit')
        self.get_logger().info(f'位置阈值: {self.position_threshold} m')
        self.get_logger().info(f'最小发送间隔: {self.min_send_interval} s')
        
    def mouse_callback(self, msg):
        """处理3D鼠标输入"""
        if len(msg.axes) < 6:
            return
            
        current_time = time.time()
        
        # 检查位置变化（前3个axes）
        for i in range(3):
            if abs(msg.axes[i]) > 0.01:  # 忽略很小的输入
                # 计算位置增量
                delta = msg.axes[i] * self.scale_factor
                
                if i == 0:  # X方向
                    self.current_position.x += delta
                elif i == 1:  # Y方向
                    self.current_position.y += delta
                elif i == 2:  # Z方向
                    self.current_position.z += delta
                
                self.position_changed = True
        
        # 检查姿态变化（后3个axes）
        for i in range(3, 6):
            if abs(msg.axes[i]) > 0.01:  # 忽略很小的输入
                # 简化处理：暂时保持姿态不变，后续可以扩展
                # 这里可以添加四元数积分逻辑
                pass
        
        # 应用位置限制
        self.apply_position_limits()
        
        # 记录鼠标输入用于调试
        self.last_mouse_input = list(msg.axes)
        
        # 打印调试信息（降低频率）
        if int(current_time * 5) % 5 == 0:  # 每5秒打印一次
            self.get_logger().info(f'鼠标输入: axes={[f"{x:.3f}" for x in msg.axes[:3]]}')
            self.get_logger().info(f'当前位置: x={self.current_position.x:.3f}, y={self.current_position.y:.3f}, z={self.current_position.z:.3f}')
    
    def apply_position_limits(self):
        """应用位置限制，确保在机械臂工作空间内"""
        self.current_position.x = max(
            self.position_limits['x'][0],
            min(self.position_limits['x'][1], self.current_position.x)
        )
        self.current_position.y = max(
            self.position_limits['y'][0],
            min(self.position_limits['y'][1], self.current_position.y)
        )
        self.current_position.z = max(
            self.position_limits['z'][0],
            min(self.position_limits['z'][1], self.current_position.z)
        )
    
    def check_and_send(self):
        """检查是否需要发送位置命令"""
        current_time = time.time()
        
        # 检查时间间隔和位置变化
        if (self.position_changed and 
            current_time - self.last_send_time >= self.min_send_interval):
            
            # 发送位置命令
            self.send_position_command()
            
            # 重置状态
            self.position_changed = False
            self.last_send_time = current_time
            
            self.get_logger().info(f'发送位置命令: x={self.current_position.x:.3f}, y={self.current_position.y:.3f}, z={self.current_position.z:.3f}')
    
    def send_position_command(self):
        """发送位置命令到机械臂"""
        # 创建消息
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "elfin_base_link"
        msg.pose.position = self.current_position
        msg.pose.orientation = self.current_orientation
        
        # 发布消息
        self.arm_publisher.publish(msg)

def main():
    rclpy.init()
    
    node = Arm3DMouseControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('3D鼠标控制节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()