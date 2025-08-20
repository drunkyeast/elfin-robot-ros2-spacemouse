#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import math
import time

class SpaceMouseCartesianControl(Node):
    def __init__(self):
        super().__init__('spacemouse_cartesian_control')
        
        # 订阅3D鼠标输入
        self.joy_subscriber = self.create_subscription(
            Joy, 
            '/spacenav/joy', 
            self.joy_callback, 
            10
        )
        
        # 发布笛卡尔目标位置到机械臂
        self.cart_publisher = self.create_publisher(
            PoseStamped, 
            '/cart_goal', 
            10
        )
        
        # 当前末端位置状态 (初始化为安全位置)
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "elfin_base_link"
        self.current_pose.pose.position.x = 0.4
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.4
        # 保持末端向下的姿态
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 1.0  # 向下姿态
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 0.0
        
        # 控制参数
        self.translation_scale = 0.02   # 平移灵敏度: 2cm/unit
        self.rotation_scale = 0.03      # 旋转灵敏度: 0.03rad/unit  
        self.deadzone = 0.05            # 死区阈值
        
        # 工作空间限制
        self.workspace_limits = {
            'x': (0.2, 0.7),    # X方向范围
            'y': (-0.3, 0.3),   # Y方向范围
            'z': (0.1, 0.6)     # Z方向范围
        }
        
        # 控制状态
        self.has_new_input = False
        self.target_pose = None
        self.last_command_time = 0.0
        
        # 创建定时器，控制发送频率
        self.command_timer = self.create_timer(0.2, self.send_command_if_needed)  # 5Hz
        
        # 初始化完成
        self.get_logger().info('=== SpaceMouse笛卡尔控制节点已启动 ===')
        self.get_logger().info(f'平移灵敏度: {self.translation_scale} m/unit')
        self.get_logger().info(f'旋转灵敏度: {self.rotation_scale} rad/unit')
        self.get_logger().info(f'死区阈值: {self.deadzone}')
        self.get_logger().info(f'初始位置: x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, z={self.current_pose.pose.position.z}')
        self.get_logger().info('等待SpaceMouse输入...')
        
    def joy_callback(self, msg):
        """处理3D鼠标输入回调"""
        if len(msg.axes) < 6:
            self.get_logger().warn('SpaceMouse数据不完整，需要6个轴')
            return
        
        # 检查是否有有效输入（超过死区）
        max_input = max(abs(axis) for axis in msg.axes)
        if max_input < self.deadzone:
            return  # 输入太小，忽略
        
        # 复制当前位置作为基准
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "elfin_base_link"
        self.target_pose.pose = self.current_pose.pose
        
        # 平移控制 (axes[0-2])
        delta_x = msg.axes[0] * self.translation_scale
        delta_y = msg.axes[1] * self.translation_scale
        delta_z = msg.axes[2] * self.translation_scale
        
        self.target_pose.pose.position.x += delta_x
        self.target_pose.pose.position.y += delta_y
        self.target_pose.pose.position.z += delta_z
        
        # 旋转控制 (axes[3-5]) - 简化处理，保持基本姿态
        # 这里暂时只实现平移，旋转可以后续添加
        rotation_input = any(abs(msg.axes[i]) > self.deadzone for i in range(3, 6))
        if rotation_input:
            # 可以根据需要添加姿态调整逻辑
            pass
        
        # 应用工作空间限制
        self.apply_workspace_limits()
        
        # 标记有新输入
        self.has_new_input = True
        
        # 记录输入信息（降低频率）
        current_time = time.time()
        if current_time - self.last_command_time > 1.0:  # 每秒最多一次调试信息
            active_axes = [f"{i}:{msg.axes[i]:.3f}" for i in range(6) if abs(msg.axes[i]) > self.deadzone]
            if active_axes:
                self.get_logger().info(f'SpaceMouse输入: {", ".join(active_axes)}')
    
    def apply_workspace_limits(self):
        """应用工作空间限制"""
        if self.target_pose is None:
            return
        
        pos = self.target_pose.pose.position
        
        # 限制X方向
        pos.x = max(self.workspace_limits['x'][0], 
                   min(self.workspace_limits['x'][1], pos.x))
        
        # 限制Y方向  
        pos.y = max(self.workspace_limits['y'][0],
                   min(self.workspace_limits['y'][1], pos.y))
        
        # 限制Z方向
        pos.z = max(self.workspace_limits['z'][0],
                   min(self.workspace_limits['z'][1], pos.z))
    
    def send_command_if_needed(self):
        """定时检查并发送控制命令"""
        if not self.has_new_input or self.target_pose is None:
            return
        
        current_time = time.time()
        
        # 更新消息时间戳
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 发布目标位置
        self.cart_publisher.publish(self.target_pose)
        
        # 更新当前位置状态
        self.current_pose.pose.position = self.target_pose.pose.position
        
        # 记录发送信息
        pos = self.target_pose.pose.position
        self.get_logger().info(
            f'发送目标位置: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}'
        )
        
        # 重置状态
        self.has_new_input = False
        self.last_command_time = current_time
    
    def reset_to_home_position(self):
        """重置到初始位置"""
        home_pose = PoseStamped()
        home_pose.header.frame_id = "elfin_base_link"
        home_pose.header.stamp = self.get_clock().now().to_msg()
        home_pose.pose.position.x = 0.4
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = 0.4
        home_pose.pose.orientation.x = 0.0
        home_pose.pose.orientation.y = 1.0
        home_pose.pose.orientation.z = 0.0
        home_pose.pose.orientation.w = 0.0
        
        self.cart_publisher.publish(home_pose)
        self.current_pose = home_pose
        
        self.get_logger().info('机械臂重置到初始位置')

def main():
    rclpy.init()
    
    node = SpaceMouseCartesianControl()
    
    try:
        # 等待2秒让系统初始化
        time.sleep(2.0)
        
        # 发送初始位置命令
        node.reset_to_home_position()
        node.get_logger().info('========================================')
        node.get_logger().info('3D鼠标控制已就绪！')
        node.get_logger().info('使用SpaceMouse移动机械臂末端位置')
        node.get_logger().info('按 Ctrl+C 退出程序')
        node.get_logger().info('========================================')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
