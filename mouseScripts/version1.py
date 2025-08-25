#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from elfin_robot_msgs.srv import SetInt16
from std_srvs.srv import SetBool
import time

class SpaceMouseSimpleControl(Node):
    def __init__(self):
        super().__init__('spacemouse_simple_control')
        
        # 订阅SpaceMouse输入
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/spacenav/twist',
            self.twist_callback,
            10
        )
        
        # 创建服务客户端
        self.cart_teleop_client = self.create_client(SetInt16, '/cart_teleop')
        self.stop_teleop_client = self.create_client(SetBool, '/stop_teleop')
        
        # 等待服务可用
        self.get_logger().info('等待/cart_teleop服务...')
        while not self.cart_teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/cart_teleop服务不可用，等待中...')
        
        self.get_logger().info('等待/stop_teleop服务...')
        while not self.stop_teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/stop_teleop服务不可用，等待中...')
        
        # 控制参数
        self.deadzone = 0.02        # 死区阈值（降低以提高灵敏度）
        self.last_call_time = 0.0   # 上次调用时间
        self.min_interval = 0.05    # 最小调用间隔50ms（提高响应性）
        self.last_input_time = 0.0  # 上次有输入的时间
        self.stop_timeout = 0.2     # 停止超时时间200ms
        self.is_moving = False      # 是否正在运动
        
        # 参数缓存，避免重复调用相同参数的服务
        self.last_command_data = None    # 上次发送的指令数据
        self.command_repeat_count = 0    # 相同指令的重复次数
        self.max_repeat_count = 3        # 最大允许重复次数（避免完全不发送）
        
        # 创建定时器检查停止
        self.stop_timer = self.create_timer(0.1, self.check_stop_condition)
        
        self.get_logger().info('=== SpaceMouse简单控制节点已启动 ===')
        self.get_logger().info('死区阈值: {}'.format(self.deadzone))
        self.get_logger().info('调用间隔: {}秒'.format(self.min_interval))
        self.get_logger().info('停止超时: {}秒'.format(self.stop_timeout))
        self.get_logger().info('最大重复指令次数: {}'.format(self.max_repeat_count))

    def twist_callback(self, msg):
        """处理SpaceMouse输入"""
        current_time = time.time()
        
        # 频率限制
        if current_time - self.last_call_time < self.min_interval:
            return
        
        # 收集所有轴的值
        axes = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z
        }
        
        # 找到绝对值最大的轴
        max_axis = None
        max_value = 0.0
        
        for axis_name, value in axes.items():
            if abs(value) > abs(max_value) and abs(value) > self.deadzone:
                max_axis = axis_name
                max_value = value
        
        # 如果没有超过死区的输入，记录时间但不退出（用于停止检测）
        if max_axis is None:
            # 没有有效输入，但更新时间用于停止检测
            return
        
        # 映射到机械臂控制指令
        command_data = self.map_to_command(max_axis, max_value)
        
        if command_data is not None:
            # 检查是否与上次指令相同
            if self.should_send_command(command_data):
                self.send_teleop_command(command_data, max_axis, max_value)
                self.last_call_time = current_time
            else:
                # 跳过重复指令，但仍更新时间记录
                self.get_logger().debug(f'跳过重复指令: {command_data} (重复次数: {self.command_repeat_count})')
            
            self.last_input_time = current_time  # 记录有效输入时间
            self.is_moving = True

    def map_to_command(self, axis_name, value):
        """将SpaceMouse轴映射到机械臂控制指令"""
        direction = 1 if value > 0 else -1
        
        mapping = {
            'linear_x': 1,    # X轴平移
            'linear_y': 2,    # Y轴平移
            'linear_z': 3,    # Z轴平移
            'angular_x': 4,   # 绕X轴旋转
            'angular_y': 5,   # 绕Y轴旋转
            'angular_z': 6    # 绕Z轴旋转
        }
        
        if axis_name in mapping:
            return direction * mapping[axis_name]
        
        return None

    def should_send_command(self, command_data):
        """判断是否应该发送指令（避免重复发送相同参数）"""
        if self.last_command_data is None or self.last_command_data != command_data:
            # 新指令或不同指令，重置计数器
            self.last_command_data = command_data
            self.command_repeat_count = 1
            return True
        else:
            # 相同指令
            self.command_repeat_count += 1
            # 允许少量重复以确保指令到达，但避免过度频繁
            if self.command_repeat_count <= self.max_repeat_count:
                return True
            else:
                # 超过最大重复次数，跳过发送
                return False

    def send_teleop_command(self, command_data, axis_name, value):
        """发送遥操作指令"""
        request = SetInt16.Request()
        request.data = command_data
        
        # 异步调用服务
        future = self.cart_teleop_client.call_async(request)
        
        # 记录信息
        direction = "+" if value > 0 else "-"
        self.get_logger().info(
            f'发送指令: {axis_name} {direction} (值:{value:.3f}, 指令:{command_data})'
        )
        
        # 添加回调处理结果
        future.add_done_callback(self.teleop_response_callback)

    def teleop_response_callback(self, future):
        """处理服务响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f'成功: {response.message}')
            else:
                self.get_logger().warn(f'失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')

    def check_stop_condition(self):
        """检查是否需要停止机械臂"""
        current_time = time.time()
        
        # 如果正在运动且超过停止超时时间没有新输入，则停止
        if self.is_moving and (current_time - self.last_input_time) > self.stop_timeout:
            self.stop_robot()
            self.is_moving = False
            # 重置指令缓存
            self.last_command_data = None
            self.command_repeat_count = 0

    def stop_robot(self):
        """停止机械臂运动"""
        request = SetBool.Request()
        request.data = True
        
        future = self.stop_teleop_client.call_async(request)
        self.get_logger().info('🛑 发送停止指令')
        
        # 添加停止响应回调
        future.add_done_callback(self.stop_response_callback)

    def stop_response_callback(self, future):
        """处理停止服务响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('✅ 机械臂已停止')
            else:
                self.get_logger().warn(f'停止失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'停止服务异常: {e}')

def main():
    rclpy.init()
    
    try:
        node = SpaceMouseSimpleControl()
        
        node.get_logger().info('开始监听SpaceMouse输入...')
        node.get_logger().info('移动SpaceMouse来控制机械臂!')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到停止信号，退出程序...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
