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
        
        # 控制参数 - 回到简单有效的设置
        self.deadzone = 0.02        # 死区阈值
        self.last_call_time = 0.0   # 上次调用时间
        self.min_interval = 0.05    # 最小调用间隔50ms（回到v1的设置）
        self.last_input_time = 0.0  # 上次有输入的时间
        self.last_valid_input_time = 0.0  # 上次有效输入时间
        self.stop_timeout = 0.15    # 停止超时时间150ms
        self.is_moving = False      # 是否正在运动
        
        # 简单的停止管理
        self.stop_sent = False      # 是否已发送停止指令
        
        # 创建定时器检查停止
        self.stop_timer = self.create_timer(0.05, self.check_stop_condition)  # 50ms检查一次
        
        self.get_logger().info('=== SpaceMouse控制节点已启动 (version6 - 回归简单) ===')
        self.get_logger().info('死区阈值: {}'.format(self.deadzone))
        self.get_logger().info('调用间隔: {}秒'.format(self.min_interval))
        self.get_logger().info('停止超时: {}秒'.format(self.stop_timeout))

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
        
        # 如果没有超过死区的输入
        if max_axis is None:
            self.last_input_time = current_time
            return
        
        # 有有效输入 - 重置停止标志
        self.stop_sent = False
        
        # 映射到机械臂控制指令
        command_data = self.map_to_command(max_axis, max_value)
        
        if command_data is not None:
            self.send_teleop_command(command_data, max_axis, max_value)
            self.last_call_time = current_time
            self.last_input_time = current_time
            self.last_valid_input_time = current_time
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
        if not self.is_moving or self.stop_sent:
            return
            
        current_time = time.time()
        
        # 检查是否超过停止超时时间没有有效输入
        if (current_time - self.last_valid_input_time) > self.stop_timeout:
            self.stop_robot()

    def stop_robot(self):
        """停止机械臂运动 - 立即发送多个停止指令"""
        if self.stop_sent:
            return
            
        self.stop_sent = True
        self.is_moving = False
        
        # 连续发送3个停止指令，确保至少一个生效
        for i in range(3):
            request = SetBool.Request()
            request.data = True
            
            future = self.stop_teleop_client.call_async(request)
            self.get_logger().info(f'🛑 发送停止指令 #{i+1}')
            
            # 只对第一个停止指令添加回调
            if i == 0:
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
        node.get_logger().info('🎯 Version6: 简单有效 - 多重停止指令')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到停止信号，退出程序...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
