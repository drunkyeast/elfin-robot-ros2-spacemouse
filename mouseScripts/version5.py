#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from elfin_robot_msgs.srv import SetInt16
from std_srvs.srv import SetBool
import time
import threading

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
        
        # 控制参数 - 更激进的停止策略
        self.deadzone = 0.02        # 死区阈值
        self.last_call_time = 0.0   # 上次调用时间
        self.min_interval = 0.04    # 最小调用间隔40ms
        self.last_input_time = 0.0  # 上次有输入的时间
        self.last_valid_input_time = 0.0  # 上次有效输入时间
        self.stop_timeout = 0.08    # 停止超时时间80ms（更激进）
        self.is_moving = False      # 是否正在运动
        
        # 停止状态管理 - 加强版
        self.stop_sent = False      # 是否已发送停止指令
        self.stop_confirmed = False # 停止是否已确认
        self.stop_retry_count = 0   # 停止重试次数
        self.max_stop_retries = 3   # 最大停止重试次数
        
        # 线程锁，防止停止指令竞态
        self.stop_lock = threading.Lock()
        
        # 创建定时器检查停止 - 更频繁
        self.stop_timer = self.create_timer(0.015, self.check_stop_condition)  # 15ms检查一次
        
        self.get_logger().info('=== SpaceMouse控制节点已启动 (version5 - 激进停止策略) ===')
        self.get_logger().info('死区阈值: {}'.format(self.deadzone))
        self.get_logger().info('调用间隔: {}秒'.format(self.min_interval))
        self.get_logger().info('停止超时: {}秒 (激进模式)'.format(self.stop_timeout))
        self.get_logger().info('停止检查频率: 15ms')

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
        
        # 有有效输入 - 重置所有停止状态
        with self.stop_lock:
            self.stop_sent = False
            self.stop_confirmed = False
            self.stop_retry_count = 0
        
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
        
        # 异步调用服务（保持响应性）
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
                self.get_logger().debug(f'运动指令成功: {response.message}')
            else:
                self.get_logger().warn(f'运动指令失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'运动服务调用异常: {e}')

    def check_stop_condition(self):
        """检查是否需要停止机械臂"""
        if not self.is_moving:
            return
            
        current_time = time.time()
        
        # 检查是否超过停止超时时间没有有效输入
        if (current_time - self.last_valid_input_time) > self.stop_timeout:
            with self.stop_lock:
                if not self.stop_sent or (self.stop_retry_count < self.max_stop_retries and not self.stop_confirmed):
                    self.stop_robot_aggressive()
                    self.is_moving = False

    def stop_robot_aggressive(self):
        """激进停止策略 - 同步调用 + 重复发送"""
        if self.stop_retry_count >= self.max_stop_retries:
            return
            
        self.stop_retry_count += 1
        self.stop_sent = True
        
        request = SetBool.Request()
        request.data = True
        
        self.get_logger().info(f'🛑 发送停止指令 (第{self.stop_retry_count}次)')
        
        try:
            # 使用同步调用，确保停止指令立即处理
            response = self.stop_teleop_client.call(request, timeout_sec=0.1)
            
            if response.success:
                self.get_logger().info('✅ 停止指令确认成功')
                self.stop_confirmed = True
            else:
                self.get_logger().warn(f'⚠️ 停止确认失败: {response.message}')
                if self.stop_retry_count < self.max_stop_retries:
                    # 失败后立即重试
                    self.get_logger().info('🔄 立即重试停止指令')
                    time.sleep(0.01)  # 短暂等待后重试
                    self.stop_robot_aggressive()
                    
        except Exception as e:
            self.get_logger().error(f'❌ 停止服务异常: {e}')
            if self.stop_retry_count < self.max_stop_retries:
                # 异常后也重试
                self.get_logger().info('🔄 异常后重试停止指令')
                time.sleep(0.01)
                self.stop_robot_aggressive()

def main():
    rclpy.init()
    
    try:
        node = SpaceMouseSimpleControl()
        
        node.get_logger().info('开始监听SpaceMouse输入...')
        node.get_logger().info('移动SpaceMouse来控制机械臂!')
        node.get_logger().info('⚡ Version5: 激进停止策略 - 同步调用 + 重复确认')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到停止信号，退出程序...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
