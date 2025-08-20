#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from elfin_robot_msgs.srv import SetInt16
from std_srvs.srv import SetBool
import time

class SpaceMouseAdjustableControl(Node):
    def __init__(self):
        super().__init__('spacemouse_adjustable_control')
        
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
        self.get_logger().info('等待服务...')
        while not self.cart_teleop_client.wait_for_service(timeout_sec=1.0):
            pass
        while not self.stop_teleop_client.wait_for_service(timeout_sec=1.0):
            pass
        
        # ===== 可调节的映射参数 =====
        # 你可以修改这些值来调整控制感觉
        self.mapping_config = {
            # 原始映射
            'linear_x': 1,    # SpaceMouse前后 → 机械臂X轴
            'linear_y': 2,    # SpaceMouse左右 → 机械臂Y轴  
            'linear_z': 3,    # SpaceMouse上下 → 机械臂Z轴
            
            # 可选的其他映射方案（注释掉的）
            # 方案1：XY互换
            # 'linear_x': 2,    # SpaceMouse前后 → 机械臂Y轴
            # 'linear_y': 1,    # SpaceMouse左右 → 机械臂X轴  
            
            # 方案2：方向取反  
            # 'linear_x': -1,   # SpaceMouse前后 → 机械臂X轴反向
            # 'linear_y': -2,   # SpaceMouse左右 → 机械臂Y轴反向
            
            # 旋转轴
            'angular_x': 4,   # 绕X轴旋转
            'angular_y': 5,   # 绕Y轴旋转
            'angular_z': 6    # 绕Z轴旋转
        }
        
        # 控制参数
        self.deadzone = 0.02        
        self.last_call_time = 0.0   
        self.min_interval = 0.05    
        self.last_input_time = 0.0  
        self.stop_timeout = 0.2     
        self.is_moving = False      
        
        # 创建定时器检查停止
        self.stop_timer = self.create_timer(0.1, self.check_stop_condition)
        
        self.get_logger().info('=== SpaceMouse可调节控制节点已启动 ===')
        self.get_logger().info('当前映射配置:')
        for axis, cmd in self.mapping_config.items():
            self.get_logger().info(f'  {axis} → 指令{cmd}')
        self.get_logger().info('如果控制方向不对，请修改mapping_config中的映射值')

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
        
        # 如果没有有效输入，返回
        if max_axis is None:
            return
        
        # 映射到机械臂控制指令
        command_data = self.map_to_command(max_axis, max_value)
        
        if command_data is not None:
            self.send_teleop_command(command_data, max_axis, max_value)
            self.last_call_time = current_time
            self.last_input_time = current_time
            self.is_moving = True

    def map_to_command(self, axis_name, value):
        """将SpaceMouse轴映射到机械臂控制指令"""
        if axis_name not in self.mapping_config:
            return None
            
        base_command = self.mapping_config[axis_name]
        direction = 1 if value > 0 else -1
        
        # 如果base_command是负数，说明要反向
        if base_command < 0:
            direction = -direction
            base_command = abs(base_command)
        
        return direction * base_command

    def send_teleop_command(self, command_data, axis_name, value):
        """发送遥操作指令"""
        request = SetInt16.Request()
        request.data = command_data
        
        # 异步调用服务
        future = self.cart_teleop_client.call_async(request)
        
        # 记录信息，添加更详细的方向说明
        direction_map = {
            1: "X+", -1: "X-", 2: "Y+", -2: "Y-", 
            3: "Z+", -3: "Z-", 4: "Rx+", -4: "Rx-",
            5: "Ry+", -5: "Ry-", 6: "Rz+", -6: "Rz-"
        }
        direction_str = direction_map.get(command_data, str(command_data))
        
        self.get_logger().info(
            f'🎮 {axis_name} {value:.3f} → {direction_str}'
        )

    def check_stop_condition(self):
        """检查是否需要停止机械臂"""
        current_time = time.time()
        
        if self.is_moving and (current_time - self.last_input_time) > self.stop_timeout:
            self.stop_robot()
            self.is_moving = False

    def stop_robot(self):
        """停止机械臂运动"""
        request = SetBool.Request()
        request.data = True
        
        future = self.stop_teleop_client.call_async(request)
        self.get_logger().info('🛑 自动停止')

def main():
    rclpy.init()
    
    try:
        node = SpaceMouseAdjustableControl()
        
        print("\n" + "="*60)
        print("🎮 SpaceMouse控制测试")
        print("📝 请测试各个方向的控制效果：")
        print("   - 前后推拉：应该控制机械臂X轴或Y轴")
        print("   - 左右推拉：应该控制机械臂另一个轴")  
        print("   - 上下推拉：应该控制机械臂Z轴")
        print("💡 如果方向不对，停止程序修改mapping_config")
        print("="*60)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n🛑 程序已停止')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
