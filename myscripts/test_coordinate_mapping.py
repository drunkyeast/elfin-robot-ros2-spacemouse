#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from elfin_robot_msgs.srv import SetInt16
import time

class CoordinateMappingTest(Node):
    def __init__(self):
        super().__init__('coordinate_mapping_test')
        
        # 创建服务客户端
        self.cart_teleop_client = self.create_client(SetInt16, '/cart_teleop')
        
        # 等待服务可用
        while not self.cart_teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待/cart_teleop服务...')
        
        self.get_logger().info('=== 坐标轴映射测试 ===')

    def test_direction(self, command, description):
        
        """测试指定方向的运动"""
        self.get_logger().info(f'\n🔍 测试: {description}')
        self.get_logger().info(f'发送指令: data={command}')
        
        request = SetInt16.Request()
        request.data = command
        
        try:
            future = self.cart_teleop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f'✅ 成功: {response.message}')
                else:
                    self.get_logger().error(f'❌ 失败: {response.message}')
            else:
                self.get_logger().error('❌ 服务调用超时')
                
        except Exception as e:
            self.get_logger().error(f'❌ 异常: {e}')
        
        # 等待用户观察
        input('\n👀 观察机械臂运动方向，然后按Enter继续...')

def main():
    rclpy.init()
    
    try:
        node = CoordinateMappingTest()
        
        # 测试序列
        tests = [
            (1, "X+ 方向 (应该是机械臂的 X 轴正方向)"),
            (-1, "X- 方向 (应该是机械臂的 X 轴负方向)"),
            (2, "Y+ 方向 (应该是机械臂的 Y 轴正方向)"),
            (-2, "Y- 方向 (应该是机械臂的 Y 轴负方向)"),
            (3, "Z+ 方向 (应该是机械臂的 Z 轴正方向 - 向上)"),
            (-3, "Z- 方向 (应该是机械臂的 Z 轴负方向 - 向下)"),
        ]
        
        print("\n" + "="*50)
        print("🔧 机械臂坐标轴测试程序")
        print("📋 请仔细观察每个指令对应的机械臂运动方向")
        print("📝 记录下实际的运动方向，用于确定正确映射")
        print("="*50)
        
        for command, description in tests:
            node.test_direction(command, description)
        
        print("\n" + "="*50)
        print("✅ 测试完成！")
        print("📊 请根据观察结果确定SpaceMouse和机械臂的坐标对应关系")
        print("="*50)
        
    except KeyboardInterrupt:
        print('\n程序已停止')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
