#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import time

class SimpleCartTest(Node):
    def __init__(self):
        super().__init__('simple_cart_test')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            PoseStamped, 
            '/cart_goal', 
            10
        )
        
        # 定义几个测试位置
        self.test_positions = [
            # 位置1: 中间位置
            {
                'position': Point(x=0.4, y=0.0, z=0.4),
                'orientation': Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
                'name': '中间位置'
            },
            # 位置2: 前方
            {
                'position': Point(x=0.5, y=0.0, z=0.4),
                'orientation': Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
                'name': '前方位置'
            },
            # 位置3: 左边
            {
                'position': Point(x=0.4, y=0.2, z=0.4),
                'orientation': Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
                'name': '左边位置'
            },
            # 位置4: 右边
            {
                'position': Point(x=0.4, y=-0.2, z=0.4),
                'orientation': Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
                'name': '右边位置'
            },
            # 位置5: 高位置
            {
                'position': Point(x=0.4, y=0.0, z=0.5),
                'orientation': Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
                'name': '高位置'
            }
        ]
        
        self.current_position_index = 0
        
        # 创建定时器，每5秒执行一次
        self.timer = self.create_timer(5.0, self.send_next_position)
        
        self.get_logger().info('简单笛卡尔位置测试脚本已启动，每5秒发送一个新位置')
        self.get_logger().info(f'总共有 {len(self.test_positions)} 个测试位置')
        
    def send_next_position(self):
        """发送下一个测试位置"""
        if self.current_position_index < len(self.test_positions):
            position_data = self.test_positions[self.current_position_index]
            
            # 创建消息
            msg = PoseStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "elfin_base_link"
            msg.pose.position = position_data['position']
            msg.pose.orientation = position_data['orientation']
            
            # 发布消息
            self.publisher.publish(msg)
            
            self.get_logger().info(
                f'[{self.current_position_index + 1}/{len(self.test_positions)}] '
                f'发送位置: {position_data["name"]} - '
                f'x:{position_data["position"].x:.2f}, '
                f'y:{position_data["position"].y:.2f}, '
                f'z:{position_data["position"].z:.2f}'
            )
            
            self.current_position_index += 1
            
        else:
            # 所有位置都发送完了，重新开始
            self.get_logger().info('=== 所有测试位置已发送完毕，重新开始循环 ===')
            self.current_position_index = 0

def main():
    rclpy.init()
    
    node = SimpleCartTest()
    
    try:
        # 发送第一个位置
        node.send_next_position()
        
        # 开始循环
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('测试脚本被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

