#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
import time

class ArmPositionTester(Node):
    def __init__(self):
        super().__init__('arm_position_tester')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            PoseStamped, 
            '/cart_goal', 
            10
        )
        
        # 定义不同的测试位置
        self.test_positions = [
            # 位置1: 前方中间
            {
                'position': Point(x=0.4, y=0.0, z=0.4),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                'name': '前方中间'
            },
            # 位置2: 左前方
            {
                'position': Point(x=0.4, y=0.2, z=0.4),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                'name': '左前方'
            },
            # 位置3: 右前方
            {
                'position': Point(x=0.4, y=-0.2, z=0.4),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                'name': '右前方'
            },
            # 位置4: 高位置
            {
                'position': Point(x=0.4, y=0.0, z=0.6),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                'name': '高位置'
            },
            # 位置5: 低位置
            {
                'position': Point(x=0.4, y=0.0, z=0.2),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                'name': '低位置'
            },
            # 位置6: 远位置
            {
                'position': Point(x=0.6, y=0.0, z=0.4),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                'name': '远位置'
            }
        ]
        
        self.current_position_index = 0
        
        # 创建定时器，每10秒执行一次
        self.timer = self.create_timer(3.0, self.send_next_position)
        
        self.get_logger().info('机械臂位置测试脚本已启动，每10秒发送一个新位置')
        
    def send_next_position(self):
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
            
            self.get_logger().info(f'发送位置 {self.current_position_index + 1}: {position_data["name"]} - x:{position_data["position"].x:.2f}, y:{position_data["position"].y:.2f}, z:{position_data["position"].z:.2f}')
            
            self.current_position_index += 1
            
            # 如果所有位置都发送完了，重新开始
            if self.current_position_index >= len(self.test_positions):
                self.get_logger().info('所有测试位置已发送完毕，重新开始循环')
                self.current_position_index = 0
        else:
            self.current_position_index = 0

def main():
    rclpy.init()
    
    node = ArmPositionTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('测试脚本被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()