#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

class ButtonMonitor(Node):
    def __init__(self):
        super().__init__('button_monitor')
        
        self.subscription = self.create_subscription(
            Joy,
            '/spacenav/joy',
            self.joy_callback,
            10
        )
        
        self.last_buttons = None
        self.get_logger().info('监听SpaceMouse按键... (按Ctrl+C退出)')
        print("=" * 50)
        print("SpaceMouse按键监控")
        print("=" * 50)
        
    def joy_callback(self, msg):
        # 处理不同长度的按键数组
        if len(msg.buttons) == 0:
            return  # 没有按键数据，跳过
            
        # 创建固定长度的按键数组，缺少的按键用0填充
        current_buttons = [0, 0]  # 初始化两个按键都为0
        
        # 根据实际接收到的按键数量填充, 这个逻辑挺不合常理的
        # 按键数组长度 = 0：跳过（没有按键数据）
        # 按键数组长度 = 1：显示 [按键值, 0]
        # 按键数组长度 = 2：显示 [按键1值, 按键2值]
        if len(msg.buttons) >= 1:
            current_buttons[0] = msg.buttons[0]
        if len(msg.buttons) >= 2:
            current_buttons[1] = msg.buttons[1]
        
        # 只在按键状态改变时显示
        if self.last_buttons != current_buttons:
            timestamp = time.strftime("%H:%M:%S")
            print(f"[{timestamp}] 按键1: {current_buttons[0]} | 按键2: {current_buttons[1]}")
            
            # 显示按键动作
            if self.last_buttons is not None:
                if current_buttons[0] and not self.last_buttons[0]:
                    print("  → 按键1 按下!")
                elif not current_buttons[0] and self.last_buttons[0]:
                    print("  → 按键1 松开!")
                    
                if current_buttons[1] and not self.last_buttons[1]:
                    print("  → 按键2 按下!")
                elif not current_buttons[1] and self.last_buttons[1]:
                    print("  → 按键2 松开!")
            
            self.last_buttons = current_buttons

def main():
    rclpy.init()
    
    try:
        node = ButtonMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n监控结束.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
