#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import sys
import os

# 添加夹爪SDK路径
sys.path.append('/home/ubuntu/clip_junduo')
from jodellSdk.jodellSdkDemo import RgClawControl

class GripperButtonControl(Node):
    def __init__(self):
        super().__init__('gripper_button_control')
        
        # 初始化夹爪控制
        self.init_gripper()
        
        # 订阅SpaceMouse按键
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/spacenav/joy',
            self.joy_callback,
            10
        )
        
        # 按键状态跟踪
        self.last_buttons = None
        self.gripper_state = None  # None=未知, True=开, False=合
        
        self.get_logger().info('=== SpaceMouse夹爪控制节点已启动 ===')
        print("🎮 控制说明:")
        print("  按键1 (左键)  → 夹爪开")
        print("  按键2 (右键)  → 夹爪合")
        print("=" * 50)
        
    def init_gripper(self):
        """初始化夹爪连接"""
        try:
            self.claw_control = RgClawControl()
            
            # 搜索串口并建立连接
            com_list = self.claw_control.searchCom()
            if not com_list:
                self.get_logger().error("❌ 未找到夹爪设备")
                raise Exception("未找到夹爪设备")
                
            flag = self.claw_control.serialOperation(com_list[0], 115200, 1)
            if flag:
                self.get_logger().info(f"✅ 夹爪连接成功: {com_list[0]}")
            else:
                self.get_logger().error("❌ 夹爪连接失败")
                raise Exception("夹爪连接失败")
                
        except Exception as e:
            self.get_logger().error(f"夹爪初始化失败: {e}")
            sys.exit(1)
    
    def joy_callback(self, msg):
        """处理SpaceMouse按键输入"""
        if len(msg.buttons) == 0:
            return
            
        # 创建固定长度的按键数组
        current_buttons = [0, 0]
        if len(msg.buttons) >= 1:
            current_buttons[0] = msg.buttons[0]
        if len(msg.buttons) >= 2:
            current_buttons[1] = msg.buttons[1]
        
        # 只在按键状态改变时处理
        if self.last_buttons != current_buttons:
            # 检测按键按下（从0变为1）
            if self.last_buttons is not None:
                # 按键1按下 - 夹爪开
                if current_buttons[0] and not self.last_buttons[0]:
                    self.gripper_open()
                    
                # 按键2按下 - 夹爪合
                if current_buttons[1] and not self.last_buttons[1]:
                    self.gripper_close()
            
            self.last_buttons = current_buttons
    
    def gripper_open(self):
        """夹爪开"""
        try:
            timestamp = time.strftime("%H:%M:%S")
            print(f"[{timestamp}] 🖐️  执行夹爪开...")
            
            # 使用无参模式：1表示夹爪开
            flag = self.claw_control.runWithoutParam(9, 1)
            
            if flag:
                self.gripper_state = True
                self.get_logger().info("✅ 夹爪开指令发送成功")
                print("  → 夹爪正在打开...")
            else:
                self.get_logger().warn("❌ 夹爪开指令发送失败")
                
        except Exception as e:
            self.get_logger().error(f"夹爪开操作异常: {e}")
    
    def gripper_close(self):
        """夹爪合"""
        try:
            timestamp = time.strftime("%H:%M:%S")
            print(f"[{timestamp}] ✊ 执行夹爪合...")
            
            # 使用无参模式：2表示夹爪合
            flag = self.claw_control.runWithoutParam(9, 2)
            
            if flag:
                self.gripper_state = False
                self.get_logger().info("✅ 夹爪合指令发送成功")
                print("  → 夹爪正在闭合...")
            else:
                self.get_logger().warn("❌ 夹爪合指令发送失败")
                
        except Exception as e:
            self.get_logger().error(f"夹爪合操作异常: {e}")
    
    def get_gripper_status(self):
        """获取夹爪状态（可选功能）"""
        try:
            pos = self.claw_control.getClampCurrentLocation(9)
            speed = self.claw_control.getClampCurrentSpeed(9)
            torque = self.claw_control.getClampCurrentTorque(9)
            status = self.claw_control.getClampCurrentState(9)
            return pos, speed, torque, status
        except Exception as e:
            self.get_logger().warn(f"获取夹爪状态失败: {e}")
            return None, None, None, None

def main():
    rclpy.init()
    
    try:
        node = GripperButtonControl()
        
        print("\n🚀 开始监听SpaceMouse按键...")
        print("💡 提示：按住按键不会重复执行，只在按下瞬间触发")
        print("\n按 Ctrl+C 退出程序\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n👋 收到停止信号，退出程序...')
    except Exception as e:
        print(f'\n❌ 程序异常: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
