#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from auto_navigation.msg import NavigationStatus

class StatusMonitor:
    def __init__(self):
        rospy.init_node('status_monitor', anonymous=True)
        
        # 订阅状态话题
        self.status_sub = rospy.Subscriber('/auto_navigation/status', NavigationStatus, self.status_callback)
        
        # 发布控制话题
        self.cancel_pub = rospy.Publisher('/auto_navigation/cancel', Bool, queue_size=1)
        self.pause_pub = rospy.Publisher('/auto_navigation/pause', Bool, queue_size=1)
        
        # 状态变量
        self.current_status = None
        self.is_paused = False
        
        rospy.loginfo("状态监控器已启动")
    
    def status_callback(self, msg):
        """处理状态消息"""
        self.current_status = msg
        
        # 打印状态信息
        print(f"\n=== 导航状态 ===")
        print(f"状态: {msg.status}")
        print(f"当前目标: {msg.current_goal}")
        print(f"进度: {msg.progress:.1f}%")
        print(f"距离目标: {msg.distance_to_goal:.2f}m")
        print(f"预计时间: {msg.estimated_time:.1f}s")
        print(f"正在执行: {'是' if msg.is_executing else '否'}")
        
        if msg.error_message:
            print(f"错误信息: {msg.error_message}")
        
        # 根据状态执行相应操作
        if msg.status == "failed":
            print("⚠️  导航失败！")
        elif msg.status == "reached":
            print("✅ 到达目标！")
        elif msg.status == "cancelled":
            print("❌ 导航已取消")
    
    def cancel_navigation(self):
        """取消导航"""
        cancel_msg = Bool()
        cancel_msg.data = True
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("已发送取消导航命令")
    
    def pause_navigation(self):
        """暂停导航"""
        pause_msg = Bool()
        pause_msg.data = True
        self.pause_pub.publish(pause_msg)
        self.is_paused = True
        rospy.loginfo("已发送暂停导航命令")
    
    def resume_navigation(self):
        """恢复导航"""
        pause_msg = Bool()
        pause_msg.data = False
        self.pause_pub.publish(pause_msg)
        self.is_paused = False
        rospy.loginfo("已发送恢复导航命令")
    
    def run(self):
        """主循环"""
        rospy.loginfo("状态监控器已启动")
        rospy.loginfo("输入命令:")
        rospy.loginfo("1. 'cancel' - 取消导航")
        rospy.loginfo("2. 'pause' - 暂停导航")
        rospy.loginfo("3. 'resume' - 恢复导航")
        rospy.loginfo("4. 'status' - 显示当前状态")
        rospy.loginfo("5. 'quit' - 退出")
        
        while not rospy.is_shutdown():
            try:
                command = input("> ").strip().lower()
                
                if command == "cancel":
                    self.cancel_navigation()
                elif command == "pause":
                    self.pause_navigation()
                elif command == "resume":
                    self.resume_navigation()
                elif command == "status":
                    if self.current_status:
                        print(f"当前状态: {self.current_status.status}")
                        print(f"当前目标: {self.current_status.current_goal}")
                        print(f"进度: {self.current_status.progress:.1f}%")
                    else:
                        print("暂无状态信息")
                elif command == "quit":
                    break
                else:
                    print("未知命令")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"错误: {e}")

if __name__ == '__main__':
    try:
        monitor = StatusMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass 