#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String, Bool, Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from auto_navigation.msg import NavigationGoal, NavigationStatus

class AutoNavigationNode:
    def __init__(self):
        rospy.init_node('auto_navigation_node', anonymous=True)
        
        # 参数配置 - 优化超时和重试参数
        self.auto_execute = rospy.get_param('~auto_execute', True)
        self.goal_timeout = rospy.get_param('~goal_timeout', 20.0)  # 减少超时时间
        self.max_retries = rospy.get_param('~max_retries', 2)       # 减少重试次数
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)  # 增大容差
        self.planning_timeout = rospy.get_param('~planning_timeout', 10.0)  # 规划超时
        
        # 状态变量
        self.current_goal = None
        self.goal_queue = []
        self.is_navigating = False
        self.retry_count = 0
        self.current_status = "idle"
        self.last_planning_time = rospy.Time.now()
        
        # 订阅话题
        self.goal_sub = rospy.Subscriber('/auto_navigation/goal', NavigationGoal, self.goal_callback)
        self.cancel_sub = rospy.Subscriber('/auto_navigation/cancel', Bool, self.cancel_callback)
        self.pause_sub = rospy.Subscriber('/auto_navigation/pause', Bool, self.pause_callback)
        
        # 目标检测话题订阅
        self.target_sub = rospy.Subscriber('/target_detected', Bool, self.target_callback)
        
        # 返航基地坐标
        self.base_x = rospy.get_param('~base_x', 1.07)
        self.base_y = rospy.get_param('~base_y', 3.48)
        self.base_yaw = rospy.get_param('~base_yaw', -1.57)
        
        # 目标检测状态
        self.target_found = False
        self.interrupted_goal = None  # 被中断的目标
        
        # 发布话题
        self.status_pub = rospy.Publisher('/auto_navigation/status', NavigationStatus, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Action客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # 等待move_base服务器
        rospy.loginfo("等待move_base服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务器已连接")
        
        # 定时器 - 更频繁的状态检查
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
        self.planning_timer = rospy.Timer(rospy.Duration(2.0), self.check_planning_timeout)
        
        rospy.loginfo("自动导航节点已启动")
    
    def goal_callback(self, msg):
        """处理导航目标"""
        rospy.loginfo(f"收到导航目标: {msg.description}")
        
        # 添加到目标队列
        self.goal_queue.append(msg)
        
        # 按优先级排序
        self.goal_queue.sort(key=lambda x: x.priority, reverse=True)
        
        # 如果自动执行且当前没有导航，开始导航
        if self.auto_execute and not self.is_navigating:
            self.execute_next_goal()
    
    def cancel_callback(self, msg):
        """取消导航"""
        if msg.data:
            rospy.loginfo("取消导航")
            self.cancel_navigation()
    
    def pause_callback(self, msg):
        """暂停/恢复导航"""
        if msg.data:
            rospy.loginfo("暂停导航")
            self.pause_navigation()
        else:
            rospy.loginfo("恢复导航")
            self.resume_navigation()
    
    def target_callback(self, msg):
        """目标检测回调"""
        if msg.data and not self.target_found:
            self.target_found = True
            rospy.logwarn("检测到目标，准备返航")
            
            # 保存当前目标
            if self.current_goal:
                self.interrupted_goal = self.current_goal
                rospy.loginfo(f"保存被中断的目标: {self.current_goal.description}")
            
            # 取消当前导航
            self.cancel_navigation()
            
            # 开始返航流程
            self.return_to_base()
    
    def execute_next_goal(self):
        """执行下一个目标"""
        # 检查是否被目标检测中断
        if self.target_found:
            rospy.loginfo("目标检测中断中，等待返航完成")
            return
        
        if not self.goal_queue:
            rospy.loginfo("目标队列为空")
            return
        
        self.current_goal = self.goal_queue.pop(0)
        self.is_navigating = True
        self.current_status = "navigating"
        self.retry_count = 0
        self.last_planning_time = rospy.Time.now()
        
        rospy.loginfo(f"开始导航到: {self.current_goal.description}")
        
        # 创建move_base目标
        goal = MoveBaseGoal()
        goal.target_pose = self.current_goal.goal_pose
        
        # 发送目标
        self.move_base_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        
        # 设置超时 - 使用更短的超时时间
        rospy.Timer(rospy.Duration(self.current_goal.timeout), self.timeout_callback, oneshot=True)
    
    def check_planning_timeout(self, event):
        """检查规划超时"""
        # 检查是否被目标检测中断
        if self.target_found:
            return
        
        if self.is_navigating and (rospy.Time.now() - self.last_planning_time).to_sec() > self.planning_timeout:
            rospy.logwarn("规划超时，尝试重新规划")
            self.handle_planning_timeout()
    
    def handle_planning_timeout(self):
        """处理规划超时"""
        # 检查是否被目标检测中断
        if self.target_found:
            rospy.loginfo("规划超时被目标检测中断，等待返航完成")
            return
        
        if self.is_navigating:
            rospy.logwarn("规划超时，取消当前目标并重试")
            self.move_base_client.cancel_all_goals()
            rospy.sleep(1.0)
            self.handle_navigation_failure()
    
    def done_callback(self, status, result):
        """导航完成回调"""
        # 检查是否被目标检测中断
        if self.target_found:
            rospy.loginfo("导航被目标检测中断，等待返航完成")
            return
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"导航成功到达: {self.current_goal.description}")
            self.current_status = "reached"
            self.is_navigating = False
            self.retry_count = 0
            
            # 执行下一个目标
            if self.goal_queue:
                rospy.sleep(0.5)  # 减少等待时间
                self.execute_next_goal()
        else:
            rospy.logwarn(f"导航失败: {self.current_goal.description}")
            self.current_status = "failed"
            self.handle_navigation_failure()
    
    def active_callback(self):
        """导航激活回调"""
        rospy.loginfo("导航已激活")
        self.last_planning_time = rospy.Time.now()
    
    def feedback_callback(self, feedback):
        """导航反馈回调"""
        # 更新规划时间
        self.last_planning_time = rospy.Time.now()
        
        # 检查是否接近目标
        if hasattr(feedback, 'base_position') and self.current_goal:
            current_pos = feedback.base_position.pose.position
            goal_pos = self.current_goal.goal_pose.pose.position
            
            distance = np.sqrt((current_pos.x - goal_pos.x)**2 + (current_pos.y - goal_pos.y)**2)
            
            if distance < self.goal_tolerance:
                rospy.loginfo(f"接近目标，距离: {distance:.2f}m")
    
    def timeout_callback(self, event):
        """超时回调"""
        # 检查是否被目标检测中断
        if self.target_found:
            rospy.loginfo("导航超时被目标检测中断，等待返航完成")
            return
        
        if self.is_navigating:
            rospy.logwarn(f"导航超时: {self.current_goal.description}")
            self.current_status = "failed"
            self.handle_navigation_failure()
    
    def handle_navigation_failure(self):
        """处理导航失败"""
        # 检查是否被目标检测中断
        if self.target_found:
            rospy.loginfo("导航失败被目标检测中断，等待返航完成")
            return
        
        self.retry_count += 1
        
        if self.retry_count < self.max_retries:
            rospy.loginfo(f"重试导航 ({self.retry_count}/{self.max_retries})")
            # 重新添加到队列前端
            self.goal_queue.insert(0, self.current_goal)
            self.is_navigating = False
            rospy.sleep(1.0)  # 减少等待时间
            self.execute_next_goal()
        else:
            rospy.logerr(f"导航失败，已达到最大重试次数: {self.current_goal.description}")
            self.is_navigating = False
            self.retry_count = 0
            
            # 跳过这个目标，继续下一个
            if self.goal_queue:
                rospy.loginfo("跳过失败目标，继续下一个")
                rospy.sleep(1.0)
                self.execute_next_goal()
    
    def cancel_navigation(self):
        """取消导航"""
        if self.is_navigating:
            self.move_base_client.cancel_all_goals()
            self.current_status = "cancelled"
            self.is_navigating = False
            rospy.loginfo("导航已取消")
    
    def pause_navigation(self):
        """暂停导航"""
        if self.is_navigating:
            self.move_base_client.cancel_all_goals()
            rospy.loginfo("导航已暂停")
    
    def resume_navigation(self):
        """恢复导航"""
        if not self.is_navigating and self.current_goal:
            rospy.loginfo("恢复导航")
            self.execute_next_goal()
    
    def return_to_base(self):
        """返航到基地"""
        rospy.logwarn("开始返航到基地...")
        
        # 创建返航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.base_x
        goal.target_pose.pose.position.y = self.base_y
        
        # 设置朝向
        import tf2_geometry_msgs
        import tf2_ros
        q = tf2_ros.transformations.quaternion_from_euler(0, 0, self.base_yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        # 发送返航目标
        self.move_base_client.send_goal(goal, self.return_done_callback, self.active_callback, self.feedback_callback)
        
        # 设置返航超时
        rospy.Timer(rospy.Duration(30.0), self.return_timeout_callback, oneshot=True)
    
    def return_done_callback(self, status, result):
        """返航完成回调"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("成功返航到基地")
            
            # 重置目标检测状态
            self.target_found = False
            
            # 等待系统稳定
            rospy.sleep(1.0)
            
            # 重新规划被中断的路径
            if self.interrupted_goal:
                rospy.loginfo(f"重新规划被中断的路径: {self.interrupted_goal.description}")
                # 将中断的目标重新加入队列前端
                self.goal_queue.insert(0, self.interrupted_goal)
                self.interrupted_goal = None
                
                # 继续执行下一个目标
                if not self.is_navigating:
                    self.execute_next_goal()
            else:
                rospy.loginfo("没有中断的目标，继续执行队列中的下一个目标")
                if not self.is_navigating and self.goal_queue:
                    self.execute_next_goal()
        else:
            rospy.logerr("返航失败")
            self.target_found = False
            self.interrupted_goal = None
    
    def return_timeout_callback(self, event):
        """返航超时回调"""
        rospy.logerr("返航超时")
        self.target_found = False
        self.interrupted_goal = None
    
    def publish_status(self, event):
        """发布状态信息"""
        status_msg = NavigationStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.status = self.current_status
        status_msg.is_executing = self.is_navigating
        
        if self.current_goal:
            status_msg.current_goal = self.current_goal.description
            # 计算进度（简化版本）
            status_msg.progress = 50.0  # 这里可以根据实际情况计算
            status_msg.distance_to_goal = 0.0  # 这里可以根据实际情况计算
            status_msg.estimated_time = 0.0  # 这里可以根据实际情况计算
        
        self.status_pub.publish(status_msg)
    
    def run(self):
        """主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AutoNavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass