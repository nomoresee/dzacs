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
        
        # 参数配置
        self.auto_execute = rospy.get_param('~auto_execute', True)
        self.goal_timeout = rospy.get_param('~goal_timeout', 30.0)
        self.max_retries = rospy.get_param('~max_retries', 3)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)
        self.planning_timeout = rospy.get_param('~planning_timeout', 10.0)
        self.max_queue_size = rospy.get_param('~max_queue_size', 20)
        
        # 状态变量
        self.current_goal = None
        self.goal_queue = []
        self.is_navigating = False
        self.retry_count = 0
        self.current_status = "idle"
        self.last_planning_time = rospy.Time.now()
        self._goal_seq = 0
        self._active_goal_seq = None
        self._timeout_timer = None
        self._handling_result = False
        self._cancel_requested = False
        self._pending_next_timer = None
        
        # 订阅话题
        self.goal_sub = rospy.Subscriber('/auto_navigation/goal', NavigationGoal, self.goal_callback, queue_size=self.max_queue_size)
        self.cancel_sub = rospy.Subscriber('/auto_navigation/cancel', Bool, self.cancel_callback)
        self.pause_sub = rospy.Subscriber('/auto_navigation/pause', Bool, self.pause_callback)
        
        # 目标检测话题订阅
        self.target_sub = rospy.Subscriber('/target_detected', Bool, self.target_callback)
        
        # 返航基地坐标
        self.base_x = rospy.get_param('~base_x', 0)
        self.base_y = rospy.get_param('~base_y', 0)
        self.base_yaw = rospy.get_param('~base_yaw', 0)
        
        # 目标检测状态
        self.target_found = False
        self.interrupted_goal = None
        
        # 发布话题
        self.status_pub = rospy.Publisher('/auto_navigation/status', NavigationStatus, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=self.max_queue_size)
        
        # Action客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("等待move_base服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务器已连接")
        
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
        self.planning_timer = rospy.Timer(rospy.Duration(2.0), self.check_planning_timeout)
        
        rospy.loginfo("自动导航节点已启动")

    def _cancel_timeout_timer(self):
        if self._timeout_timer is not None:
            self._timeout_timer.shutdown()
            self._timeout_timer = None

    def _cancel_pending_next_timer(self):
        if self._pending_next_timer is not None:
            self._pending_next_timer.shutdown()
            self._pending_next_timer = None

    def _is_active_seq(self, seq):
        return seq is not None and seq == self._active_goal_seq

    def _request_cancel(self, reason):
        """同一目标只取消一次，避免 SimpleActionClient received DONE twice"""
        if not self.is_navigating or self._cancel_requested:
            return False
        self._cancel_requested = True
        rospy.logwarn("%s", reason)
        # 只取消当前 goal，不要反复 cancel_all_goals
        try:
            self.move_base_client.cancel_goal()
        except Exception as ex:
            rospy.logwarn("cancel_goal 异常，回退 cancel_all_goals: %s", ex)
            self.move_base_client.cancel_all_goals()
        return True

    def _schedule_execute_next(self, delay=0.3):
        """延后发起下一个目标，避免在 DONE 回调栈内 send_goal"""
        self._cancel_pending_next_timer()
        self._pending_next_timer = rospy.Timer(
            rospy.Duration(delay),
            self._on_pending_next,
            oneshot=True,
        )

    def _on_pending_next(self, _event):
        self._pending_next_timer = None
        if not self.is_navigating:
            self.execute_next_goal()    
    def goal_callback(self, msg):
        """处理导航目标（支持整路径一次性入队）"""
        rospy.loginfo(f"收到导航目标: {msg.description}")

        if len(self.goal_queue) >= self.max_queue_size:
            rospy.logwarn(
                "目标队列已满(%d)，丢弃: %s",
                self.max_queue_size,
                msg.description,
            )
            return

        self.goal_queue.append(msg)
        self.goal_queue.sort(key=lambda x: x.priority, reverse=True)

        rospy.loginfo("当前队列长度: %d/%d", len(self.goal_queue), self.max_queue_size)

        if self.auto_execute and not self.is_navigating:
            self.execute_next_goal()
    
    def cancel_callback(self, msg):
        if msg.data:
            rospy.loginfo("取消导航")
            self.cancel_navigation()
    
    def pause_callback(self, msg):
        if msg.data:
            rospy.loginfo("暂停导航")
            self.pause_navigation()
        else:
            rospy.loginfo("恢复导航")
            self.resume_navigation()
    
    def target_callback(self, msg):
        if msg.data and not self.target_found:
            self.target_found = True
            rospy.logwarn("检测到目标，准备返航")
            
            if self.current_goal:
                self.interrupted_goal = self.current_goal
                rospy.loginfo(f"保存被中断的目标: {self.current_goal.description}")
            
            self.cancel_navigation()
            # 等取消完成后再发返航，避免与旧目标 DONE 打架
            rospy.Timer(
                rospy.Duration(0.3),
                lambda _e: self.return_to_base(),
                oneshot=True,
            )
    
    def execute_next_goal(self):
        """执行下一个目标"""
        if self.target_found:
            rospy.loginfo("目标检测中断中，等待返航完成")
            return

        if self.is_navigating:
            return
        
        if not self.goal_queue:
            rospy.loginfo("目标队列为空")
            return
        
        next_goal = self.goal_queue.pop(0)
        # 仅切换到新目标时清零重试计数，同一目标重试时保留
        if (
            self.current_goal is None
            or next_goal.description != self.current_goal.description
        ):
            self.retry_count = 0
        self.current_goal = next_goal
        self.is_navigating = True
        self.current_status = "navigating"
        self.last_planning_time = rospy.Time.now()
        self._handling_result = False
        self._cancel_requested = False
        
        rospy.loginfo(f"开始导航到: {self.current_goal.description}")
        
        goal = MoveBaseGoal()
        goal.target_pose = self.current_goal.goal_pose

        # 取消上一个超时定时器，并为本次目标分配序号，忽略过期 DONE
        self._cancel_timeout_timer()
        self._cancel_pending_next_timer()
        self._goal_seq += 1
        self._active_goal_seq = self._goal_seq
        seq = self._active_goal_seq

        timeout = self.current_goal.timeout if self.current_goal.timeout > 0 else self.goal_timeout

        self.move_base_client.send_goal(
            goal,
            done_cb=lambda status, result, s=seq: self.done_callback(status, result, s),
            active_cb=self.active_callback,
            feedback_cb=self.feedback_callback,
        )
        self.publish_status_now()

        self._timeout_timer = rospy.Timer(
            rospy.Duration(timeout),
            lambda event, s=seq: self.timeout_callback(event, s),
            oneshot=True,
        )
    
    def check_planning_timeout(self, event):
        if self.target_found or self._cancel_requested or self._handling_result:
            return
        
        if self.is_navigating and (rospy.Time.now() - self.last_planning_time).to_sec() > self.planning_timeout:
            # 同一目标只取消一次；DONE 到来前不要再次 cancel
            self._request_cancel("规划超时，取消当前目标（等待 DONE 后重试）")
    
    def done_callback(self, status, result, seq=None):
        """导航完成回调（带序号，防止旧目标/重复 DONE）"""
        if not self._is_active_seq(seq):
            rospy.logdebug("忽略过期 DONE 回调 seq=%s active=%s", seq, self._active_goal_seq)
            return

        if self._handling_result:
            rospy.logwarn("忽略重复 DONE 回调")
            return
        self._handling_result = True
        self._cancel_requested = False

        self._cancel_timeout_timer()
        try:
            self.move_base_client.stop_tracking_goal()
        except Exception:
            pass

        if self.target_found:
            rospy.loginfo("导航被目标检测中断，等待返航完成")
            self.is_navigating = False
            return
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"导航成功到达: {self.current_goal.description}")
            self.current_status = "reached"
            self.is_navigating = False
            self.retry_count = 0
            self.publish_status_now()

            if self.goal_queue:
                self._schedule_execute_next(0.5)
            else:
                self.current_status = "idle"
                self._active_goal_seq = None
                self.publish_status_now()
        else:
            rospy.logwarn(
                "导航未成功: %s (status=%d)",
                self.current_goal.description if self.current_goal else "?",
                status,
            )
            self.current_status = "failed"
            self.is_navigating = False
            self.publish_status_now()
            self.handle_navigation_failure()
    
    def active_callback(self):
        rospy.loginfo("导航已激活")
        self.last_planning_time = rospy.Time.now()
    
    def feedback_callback(self, feedback):
        self.last_planning_time = rospy.Time.now()
        
        if hasattr(feedback, 'base_position') and self.current_goal:
            current_pos = feedback.base_position.pose.position
            goal_pos = self.current_goal.goal_pose.pose.position
            
            distance = np.sqrt((current_pos.x - goal_pos.x)**2 + (current_pos.y - goal_pos.y)**2)
            
            if distance < self.goal_tolerance:
                rospy.loginfo(f"接近目标，距离: {distance:.2f}m")
    
    def timeout_callback(self, event, seq=None):
        """超时只取消一次，重试交给 done_callback"""
        if not self._is_active_seq(seq):
            return
        if self.target_found:
            return
        self.current_status = "failed"
        self._request_cancel(
            "导航超时，取消目标: %s"
            % (self.current_goal.description if self.current_goal else "?")
        )
    
    def handle_navigation_failure(self):
        if self.target_found:
            rospy.loginfo("导航失败被目标检测中断，等待返航完成")
            return
        
        self.retry_count += 1
        self.is_navigating = False
        
        if self.retry_count < self.max_retries:
            rospy.loginfo(f"重试导航 ({self.retry_count}/{self.max_retries})")
            if self.current_goal:
                self.goal_queue.insert(0, self.current_goal)
            self._schedule_execute_next(1.0)
        else:
            rospy.logerr(
                "导航失败，已达到最大重试次数: %s",
                self.current_goal.description if self.current_goal else "?",
            )
            self.retry_count = 0
            self._active_goal_seq = None
            
            if self.goal_queue:
                rospy.loginfo("跳过失败目标，继续下一个")
                self._schedule_execute_next(1.0)
            else:
                self.current_status = "idle"
                self.publish_status_now()
    
    def cancel_navigation(self):
        if self.is_navigating:
            self._cancel_timeout_timer()
            self._cancel_pending_next_timer()
            self._active_goal_seq = None
            self._cancel_requested = True
            try:
                self.move_base_client.cancel_goal()
            except Exception:
                self.move_base_client.cancel_all_goals()
            self.current_status = "cancelled"
            self.is_navigating = False
            self._handling_result = False
            rospy.loginfo("导航已取消")
    
    def pause_navigation(self):
        if self.is_navigating:
            self._cancel_timeout_timer()
            self._cancel_pending_next_timer()
            self._active_goal_seq = None
            self._cancel_requested = True
            try:
                self.move_base_client.cancel_goal()
            except Exception:
                self.move_base_client.cancel_all_goals()
            self.is_navigating = False
            rospy.loginfo("导航已暂停")
    
    def resume_navigation(self):
        if not self.is_navigating and self.current_goal:
            rospy.loginfo("恢复导航")
            self.goal_queue.insert(0, self.current_goal)
            self.execute_next_goal()
    
    def return_to_base(self):
        rospy.logwarn("开始返航到基地...")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.base_x
        goal.target_pose.pose.position.y = self.base_y
        
        import tf.transformations as tft
        q = tft.quaternion_from_euler(0, 0, self.base_yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self._cancel_timeout_timer()
        self._cancel_pending_next_timer()
        self._goal_seq += 1
        self._active_goal_seq = self._goal_seq
        seq = self._active_goal_seq
        self.is_navigating = True
        self._handling_result = False
        self._cancel_requested = False
        
        self.move_base_client.send_goal(
            goal,
            done_cb=lambda status, result, s=seq: self.return_done_callback(status, result, s),
            active_cb=self.active_callback,
            feedback_cb=self.feedback_callback,
        )

        self._timeout_timer = rospy.Timer(
            rospy.Duration(30.0),
            lambda event, s=seq: self.return_timeout_callback(event, s),
            oneshot=True,
        )
    
    def return_done_callback(self, status, result, seq=None):
        if not self._is_active_seq(seq):
            return
        if self._handling_result:
            return
        self._handling_result = True
        self._cancel_requested = False
        self._cancel_timeout_timer()
        self.is_navigating = False
        try:
            self.move_base_client.stop_tracking_goal()
        except Exception:
            pass

        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("成功返航到基地")
            self.target_found = False
            
            if self.interrupted_goal:
                rospy.loginfo(f"重新规划被中断的路径: {self.interrupted_goal.description}")
                self.goal_queue.insert(0, self.interrupted_goal)
                self.interrupted_goal = None
                self._schedule_execute_next(1.0)
            else:
                rospy.loginfo("没有中断的目标，继续执行队列中的下一个目标")
                if self.goal_queue:
                    self._schedule_execute_next(1.0)
        else:
            rospy.logerr("返航失败")
            self.target_found = False
            self.interrupted_goal = None
            self._active_goal_seq = None
    
    def return_timeout_callback(self, event, seq=None):
        if not self._is_active_seq(seq):
            return
        self._request_cancel("返航超时，取消返航目标")
    
    def publish_status(self, event):
        self.publish_status_now()

    def publish_status_now(self):
        status_msg = NavigationStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.status = self.current_status
        status_msg.is_executing = self.is_navigating
        
        if self.current_goal:
            status_msg.current_goal = self.current_goal.description
            status_msg.progress = 50.0
            status_msg.distance_to_goal = 0.0
            status_msg.estimated_time = 0.0
        
        self.status_pub.publish(status_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AutoNavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
