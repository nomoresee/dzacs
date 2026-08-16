#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String, Bool, Int32, UInt8, UInt8MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from auto_navigation.msg import NavigationGoal, NavigationStatus


class AutoNavigationNode:
    # 裁判系统物资点编号对应的地图坐标，格式为：点号: (x, y, yaw)。
    # x、y 的单位为米，yaw 的单位为弧度；当前到达判断只比较 x、y。
    # B1/B2、B15/B16 分别共用同一物理位置，但仍保留各自的裁判点号。
    # B8/B9 为彼此独立的可选物资点，可同时出现，不互斥。
    # B11/B12：仅在裁判物资包含对应点时进入路径并参与命名；
    # 若都不包含，路径仍经过 B12，但不把 12 写入路径文件名。
    MATERIAL_POINTS = {
        1: (1.20, 2.21, 0.00),
        2: (1.20, 2.21, 0.00),
        3: (3.19, 2.31, -0.13),
        4: (3.43, 1.67, -1.40),
        5: (3.67, 0.48, -1.54),
        6: (2.98, -0.03, -3.14),
        7: (3.45, -0.12, -2.35),
        8: (2.59, -0.02, -3.11),
        9: (2.57, -0.18, 3.13),
        10: (1.74, -0.13, 2.93),
        11: (2.10, 0.57, 0.03),
        12: (1.65, 0.65, 0.09),
        13: (2.67, 0.82, 1.35),
        14: (2.53, 1.40, 3.04),
        15: (1.32, 1.38, -3.09),
        16: (1.32, 1.38, -3.09),
    }

    def __init__(self):
        rospy.init_node('auto_navigation_node', anonymous=True)

        # 参数配置 - 优化超时和重试参数
        self.auto_execute = rospy.get_param('~auto_execute', True)
        self.goal_timeout = rospy.get_param('~goal_timeout', 20.0)
        self.max_retries = rospy.get_param('~max_retries', 2)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        self.planning_timeout = rospy.get_param('~planning_timeout', 10.0)
        # 裁判系统发布全部物资点编号的话题，消息类型为 UInt8MultiArray。
        self.referee_material_topic = rospy.get_param(
            '~referee_material_topic', '/all_Material_Number')
        # 确认到达后向云台发布点号的话题，消息类型为 UInt8。
        self.arrived_material_topic = rospy.get_param(
            '~arrived_material_topic', '/arrived_material_number')
        # 当前位置与物资点的平面距离不超过该值时判定到达，单位为米。
        self.material_arrive_tolerance = rospy.get_param(
            '~material_arrive_tolerance', 0.25)
        self.material_wait_duration = rospy.get_param(
            '~material_wait_duration', 3.0)
        self.switch_model_on_route_complete = rospy.get_param(
            '~switch_model_on_route_complete', True)
        self.model_switch_topic = rospy.get_param(
            '~model_switch_topic', '/switch_model')
        self.route_complete_model = rospy.get_param(
            '~route_complete_model', 'light_det2')

        # 状态变量
        self.current_goal = None
        self.goal_queue = []
        self.is_navigating = False
        self.retry_count = 0
        # 记录正在重试的目标，避免 execute_next_goal() 把累计重试次数重新清零。
        self.retry_goal = None
        self.current_status = "idle"
        self.last_planning_time = rospy.Time.now()
        # SimpleActionClient 只能可靠地跟踪一个目标。每次目标切换时递增代次，
        # 让旧目标的回调和旧定时器失效，避免它们干扰新目标的状态机。
        self.goal_generation = 0
        self.goal_timeout_timer = None
        self.next_goal_timer = None
        # 保存裁判系统物资点号（升序）；仅首次有效接收时写入并打印。
        self.referee_material_numbers = []
        # 保存 /amcl_pose 最近一次发布的机器人实时位姿。
        self.current_pose = None

        # 订阅话题
        self.goal_sub = rospy.Subscriber(
            '/auto_navigation/goal', NavigationGoal, self.goal_callback)
        self.cancel_sub = rospy.Subscriber(
            '/auto_navigation/cancel', Bool, self.cancel_callback)
        self.pause_sub = rospy.Subscriber(
            '/auto_navigation/pause', Bool, self.pause_callback)
        # 接收裁判系统物资点，例如 data: [1, 3, 7, 15]。
        self.material_sub = rospy.Subscriber(
            self.referee_material_topic, UInt8MultiArray, self.material_numbers_callback)
        # 接收机器人在 map 坐标系中的实时定位。
        self.amcl_pose_sub = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        # 目标检测话题订阅
        self.target_sub = rospy.Subscriber(
            '/target_detected', Bool, self.target_callback)

        # 返航基地坐标
        self.base_x = rospy.get_param('~base_x', 0)
        self.base_y = rospy.get_param('~base_y', 0)
        self.base_yaw = rospy.get_param('~base_yaw', 0)

        # 目标检测状态
        self.target_found = False
        self.interrupted_goal = None

        # 发布话题
        self.status_pub = rospy.Publisher(
            '/auto_navigation/status', NavigationStatus, queue_size=10)
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        # 导航成功且坐标匹配时，将具体的 B 点编号发布给云台。
        self.arrived_material_pub = rospy.Publisher(
            self.arrived_material_topic, UInt8, queue_size=10)
        self.model_switch_pub = rospy.Publisher(
            self.model_switch_topic, String, queue_size=1)

        # Action客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # 等待move_base服务器
        rospy.loginfo("等待move_base服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务器已连接")

        if self.switch_model_on_route_complete:
            rospy.loginfo(
                "最后一个导航点到达后将发布模型切换: %s -> %s",
                self.model_switch_topic,
                self.route_complete_model)

        # 定时器
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
        self.planning_timer = rospy.Timer(rospy.Duration(2.0), self.check_planning_timeout)

        rospy.loginfo("自动导航节点已启动")

    def goal_callback(self, msg):
        """处理导航目标"""
        rospy.loginfo("收到导航目标: %s", msg.description)

        self.goal_queue.append(msg)
        self.goal_queue.sort(key=lambda x: x.priority, reverse=True)

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

            if self.current_goal:
                self.interrupted_goal = self.current_goal
                rospy.loginfo("保存被中断的目标: %s", self.current_goal.description)

            self.cancel_navigation()
            self.return_to_base()

    def execute_next_goal(self):
        """执行下一个目标"""
        if self.target_found:
            rospy.loginfo("目标检测中断中，等待返航完成")
            return

        if not self.goal_queue:
            rospy.loginfo("目标队列为空")
            return

        self.current_goal = self.goal_queue.pop(0)
        self.is_navigating = True
        self.current_status = "navigating"
        # 新目标从 0 开始计数；同一个目标重试时保留已有计数。
        if self.current_goal is not self.retry_goal:
            self.retry_count = 0
        self.retry_goal = None
        self.last_planning_time = rospy.Time.now()

        rospy.loginfo("开始导航到: %s", self.current_goal.description)

        goal = MoveBaseGoal()
        goal.target_pose = self.current_goal.goal_pose

        self.send_move_base_goal(
            goal, self.done_callback, self.current_goal.timeout, self.timeout_callback)

    def invalidate_current_goal(self, stop_tracking=False):
        """使当前目标的异步回调失效，并清理其超时定时器。"""
        self.goal_generation += 1
        if self.goal_timeout_timer is not None:
            self.goal_timeout_timer.shutdown()
            self.goal_timeout_timer = None
        if stop_tracking:
            self.move_base_client.stop_tracking_goal()

    def send_move_base_goal(self, goal, done_callback, timeout, timeout_callback):
        """发送带代次保护的目标，丢弃上一个目标迟到的回调。"""
        self.invalidate_current_goal(stop_tracking=True)
        generation = self.goal_generation

        def done(status, result):
            if generation != self.goal_generation:
                return
            if self.goal_timeout_timer is not None:
                self.goal_timeout_timer.shutdown()
                self.goal_timeout_timer = None
            done_callback(status, result)

        def active():
            if generation == self.goal_generation:
                self.active_callback()

        def feedback(msg):
            if generation == self.goal_generation:
                self.feedback_callback(msg)

        def timed_out(event):
            if generation == self.goal_generation:
                timeout_callback(event)

        self.move_base_client.send_goal(goal, done, active, feedback)
        self.goal_timeout_timer = rospy.Timer(
            rospy.Duration(timeout), timed_out, oneshot=True)

    def schedule_next_goal(self, delay=0.2):
        """退出 actionlib 回调栈后再发送下一目标。"""
        if self.next_goal_timer is not None:
            self.next_goal_timer.shutdown()
        self.next_goal_timer = rospy.Timer(
            rospy.Duration(delay), lambda event: self.execute_next_goal(), oneshot=True)

    def check_planning_timeout(self, event):
        """检查规划超时"""
        if self.target_found:
            return

        if self.is_navigating and (
                rospy.Time.now() - self.last_planning_time).to_sec() > self.planning_timeout:
            rospy.logwarn("规划超时，尝试重新规划")
            self.handle_planning_timeout()

    def handle_planning_timeout(self):
        """处理规划超时"""
        if self.target_found:
            rospy.loginfo("规划超时被目标检测中断，等待返航完成")
            return

        if self.is_navigating:
            rospy.logwarn("规划超时，取消当前目标并重试")
            self.move_base_client.cancel_goal()
            self.invalidate_current_goal(stop_tracking=True)
            self.handle_navigation_failure()

    def done_callback(self, status, result):
        """导航完成回调"""
        if self.target_found:
            rospy.loginfo("导航被目标检测中断，等待返航完成")
            return

        if status == actionlib.GoalStatus.SUCCEEDED:
            completed_goal = self.current_goal
            rospy.loginfo("导航成功到达: %s", completed_goal.description)
            self.current_status = "reached"
            self.is_navigating = False
            self.retry_count = 0
            arrived_material = self.publish_arrived_material_if_needed()

            # goal_publisher 将整条路线的最后一个目标标记为 route_end。
            # 在 action 成功回调中直接发布，避免周期状态话题漏掉 reached。
            if completed_goal.goal_type == "route_end":
                if self.goal_queue:
                    rospy.logwarn(
                        "已到达 route_end，但队列中仍有 %d 个目标，暂不切换模型",
                        len(self.goal_queue))
                else:
                    self.publish_route_complete_model_switch()

            if self.goal_queue:
                self.invalidate_current_goal(stop_tracking=True)
                next_goal_delay = (
                    self.material_wait_duration if arrived_material else 0.2
                )
                if arrived_material:
                    rospy.loginfo(
                        "已向云台发布物资点，等待%.1f秒后执行下一目标",
                        self.material_wait_duration)
                self.schedule_next_goal(next_goal_delay)
        else:
            rospy.logwarn("导航失败: %s", self.current_goal.description)
            self.current_status = "failed"
            self.handle_navigation_failure()

    def publish_route_complete_model_switch(self):
        """最后一个路线点到达后，向检测节点发送一次模型切换请求。"""
        if not self.switch_model_on_route_complete:
            rospy.loginfo("路线已完成，但自动切换模型已关闭")
            return

        model_name = str(self.route_complete_model).strip()
        if not model_name:
            rospy.logerr("route_complete_model 为空，未发布模型切换请求")
            return

        self.model_switch_pub.publish(String(data=model_name))
        rospy.loginfo(
            "最后一个导航点已到达，已发布模型切换请求: %s -> %s",
            self.model_switch_topic,
            model_name)

    def material_numbers_callback(self, msg):
        """接收裁判系统物资点；仅首次有效数据时打印一次排序结果。"""
        if self.referee_material_numbers:
            return

        raw = [
            int(number) for number in msg.data if int(number) in self.MATERIAL_POINTS
        ]
        if len(raw) < 4:
            return

        self.referee_material_numbers = sorted(raw[:4])
        rospy.loginfo("裁判系统物资点: %s", self.referee_material_numbers)

    def amcl_pose_callback(self, msg):
        """保存机器人在 map 坐标系中的最新实时位姿。"""
        self.current_pose = msg.pose.pose

    def find_arrived_material_number(self):
        """
        判断刚到达的路径点是否对应裁判系统要求识别的物资点。

        用当前导航目标的 x、y（即路径点坐标）与裁判物资点坐标匹配，
        不依赖 yaw。B1/B2 或 B15/B16 坐标相同时，因列表已升序，取较小编号。
        """
        if self.current_goal is None:
            return None
        if not self.referee_material_numbers:
            rospy.logwarn("尚未收到裁判系统物资点，无法发布识别编号")
            return None

        goal_x = self.current_goal.goal_pose.pose.position.x
        goal_y = self.current_goal.goal_pose.pose.position.y

        for number in self.referee_material_numbers:
            point_x, point_y, _ = self.MATERIAL_POINTS[number]
            distance = np.hypot(goal_x - point_x, goal_y - point_y)
            if distance <= self.material_arrive_tolerance:
                rospy.loginfo(
                    "路径点匹配物资点 %d，目标=(%.2f, %.2f)，距离=%.3fm",
                    number, goal_x, goal_y, distance)
                return number

        return None

    def publish_arrived_material_if_needed(self):
        """
        到达路径点后，若该点对应裁判下发的需识别物资点，
        则在 /arrived_material_number 发布物资编号（UInt8，1～16）供云台识别。
        普通路径点（A 点或未在裁判列表中的 B 点）不发布。
        """
        number = self.find_arrived_material_number()
        if number is None:
            rospy.loginfo("当前路径点无需识别物资，不发布 /arrived_material_number")
            return

        self.arrived_material_pub.publish(UInt8(data=number))
        rospy.loginfo(
            "已发布需识别物资点编号: %d -> %s",
            number, self.arrived_material_topic)
        return True


    def active_callback(self):
        """导航激活回调"""
        rospy.loginfo("导航已激活")
        self.last_planning_time = rospy.Time.now()

    def feedback_callback(self, feedback):
        """导航反馈回调"""
        self.last_planning_time = rospy.Time.now()

        if hasattr(feedback, 'base_position') and self.current_goal:
            current_pos = feedback.base_position.pose.position
            goal_pos = self.current_goal.goal_pose.pose.position
            distance = np.sqrt(
                (current_pos.x - goal_pos.x) ** 2 + (current_pos.y - goal_pos.y) ** 2)
            if distance < self.goal_tolerance:
                rospy.loginfo("接近目标，距离: %.2fm", distance)

    def timeout_callback(self, event):
        """超时回调"""
        if self.target_found:
            rospy.loginfo("导航超时被目标检测中断，等待返航完成")
            return

        if self.is_navigating:
            rospy.logwarn("导航超时: %s", self.current_goal.description)
            self.current_status = "failed"
            self.move_base_client.cancel_goal()
            self.invalidate_current_goal(stop_tracking=True)
            self.handle_navigation_failure()

    def handle_navigation_failure(self):
        """处理导航失败"""
        if self.target_found:
            rospy.loginfo("导航失败被目标检测中断，等待返航完成")
            return

        self.retry_count += 1

        if self.retry_count < self.max_retries:
            rospy.loginfo("重试导航 (%d/%d)", self.retry_count, self.max_retries)
            self.retry_goal = self.current_goal
            self.goal_queue.insert(0, self.current_goal)
            self.is_navigating = False
            self.schedule_next_goal(1.0)
        else:
            rospy.logerr(
                "导航失败，已达到最大重试次数: %s", self.current_goal.description)
            self.is_navigating = False
            self.retry_count = 0

            if self.goal_queue:
                rospy.loginfo("跳过失败目标，继续下一个")
                self.schedule_next_goal(1.0)

    def cancel_navigation(self):
        """取消导航"""
        if self.is_navigating:
            self.move_base_client.cancel_goal()
            self.invalidate_current_goal(stop_tracking=True)
            self.current_status = "cancelled"
            self.is_navigating = False
            rospy.loginfo("导航已取消")

    def pause_navigation(self):
        """暂停导航"""
        if self.is_navigating:
            self.move_base_client.cancel_goal()
            self.invalidate_current_goal(stop_tracking=True)
            rospy.loginfo("导航已暂停")

    def resume_navigation(self):
        """恢复导航"""
        if not self.is_navigating and self.current_goal:
            rospy.loginfo("恢复导航")
            self.execute_next_goal()

    def return_to_base(self):
        """返航到基地"""
        rospy.logwarn("开始返航到基地...")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.base_x
        goal.target_pose.pose.position.y = self.base_y

        import tf
        q = tf.transformations.quaternion_from_euler(0, 0, self.base_yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.send_move_base_goal(
            goal, self.return_done_callback, 30.0, self.return_timeout_callback)

    def return_done_callback(self, status, result):
        """返航完成回调"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("成功返航到基地")
            self.target_found = False

            if self.interrupted_goal:
                rospy.loginfo(
                    "重新规划被中断的路径: %s", self.interrupted_goal.description)
                self.goal_queue.insert(0, self.interrupted_goal)
                self.interrupted_goal = None

                if not self.is_navigating:
                    self.invalidate_current_goal(stop_tracking=True)
                    self.schedule_next_goal(1.0)
            else:
                rospy.loginfo("没有中断的目标，继续执行队列中的下一个目标")
                if not self.is_navigating and self.goal_queue:
                    self.invalidate_current_goal(stop_tracking=True)
                    self.schedule_next_goal(1.0)
        else:
            rospy.logerr("返航失败")
            self.target_found = False
            self.interrupted_goal = None

    def return_timeout_callback(self, event):
        """返航超时回调"""
        rospy.logerr("返航超时")
        self.move_base_client.cancel_goal()
        self.invalidate_current_goal(stop_tracking=True)
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
            status_msg.progress = 50.0
            status_msg.distance_to_goal = 0.0
            status_msg.estimated_time = 0.0

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
