#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import math
import threading
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Int32, UInt8, UInt8MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from auto_navigation.msg import NavigationGoal, NavigationStatus


class AutoNavigationNode:
    # 裁判系统物资点编号对应的地图坐标，格式为：点号: (x, y, yaw)。
    # x、y 的单位为米，yaw 的单位为弧度；当前到达判断只比较 x、y。
    # B1/B2、B8/9、B15/B16 分别共用同一物理位置，但仍保留各自的裁判点号。
    # B11/B12：仅在裁判物资包含对应点时进入路径并参与命名；
    # 若都不包含，路径仍经过 B12，但不把 12 写入路径文件名。
    MATERIAL_POINTS = {
        1: (1.24, 2.09, 0),
        2: (1.24, 2.09, 0),
        3: (3.14, 2.20, -0.13),
        4: (3.43, 1.67, -1.40),
        5: (3.57, 0.48, -1.54),
        6: (2.89, -0.11, -3.14),
        7: (3.49, 0.22, -1.54),
        8: (2.63, -0.12, -3.12),
        9: (2.63, -0.12, -3.12),
        10: (1.90, -0.13, -3.09),
        11: (2.10, 0.65, 0.03),
        12: (1.65, 0.65, 0.09),
        13: (2.55, 0.80, 1.29),
        14: (2.66, 1.21, 3.04),
        15: (1.53, 1.4, -3.09),
        16: (1.53, 1.4, -3.09)
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
        # 导航期间的物资预扫描目标。该话题不表示“已经到达”，不会影响
        # 语音/识别的到达事件；data=1..16 选择扫描点，data=0 回中并锁住。
        self.material_scan_target_topic = rospy.get_param(
            '~material_scan_target_topic', '/material_scan_target')
        # 物资到达事件后的云台独立切换时间。它不影响底盘停留或下一目标发送。
        self.material_scan_switch_delay = rospy.get_param(
            '~material_scan_switch_delay', 3.0)
        self.switch_model_on_route_complete = rospy.get_param(
            '~switch_model_on_route_complete', True)
        self.model_switch_topic = rospy.get_param(
            '~model_switch_topic', '/switch_model')
        self.route_complete_model = rospy.get_param(
            '~route_complete_model', 'light_det')
        self.shooting_mode_topic = rospy.get_param(
            '~shooting_mode_topic', '/shooting_mode')
        self.current_model_topic = rospy.get_param(
            '~current_model_topic', '/current_model')
        self.model_switch_timeout = rospy.get_param(
            '~model_switch_timeout', 15.0)
        # A→B 直线过坡模式。只在路径实际经过指定 A、随后经过指定 B 时启用。
        self.slope_mode_enabled = rospy.get_param('~slope_mode_enabled', True)
        # 仅供现场短时直线过坡测试使用；默认必须关闭。
        self.slope_test_bypass_safety = rospy.get_param(
            '~slope_test_bypass_safety', False)
        self.slope_start = (
            rospy.get_param('~slope_start_x', 1.30),
            rospy.get_param('~slope_start_y', 2.07))
        self.slope_end = (
            rospy.get_param('~slope_end_x', 2.83),
            rospy.get_param('~slope_end_y', 2.05))
        self.slope_heading_yaw = rospy.get_param('~slope_heading_yaw', 0.01)
        self.slope_linear_speed = rospy.get_param('~slope_linear_speed', 0.5)
        self.slope_alignment_linear_speed = rospy.get_param(
            '~slope_alignment_linear_speed', 0.1)
        self.slope_goal_tolerance = rospy.get_param('~slope_goal_tolerance', 0.05)
        self.slope_route_match_tolerance = rospy.get_param(
            '~slope_route_match_tolerance', 0.015)
        # 只有到达 A 时定位足够新且车头已基本朝向坡道，才允许直线控制接管；
        # 否则保留后续队列，继续交给 move_base，避免阿克曼车大角度画弧。
        self.slope_takeover_max_pose_age = rospy.get_param(
            '~slope_takeover_max_pose_age', 1.0)
        self.slope_takeover_max_position_error = rospy.get_param(
            '~slope_takeover_max_position_error', 0.2)
        self.slope_takeover_max_heading_error = rospy.get_param(
            '~slope_takeover_max_heading_error', 0.5)
        self.slope_heading_tolerance = rospy.get_param(
            '~slope_heading_tolerance', 0.03)
        self.slope_heading_kp = rospy.get_param('~slope_heading_kp', 2.0)
        self.slope_cross_track_kp = rospy.get_param('~slope_cross_track_kp', 1.0)
        self.slope_max_angular_speed = rospy.get_param(
            '~slope_max_angular_speed', 1.0)
        # 限制 yaw_rate / linear_speed，避免低速时 dzactuator 将舵角钳到机械极限。
        self.slope_max_curvature = rospy.get_param('~slope_max_curvature', 2.0)
        # 阿克曼底盘不能原地转向。进入或恢复直线前使用低速前行配合转向
        # 做姿态微调；离开直线状态的阈值大于对齐完成阈值，避免抖动。
        self.slope_realign_tolerance = rospy.get_param(
            '~slope_realign_tolerance', 0.06)
        self.slope_alignment_max_angular_speed = rospy.get_param(
            '~slope_alignment_max_angular_speed', 0.25)
        self.slope_control_rate = rospy.get_param('~slope_control_rate', 20.0)
        self.slope_timeout = rospy.get_param('~slope_timeout', 15.0)
        self.slope_approach_distance = rospy.get_param(
            '~slope_approach_distance', 0.25)
        self.slope_max_cross_track_error = rospy.get_param(
            '~slope_max_cross_track_error', 0.25)
        self.slope_min_front_clearance = rospy.get_param(
            '~slope_min_front_clearance', 0.25)
        self.slope_front_angle = rospy.get_param('~slope_front_angle', 0.35)
        # base_link -> laser 的静态变换绕 Z 轴旋转 pi，因此雷达原始角度 pi
        # 才对应车体正前方。参数保留给现场标定覆盖。
        self.slope_scan_forward_angle = rospy.get_param(
            '~slope_scan_forward_angle', math.pi)
        self.slope_scan_timeout = rospy.get_param('~slope_scan_timeout', 0.5)
        # 到达 A 后 AMCL 可能因底盘静止而暂不重发位姿；仅允许用最近一次
        # 有效位姿做一次低速起步，正式直线行驶仍要求持续的新鲜位姿。
        self.slope_initial_pose_grace = rospy.get_param(
            '~slope_initial_pose_grace', 1.0)
        self.slope_initial_bootstrap_speed = rospy.get_param(
            '~slope_initial_bootstrap_speed', 0.1)

        # 状态变量
        self.current_goal = None
        self.goal_queue = []
        self.is_navigating = False
        self.retry_count = 0
        self.navigation_paused = False
        self.navigation_cancelled = False
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
        self.current_pose_time = None
        self.current_model = ''
        self.waiting_for_shooting_model = False
        self.shooting_mode_active = False
        self.model_switch_timer = None
        self.active_material_scan_target = None
        self.pending_material_scan_target = None
        self.material_scan_switch_timer = None
        self.latest_front_clearance = None
        self.latest_scan_time = None
        self.slope_mode_active = False
        self.slope_aligning = False
        self.slope_end_goal = None
        self.slope_control_timer = None
        self.slope_start_time = None
        self.slope_mode_enter_time = None
        # rospy 的 Timer、取消和目标回调可并发执行。用可重入锁保证停止命令
        # 不会被仍在运行的一次控制回调随后覆盖。
        self.slope_control_lock = threading.RLock()

        # 订阅话题
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
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # RKNN publishes this latched topic only after a model is actually active.
        self.current_model_sub = rospy.Subscriber(
            self.current_model_topic, String, self.current_model_callback)

        # 目标检测状态必须先完成初始化，再订阅，避免启动瞬间回调访问未创建字段。
        self.target_found = False
        self.interrupted_goal = None

        # 目标检测话题订阅
        self.target_sub = rospy.Subscriber(
            '/target_detected', Bool, self.target_callback)

        # 返航基地坐标
        self.base_x = rospy.get_param('~base_x', 0)
        self.base_y = rospy.get_param('~base_y', 0)
        self.base_yaw = rospy.get_param('~base_yaw', 0)

        # 发布话题
        self.status_pub = rospy.Publisher(
            '/auto_navigation/status', NavigationStatus, queue_size=10)
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        # 导航成功且坐标匹配时，将具体的 B 点编号发布给云台。
        self.arrived_material_pub = rospy.Publisher(
            self.arrived_material_topic, UInt8, queue_size=10)
        self.material_scan_target_pub = rospy.Publisher(
            self.material_scan_target_topic, UInt8, queue_size=1)
        self.model_switch_pub = rospy.Publisher(
            self.model_switch_topic, String, queue_size=1)
        # This is the only automatic entry point for gimbal shooting scan mode.
        self.shooting_mode_pub = rospy.Publisher(
            self.shooting_mode_topic, Bool, queue_size=1, latch=True)
        # 直线过坡期间只有本节点向 /cmd_vel 发布，move_base 已在 A 点成功后
        # 完成并停止跟踪，避免两个控制器同时抢占底盘。
        self.slope_cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.on_shutdown(self.shutdown_callback)

        # Action客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # 等待move_base服务器
        rospy.loginfo("等待move_base服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务器已连接")

        # 仅在状态变量和 move_base 客户端全部就绪后再接收路径，防止首个
        # 目标到来时访问尚未初始化的 target_found/move_base_client。
        self.goal_sub = rospy.Subscriber(
            '/auto_navigation/goal', NavigationGoal, self.goal_callback)

        if self.switch_model_on_route_complete:
            rospy.loginfo(
                "打靶点到达后将发布模型切换: %s -> %s",
                self.model_switch_topic,
                self.route_complete_model)
        if self.slope_mode_enabled:
            rospy.loginfo(
                "直线过坡已启用：A=(%.2f, %.2f) -> B=(%.2f, %.2f), "
                "速度=%.2fm/s, 目标容差=%.2fm, 朝向=%.3frad",
                self.slope_start[0], self.slope_start[1],
                self.slope_end[0], self.slope_end[1],
                self.slope_linear_speed, self.slope_goal_tolerance,
                self.slope_heading_yaw)

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
        should_return = False
        with self.slope_control_lock:
            if msg.data and not self.target_found:
                self.target_found = True
                rospy.logwarn("检测到目标，准备返航")

                if self.current_goal:
                    self.interrupted_goal = self.current_goal
                    rospy.loginfo(
                        "保存被中断的目标: %s", self.current_goal.description)

                # 与直线接管共用同一把锁，确保保存的是接管前 A 或接管后 B，
                # 不会在队列删除的中间状态捕获错误目标。
                self._cancel_navigation_locked(persistent=False)
                should_return = True
        if should_return:
            self.return_to_base()

    def execute_next_goal(self):
        with self.slope_control_lock:
            return self._execute_next_goal_locked()

    def _execute_next_goal_locked(self):
        """执行下一个目标"""
        if self.navigation_paused or self.navigation_cancelled:
            rospy.loginfo("导航处于暂停/取消状态，等待显式恢复")
            return
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

        # 只选择下一个“裁判物资点”，忽略路线中的中转点。这样底盘驶向
        # 该物资点时云台已经在对应 Pitch/Yaw 小窗口内扫描。
        self.prepare_material_scan_for_pending_route()

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
        if self.target_found or self.slope_mode_active:
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
            self.handle_successful_goal(completed_goal, allow_slope_mode=True)
        else:
            rospy.logwarn("导航失败: %s", self.current_goal.description)
            self.current_status = "failed"
            self.handle_navigation_failure()

    def publish_route_complete_model_switch(self):
        """Enter shooting only after the shooting model reports ready."""
        if not self.switch_model_on_route_complete:
            rospy.loginfo("已到达打靶点，但自动打靶模式已关闭")
            return

        model_name = str(self.route_complete_model).strip()
        if not model_name:
            rospy.logerr("route_complete_model 为空，未进入打靶模式")
            return

        # Keep the gimbal stopped until the requested model confirms activation.
        self.shooting_mode_pub.publish(Bool(data=False))
        self.waiting_for_shooting_model = True
        self.shooting_mode_active = False
        self.model_switch_pub.publish(String(data=model_name))
        rospy.loginfo(
            "打靶点已到达，等待模型就绪: %s -> %s",
            self.model_switch_topic,
            model_name)
        if self.model_switch_timer is not None:
            self.model_switch_timer.shutdown()
        self.model_switch_timer = rospy.Timer(
            rospy.Duration(self.model_switch_timeout),
            self.model_switch_timeout_callback, oneshot=True)

        # A latched /current_model may already contain the requested model.
        if self.current_model == model_name:
            self.activate_shooting_mode()

    def current_model_callback(self, msg):
        """Record the model that has completed loading in the RKNN node."""
        model_name = str(msg.data).strip()
        if not model_name:
            return
        self.current_model = model_name
        rospy.loginfo("RKNN active model: %s", model_name)
        if (self.waiting_for_shooting_model and
                model_name == str(self.route_complete_model).strip()):
            self.activate_shooting_mode()

    def activate_shooting_mode(self):
        """Release material pose lock and start scan after model readiness."""
        if self.shooting_mode_active:
            return
        if self.model_switch_timer is not None:
            self.model_switch_timer.shutdown()
            self.model_switch_timer = None
        self.waiting_for_shooting_model = False
        self.shooting_mode_active = True
        self.shooting_mode_pub.publish(Bool(data=True))
        rospy.loginfo(
            "打靶模型 %s 已就绪，已发布 %s=true，云台开始扫描",
            self.current_model, self.shooting_mode_topic)

    def model_switch_timeout_callback(self, event):
        if not self.waiting_for_shooting_model:
            return
        self.waiting_for_shooting_model = False
        rospy.logerr(
            "打靶模型 %s 在 %.1f 秒内未就绪，云台保持停止扫描",
            self.route_complete_model, self.model_switch_timeout)

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
        self.current_pose_time = rospy.Time.now()

    def scan_callback(self, msg):
        """保存车头扇区的最近激光距离，作为直线过坡的急停门槛。"""
        closest = None
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if (abs(self.normalize_angle(
                    angle - self.slope_scan_forward_angle)) > self.slope_front_angle):
                continue
            # 本仓 rplidar 驱动用 +inf 表示该方向没有回波，即量程内无障碍；
            # NaN、负无穷或范围外数值仍视为无效。扇区全无有效射线时保持 None。
            if math.isinf(distance) and distance > 0.0:
                candidate = msg.range_max
            elif math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                candidate = distance
            else:
                continue
            closest = candidate if closest is None else min(closest, candidate)
        self.latest_front_clearance = closest
        self.latest_scan_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        """将角度规约到 [-pi, pi]。"""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(value, upper))

    @staticmethod
    def pose_yaw(pose):
        """从 geometry_msgs/Pose 的四元数计算 Yaw。"""
        orientation = pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_matches_slope_point(self, goal, point):
        if goal is None:
            return False
        position = goal.goal_pose.pose.position
        return (math.hypot(position.x - point[0], position.y - point[1]) <=
                self.slope_route_match_tolerance)

    def find_slope_end_goal_index(self):
        """在 A 之后的队列中找到第一个 B，并返回其索引。"""
        for index, goal in enumerate(self.goal_queue):
            if self.goal_matches_slope_point(goal, self.slope_end):
                return index
        return None

    def start_slope_mode_if_applicable(self, completed_goal):
        with self.slope_control_lock:
            return self._start_slope_mode_if_applicable_locked(completed_goal)

    def _start_slope_mode_if_applicable_locked(self, completed_goal):
        """仅在路线实际经过 A→B 时接管中间坡道路段。"""
        if (not self.slope_mode_enabled or self.slope_mode_active or
                self.navigation_paused or self.navigation_cancelled or
                self.target_found or
                not self.goal_matches_slope_point(completed_goal, self.slope_start)):
            return False

        end_index = self.find_slope_end_goal_index()
        if end_index is None:
            rospy.logwarn(
                "到达坡道 A，但后续路线没有匹配 B=(%.2f, %.2f)；继续正常导航",
                self.slope_end[0], self.slope_end[1])
            return False

        if self.current_pose is None or self.current_pose_time is None:
            rospy.logwarn("到达坡道 A 时没有 AMCL 位姿；放弃直线接管，继续正常导航")
            return False
        pose_age = (rospy.Time.now() - self.current_pose_time).to_sec()
        if pose_age < 0.0 or pose_age > self.slope_takeover_max_pose_age:
            rospy.logwarn(
                "到达坡道 A 时 AMCL 位姿年龄 %.2fs 超过 %.2fs；"
                "放弃直线接管，继续正常导航",
                pose_age, self.slope_takeover_max_pose_age)
            return False
        position = self.current_pose.position
        position_error = math.hypot(
            position.x - self.slope_start[0], position.y - self.slope_start[1])
        if position_error > self.slope_takeover_max_position_error:
            rospy.logwarn(
                "到达坡道 A 时定位偏差 %.2fm 超过 %.2fm；"
                "放弃直线接管，继续正常导航",
                position_error, self.slope_takeover_max_position_error)
            return False
        entry_yaw = self.pose_yaw(self.current_pose)
        entry_heading_error = self.normalize_angle(
            self.slope_heading_yaw - entry_yaw)
        if abs(entry_heading_error) > self.slope_takeover_max_heading_error:
            rospy.logwarn(
                "到达坡道 A 时航向偏差 %.3frad 超过 %.3frad；"
                "放弃直线接管，继续正常导航",
                entry_heading_error, self.slope_takeover_max_heading_error)
            return False

        # A 与 B 之间的原始密集路径点仅用于原先的自动规划；直线模式接管
        # 后跳过它们，但保留 B 之后的全部路线。
        skipped_count = end_index
        self.slope_end_goal = self.goal_queue[end_index]
        del self.goal_queue[:end_index + 1]
        self.current_goal = self.slope_end_goal
        self.slope_mode_active = True
        self.slope_aligning = True
        self.slope_start_time = rospy.Time.now()
        self.slope_mode_enter_time = self.slope_start_time
        self.current_status = "slope_align"
        self.is_navigating = True
        self.invalidate_current_goal(stop_tracking=True)
        self.publish_slope_stop()
        self.slope_control_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.slope_control_rate, 1.0)),
            self.slope_control_callback)
        if self.slope_test_bypass_safety:
            rospy.logwarn(
                "直线过坡测试旁路已启用：AMCL 新鲜度、激光、超时和"
                "横向偏差保护已关闭；B 平面硬停车仍保留")
        rospy.logwarn(
            "进入直线过坡模式：A=(%.2f, %.2f) -> B=(%.2f, %.2f)，"
            "跳过 %d 个坡道中间导航点，速度上限 %.2fm/s",
            self.slope_start[0], self.slope_start[1],
            self.slope_end[0], self.slope_end[1], skipped_count,
            self.slope_linear_speed)
        return True

    def publish_slope_stop(self):
        self.slope_cmd_vel_pub.publish(Twist())

    def stop_slope_mode(self):
        """停止直线控制并发送零速度，不能让过坡命令残留。"""
        timer = None
        with self.slope_control_lock:
            timer = self.slope_control_timer
            self.slope_control_timer = None
            self.slope_mode_active = False
            self.slope_aligning = False
            self.slope_start_time = None
            self.slope_mode_enter_time = None
            self.slope_end_goal = None
            self.publish_slope_stop()
        if timer is not None:
            timer.shutdown()

    def shutdown_callback(self):
        """节点正常退出时发送零速度，避免直线控制命令残留。"""
        self.stop_slope_mode()

    def finish_slope_mode(self, success, reason):
        completed_goal = self.current_goal
        self.stop_slope_mode()
        if not success:
            # B 已在直线模式接管时从队列中取出。失败停车后把它放回队首，
            # 这样后续显式恢复导航时不会越过坡道终点直接执行下一个目标。
            if (completed_goal is not None and
                    (not self.goal_queue or
                     self.goal_queue[0] is not completed_goal)):
                self.goal_queue.insert(0, completed_goal)
            self.current_status = "slope_failed"
            self.is_navigating = False
            rospy.logerr("直线过坡已安全停车：%s", reason)
            return

        rospy.loginfo("直线过坡完成，到达 B：%s", completed_goal.description)
        self.handle_successful_goal(completed_goal, allow_slope_mode=False)

    def slope_control_callback(self, _event):
        with self.slope_control_lock:
            self._slope_control_callback_locked()

    def _slope_control_callback_locked(self):
        """A→B 的闭环直线控制：阿克曼底盘低速转向对齐后直线前进。"""
        if not self.slope_mode_active:
            return
        if self.navigation_paused or self.navigation_cancelled:
            self.stop_slope_mode()
            return
        if self.target_found:
            self.stop_slope_mode()
            return
        # 对齐阶段现在也会低速移动，因此正常保护模式下同样受总超时限制。
        if (not self.slope_test_bypass_safety and
                (rospy.Time.now() - self.slope_start_time).to_sec() > self.slope_timeout):
            self.finish_slope_mode(False, "过坡控制超时")
            return
        if self.current_pose is None or self.current_pose_time is None:
            self.publish_slope_stop()
            rospy.logwarn_throttle(1.0, "直线过坡等待有效 AMCL 位姿，保持停车")
            return
        pose_age = (rospy.Time.now() - self.current_pose_time).to_sec()
        if pose_age < 0.0:
            self.publish_slope_stop()
            rospy.logwarn_throttle(1.0, "检测到 ROS 时间倒退，直线过坡保持停车")
            return
        initial_pose_grace_active = (
            self.slope_mode_enter_time is not None and
            (rospy.Time.now() - self.slope_mode_enter_time).to_sec() <=
            self.slope_initial_pose_grace and
            pose_age <= self.slope_takeover_max_pose_age)
        if (not self.slope_test_bypass_safety and pose_age > 0.5 and
                not initial_pose_grace_active):
            self.publish_slope_stop()
            rospy.logwarn_throttle(1.0, "直线过坡等待有效 AMCL 位姿，保持停车")
            return
        if pose_age > 0.5 and not self.slope_test_bypass_safety:
            rospy.logwarn_throttle(
                1.0, "坡道入口沿用 %.2fs 前的 AMCL 位姿低速起步，"
                "最多 %.1fs，随后仍要求新鲜位姿",
                pose_age, self.slope_initial_pose_grace)
        position = self.current_pose.position
        current_yaw = self.pose_yaw(self.current_pose)
        command = Twist()

        # 低速对齐也会让车辆移动，所以所有运动共用同一组安全门槛。
        distance_to_end = math.hypot(
            self.slope_end[0] - position.x, self.slope_end[1] - position.y)
        if distance_to_end <= self.slope_goal_tolerance:
            self.finish_slope_mode(True, "到达 B")
            return

        line_dx = self.slope_end[0] - self.slope_start[0]
        line_dy = self.slope_end[1] - self.slope_start[1]
        line_length = math.hypot(line_dx, line_dy)
        if line_length <= 1e-6:
            self.finish_slope_mode(False, "坡道 A/B 不能重合")
            return

        unit_x = line_dx / line_length
        unit_y = line_dy / line_length
        rel_x = position.x - self.slope_start[0]
        rel_y = position.y - self.slope_start[1]
        progress = rel_x * unit_x + rel_y * unit_y
        cross_track_error = -unit_y * rel_x + unit_x * rel_y

        if (not self.slope_test_bypass_safety and
                (self.latest_front_clearance is None or
                self.latest_scan_time is None or
                (rospy.Time.now() - self.latest_scan_time).to_sec() > self.slope_scan_timeout)):
            self.publish_slope_stop()
            rospy.logwarn_throttle(1.0, "直线过坡等待有效的前向激光数据，保持停车")
            return
        if (not self.slope_test_bypass_safety and
                self.latest_front_clearance < self.slope_min_front_clearance):
            self.publish_slope_stop()
            rospy.logwarn_throttle(
                1.0, "直线过坡前方 %.2fm 小于安全距离 %.2fm，保持停车",
                self.latest_front_clearance, self.slope_min_front_clearance)
            return
        if (not self.slope_test_bypass_safety and
                abs(cross_track_error) > self.slope_max_cross_track_error):
            self.finish_slope_mode(
                False, "横向偏差 %.2fm 超过 %.2fm" % (
                    cross_track_error, self.slope_max_cross_track_error))
            return
        remaining = line_length - progress
        # B 平面是不可绕过的硬边界；即使现场旁路开启，也不能继续向前开。
        if remaining <= 0.0:
            self.finish_slope_mode(False, "已到达或越过 B 平面但横向位置不合格")
            return

        speed_scale = self.clamp(
            remaining / max(self.slope_approach_distance, 1e-3), 0.0, 1.0)

        # 对齐和正常行驶使用同一个、包含横向误差修正的航向目标，避免两个
        # 状态互相拉扯形成锯齿。
        desired_yaw = self.normalize_angle(
            self.slope_heading_yaw - math.atan(
                self.slope_cross_track_kp * cross_track_error))
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)

        # 直线行驶中朝向再次偏离时，重新低速转向校准。阿克曼底盘必须保持
        # 非零线速度，dzactuator 才能把 angular.z 转换成有效转角。
        if (not self.slope_aligning and
                abs(yaw_error) > self.slope_realign_tolerance):
            self.slope_aligning = True
            rospy.loginfo("坡道航向误差 %.3frad，重新进行低速转向校准", yaw_error)
        if self.slope_aligning:
            self.current_status = "slope_align"
            if abs(yaw_error) <= self.slope_heading_tolerance:
                self.slope_aligning = False
                rospy.loginfo("坡道朝向已对齐到目标 %.3frad，开始/恢复直线前进",
                              desired_yaw)
            else:
                alignment_speed = min(
                    max(0.0, self.slope_alignment_linear_speed),
                    max(0.0, self.slope_linear_speed))
                if pose_age > 0.5 and not self.slope_test_bypass_safety:
                    alignment_speed = min(
                        alignment_speed, self.slope_initial_bootstrap_speed)
                if alignment_speed <= 0.0:
                    self.finish_slope_mode(False, "坡道对齐线速度必须大于 0")
                    return
                command.linear.x = alignment_speed * speed_scale
                angular_limit = min(
                    self.slope_alignment_max_angular_speed,
                    command.linear.x * max(0.0, self.slope_max_curvature))
                command.angular.z = self.clamp(
                    self.slope_heading_kp * yaw_error,
                    -angular_limit, angular_limit)
                self.slope_cmd_vel_pub.publish(command)
                return

        # 横向误差只用于小幅修正车头，车辆始终以正向线速度直行，不允许
        # 自动规划重新选择斜向或绕行路线。
        drive_speed = self.slope_linear_speed
        if pose_age > 0.5 and not self.slope_test_bypass_safety:
            # 入口旧位姿只允许低速移动足以触发 AMCL 的最小距离；未等到新位姿
            # 就会在 initial_pose_grace 到期后重新停车。
            drive_speed = min(
                drive_speed, self.slope_initial_bootstrap_speed)
        command.linear.x = drive_speed * speed_scale
        angular_limit = min(
            self.slope_max_angular_speed,
            abs(command.linear.x) * max(0.0, self.slope_max_curvature))
        command.angular.z = self.clamp(
            self.slope_heading_kp * yaw_error,
            -angular_limit, angular_limit)
        self.current_status = "slope_drive"
        self.slope_cmd_vel_pub.publish(command)

    def handle_successful_goal(self, completed_goal, allow_slope_mode):
        with self.slope_control_lock:
            return self._handle_successful_goal_locked(
                completed_goal, allow_slope_mode)

    def _handle_successful_goal_locked(self, completed_goal, allow_slope_mode):
        """处理 move_base 或直线过坡完成后的共用路线状态。"""
        if self.target_found:
            rospy.loginfo("目标检测中断已生效，忽略迟到的导航成功回调")
            return
        self.current_status = "reached"
        self.is_navigating = False
        self.retry_count = 0
        arrived_material = self.publish_arrived_material_if_needed()

        # 到达物资点只启动云台自己的 3 秒切换计时；绝不改变底盘的
        # 停留时间或下一个导航目标。第四点没有下一个物资点，因此继续
        # 扫描第四点直到真正到达打靶点。
        if arrived_material:
            self.schedule_next_material_scan_if_available()

        if allow_slope_mode and self.start_slope_mode_if_applicable(completed_goal):
            return

        # 暂停/取消可能恰好发生在 move_base 成功与直线接管之间。状态锁存后
        # 不再调度下一目标，也不触发终点模型切换，等待显式恢复。
        if self.navigation_paused or self.navigation_cancelled:
            rospy.loginfo("到达当前目标后检测到暂停/取消，保持停车")
            return

        # All selected route files end at the shooting point. The final
        # goal is marked route_end by goal_publisher.
        if completed_goal.goal_type == "route_end":
            if self.goal_queue:
                rospy.logwarn(
                    "已到达 route_end，但队列中仍有 %d 个目标，暂不切换模型",
                    len(self.goal_queue))
            else:
                # 仅在实际到达打靶点时强制结束物资扫描并回中心。
                self.stop_material_scan_and_center()
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

    def find_arrived_material_number(self):
        """
        判断刚到达的路径点是否对应裁判系统要求识别的物资点。

        用当前导航目标的 x、y（即路径点坐标）与裁判物资点坐标匹配，
        不依赖 yaw。B1/B2 或 B15/B16 坐标相同时，因列表已升序，取较小编号。
        """
        return self.find_material_number_for_goal(self.current_goal)

    def find_material_number_for_goal(self, goal):
        """返回任意路线目标对应的裁判物资编号，普通路径点返回 None。"""
        if goal is None:
            return None
        if not self.referee_material_numbers:
            return None

        goal_x = goal.goal_pose.pose.position.x
        goal_y = goal.goal_pose.pose.position.y

        for number in self.referee_material_numbers:
            point_x, point_y, _ = self.MATERIAL_POINTS[number]
            distance = np.hypot(goal_x - point_x, goal_y - point_y)
            if distance <= self.material_arrive_tolerance:
                rospy.loginfo(
                    "路径点匹配物资点 %d，目标=(%.2f, %.2f)，距离=%.3fm",
                    number, goal_x, goal_y, distance)
                return number

        return None

    def find_next_selected_material_number(self):
        """从当前尚未执行的路线中找下一个实际需要识别的物资点。"""
        for goal in self.goal_queue:
            number = self.find_material_number_for_goal(goal)
            if number is not None:
                return number
        return None

    def prepare_material_scan_for_pending_route(self):
        """在行驶过程中预置下一物资点的局部扫描，普通中转点不改变目标。"""
        if self.shooting_mode_active:
            return

        # 物资到达后的三秒窗口内，云台必须继续扫描当前点。即使底盘已经
        # 启动下一个导航目标，也不能由本函数提前覆盖扫描目标。
        if self.pending_material_scan_target is not None:
            return

        pending_goals = [self.current_goal] + list(self.goal_queue)
        next_number = None
        for goal in pending_goals:
            next_number = self.find_material_number_for_goal(goal)
            if next_number is not None:
                break

        if next_number is None or next_number == self.active_material_scan_target:
            return

        self.material_scan_target_pub.publish(UInt8(data=next_number))
        self.active_material_scan_target = next_number
        rospy.loginfo(
            "驶向物资点期间预扫描：点位 %d -> %s",
            next_number, self.material_scan_target_topic)

    def schedule_next_material_scan_if_available(self):
        """只调度云台切换；不修改底盘导航计时或队列。"""
        next_number = self.find_next_selected_material_number()
        if next_number is None:
            rospy.loginfo("当前为最后一个物资点，云台保持扫描直到打靶点到达")
            return

        if self.material_scan_switch_timer is not None:
            self.material_scan_switch_timer.shutdown()

        self.pending_material_scan_target = next_number
        self.material_scan_switch_timer = rospy.Timer(
            rospy.Duration(self.material_scan_switch_delay),
            self.material_scan_switch_timer_callback, oneshot=True)
        rospy.loginfo(
            "物资到达通知已发布；云台继续扫描当前点，%.1f 秒后切换到点位 %d",
            self.material_scan_switch_delay, next_number)

    def material_scan_switch_timer_callback(self, _event):
        """独立于底盘状态，在三秒后切换云台局部扫描目标。"""
        next_number = self.pending_material_scan_target
        self.material_scan_switch_timer = None
        self.pending_material_scan_target = None
        if next_number is None or self.shooting_mode_active:
            return

        self.material_scan_target_pub.publish(UInt8(data=next_number))
        self.active_material_scan_target = next_number
        rospy.loginfo(
            "云台独立计时结束，已切换扫描点位 %d -> %s",
            next_number, self.material_scan_target_topic)

    def stop_material_scan_and_center(self):
        """打靶点到达后回到全局中心，并阻止视觉覆盖直到模型就绪。"""
        if self.material_scan_switch_timer is not None:
            self.material_scan_switch_timer.shutdown()
            self.material_scan_switch_timer = None
        self.pending_material_scan_target = None
        self.material_scan_target_pub.publish(UInt8(data=0))
        self.active_material_scan_target = None
        rospy.loginfo(
            "打靶点已到达：%s=0，云台回中心并等待打靶模型",
            self.material_scan_target_topic)

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

    def cancel_navigation(self, persistent=True):
        with self.slope_control_lock:
            return self._cancel_navigation_locked(persistent)

    def _cancel_navigation_locked(self, persistent=True):
        """取消导航"""
        if persistent:
            self.navigation_cancelled = True
        cancelled_goal = self.current_goal
        if self.slope_mode_active:
            self.stop_slope_mode()
            self.current_status = "cancelled"
            self.is_navigating = False
            if (persistent and cancelled_goal is not None and
                    (not self.goal_queue or self.goal_queue[0] is not cancelled_goal)):
                self.goal_queue.insert(0, cancelled_goal)
            rospy.loginfo("直线过坡已取消并停车")
            return
        if self.is_navigating:
            self.move_base_client.cancel_goal()
            self.invalidate_current_goal(stop_tracking=True)
            self.current_status = "cancelled"
            self.is_navigating = False
            if (persistent and cancelled_goal is not None and
                    (not self.goal_queue or self.goal_queue[0] is not cancelled_goal)):
                self.goal_queue.insert(0, cancelled_goal)
            rospy.loginfo("导航已取消")

    def pause_navigation(self):
        with self.slope_control_lock:
            return self._pause_navigation_locked()

    def _pause_navigation_locked(self):
        """暂停导航"""
        self.navigation_paused = True
        paused_goal = self.current_goal
        if self.slope_mode_active:
            self.stop_slope_mode()
            self.current_status = "paused"
            self.is_navigating = False
            if (paused_goal is not None and
                    (not self.goal_queue or self.goal_queue[0] is not paused_goal)):
                self.goal_queue.insert(0, paused_goal)
            rospy.loginfo("直线过坡已暂停并停车")
            return
        if self.is_navigating:
            self.move_base_client.cancel_goal()
            self.invalidate_current_goal(stop_tracking=True)
            self.current_status = "paused"
            self.is_navigating = False
            if (paused_goal is not None and
                    (not self.goal_queue or self.goal_queue[0] is not paused_goal)):
                self.goal_queue.insert(0, paused_goal)
            rospy.loginfo("导航已暂停")

    def resume_navigation(self):
        with self.slope_control_lock:
            return self._resume_navigation_locked()

    def _resume_navigation_locked(self):
        """恢复导航"""
        self.navigation_paused = False
        self.navigation_cancelled = False
        if not self.is_navigating and self.goal_queue:
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
        with self.slope_control_lock:
            if status == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("成功返航到基地")
                self.target_found = False

                if self.interrupted_goal:
                    interrupted_goal = self.interrupted_goal
                    rospy.loginfo(
                        "重新规划被中断的路径: %s",
                        interrupted_goal.description)
                    if not any(
                            goal is interrupted_goal for goal in self.goal_queue):
                        self.goal_queue.insert(0, interrupted_goal)
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
