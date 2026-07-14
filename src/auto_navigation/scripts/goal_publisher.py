#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import rospy
import rospkg
from auto_navigation.msg import NavigationGoal, NavigationStatus

# ----------------- 常量 -----------------
ORIGIN = (0, 0, 0, "返回起点")

# 找到包根目录，并指向 Roads/
# 运行时 Roads/ 中应只放一个路径 txt，节点自动加载该文件
rospack   = rospkg.RosPack()
PKG_PATH  = rospack.get_path("auto_navigation")
PATH_ROOT = os.path.join(PKG_PATH, "Roads")
# ---------------------------------------

class GoalPublisher:
    def __init__(self):
        rospy.init_node("goal_publisher", anonymous=True)

        self.max_queue_size = rospy.get_param("~max_queue_size", 20)
        self.goal_pub = rospy.Publisher(
            "/auto_navigation/goal", NavigationGoal, queue_size=self.max_queue_size
        )

        # 订阅导航状态
        self.status_sub = rospy.Subscriber(
            "/auto_navigation/status", NavigationStatus, self.status_callback
        )
        
        # 状态变量
        self.current_status = None
        self._last_status = None
        self.waiting_for_completion = False
        self.current_goal_index = 0
        self.current_path_goals = []
        self.wait_start_time = None
        self.wait_duration = 300.0  # 等待时间（秒）
        self.path_completed = False  # 标记路径是否完成
        self.waiting_after_path = False  # 标记是否在路径完成后等待

        rospy.sleep(1.0)
        rospy.loginfo("导航目标发布器已启动（整路径入队，队列上限 %d）", self.max_queue_size)

    # ---------- 消息发布封装 ----------
    def publish_goal(self, x, y, yaw=0.0,
                     description="目标点", priority=5, timeout=60.0):
        goal = NavigationGoal()
        goal.header.stamp    = rospy.Time.now()
        goal.header.frame_id = "map"

        # 位置
        goal.goal_pose.header      = goal.header
        goal.goal_pose.pose.position.x = x
        goal.goal_pose.pose.position.y = y

        # 朝向
        goal.goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 其余字段
        goal.goal_type    = "point"
        goal.priority     = priority
        goal.auto_execute = True
        goal.timeout      = timeout
        goal.description  = description

        self.goal_pub.publish(goal)
        rospy.loginfo(f"已发布导航目标: {description} ({x:.2f}, {y:.2f})")

    def status_callback(self, status):
        """导航状态回调：整路径已入队，这里只统计到达，不再逐点发布"""
        self.current_status = status

        if self.waiting_for_completion and status.status == "reached":
            # 边沿触发，避免 status 定时重复发布 "reached" 导致重复计数
            if self._last_status != "reached":
                self.current_goal_index += 1
                rospy.loginfo(
                    "目标点 %d/%d 已到达",
                    self.current_goal_index,
                    len(self.current_path_goals),
                )
                if self.current_goal_index >= len(self.current_path_goals):
                    rospy.loginfo(
                        "路径所有目标点已完成，开始等待 %.0f 秒...",
                        self.wait_duration,
                    )
                    self.wait_start_time = rospy.Time.now()
                    self.waiting_for_completion = False
                    self.waiting_after_path = True
                    self.path_completed = True

        self._last_status = status.status
            
    def check_wait_completion(self):
        """检查等待是否完成"""
        if self.wait_start_time is not None and self.waiting_after_path:
            elapsed = (rospy.Time.now() - self.wait_start_time).to_sec()
            if elapsed >= self.wait_duration:
                rospy.loginfo(f"等待完成，返回起点...")
                self.wait_start_time = None
                self.waiting_after_path = False
                self.path_completed = False
                
                # 返回起点
                self.publish_sequence_goals([ORIGIN])
                self.current_path_goals = []
                self.current_goal_index = 0

    def publish_sequence_goals(self, goals):
        """一次性发布若干目标点入队（按顺序，前点优先级更高）"""
        n = len(goals)
        for i, (x, y, yaw, desc) in enumerate(goals):
            self.publish_goal(x, y, yaw, desc, priority=n - i)
            rospy.sleep(0.05)

    def publish_sequence_goals_with_wait(self, goals):
        """一次性把整条路径入队，全部到达后再等待"""
        if not goals:
            return

        if len(goals) > self.max_queue_size:
            rospy.logwarn(
                "路径点数 %d 超过队列上限 %d，已截断",
                len(goals),
                self.max_queue_size,
            )
            goals = goals[: self.max_queue_size]

        self.current_path_goals = goals
        self.current_goal_index = 0
        self._last_status = None
        self.waiting_for_completion = True
        self.path_completed = False
        self.waiting_after_path = False

        n = len(goals)
        for i, (x, y, yaw, desc) in enumerate(goals):
            # 靠前的点 priority 更大，排序后仍保持路径顺序
            self.publish_goal(x, y, yaw, desc, priority=n - i)
            rospy.sleep(0.05)

        rospy.loginfo("已一次性入队整条路径，共 %d 个目标点", n)

    # ---------- 路径加载 ----------
    def load_paths(self):
        """加载 Roads/ 中唯一的路径 txt，返回 [[(x,y,yaw,desc)…], …]"""
        if not os.path.isdir(PATH_ROOT):
            rospy.logerr(f"未找到路径目录: {PATH_ROOT}")
            return []

        txt_files = sorted(
            f for f in os.listdir(PATH_ROOT)
            if f.endswith(".txt") and os.path.isfile(os.path.join(PATH_ROOT, f))
        )
        if not txt_files:
            rospy.logerr(f"路径目录中没有 txt 文件: {PATH_ROOT}")
            return []
        if len(txt_files) > 1:
            rospy.logerr(
                f"路径目录中应只有 1 个 txt，当前有 {len(txt_files)} 个: {txt_files}"
            )
            return []

        target = txt_files[0]
        file = os.path.join(PATH_ROOT, target)
        name = os.path.splitext(target)[0]
        waypoints = []
        with open(file, encoding="utf-8") as fp:
            for ln, line in enumerate(fp, 1):
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                try:
                    parts = list(map(float, line.split()))
                    x, y = parts[:2]
                    yaw = parts[2] if len(parts) > 2 else 0.0
                    waypoints.append((x, y, yaw, f"{name}_{ln}"))
                except ValueError:
                    rospy.logwarn(f"{file}:{ln} 行格式错误，已跳过")

        if not waypoints:
            return []

        rospy.loginfo(f"已加载 {target}，共 {len(waypoints)} 个点")
        return [waypoints]

    # ---------- 主执行逻辑 ----------
    def run_all_paths(self):
        paths = self.load_paths()
        if not paths:
            rospy.logwarn("未检测到任何路径，退出")
            return

        rospy.loginfo(f"共检测到 {len(paths)} 条路径，将依次执行")
        for idx, goals in enumerate(paths, 0):
            rospy.loginfo(f"开始执行第 {idx} 条路径")
            self.publish_sequence_goals_with_wait(goals)
            
            # 等待当前路径完成
            while self.current_path_goals:
                self.check_wait_completion()  # 检查等待状态
                rospy.sleep(0.1)  # 更频繁地检查

            rospy.loginfo(f"第 {idx} 条路径已完成并返回起点\n")

# ----------------- main -----------------
if __name__ == "__main__":
    try:
        publisher = GoalPublisher()
        publisher.run_all_paths()   # 启动立即执行所有路径
        rospy.spin()
    except rospy.ROSInterruptException:
        pass