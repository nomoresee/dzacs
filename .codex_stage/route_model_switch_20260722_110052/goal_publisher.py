#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os

import rospy
import rospkg
from std_msgs.msg import UInt8MultiArray

from auto_navigation.msg import NavigationGoal, NavigationStatus


ORIGIN = (0.0, 0.0, 0.0, "返回起点")

# A 点仅作路径经过点，不参与命名。
# 物资点 -> 路径点编号；1/2 共用 B1，15/16 共用 B15。
MATERIAL_TO_PATH_POINT = {
    1: 1,
    2: 1,
    3: 3,
    7: 7,
    8: 8,
    9: 9,
    11: 11,
    12: 12,
    15: 15,
    16: 15,
}

# 文件名中的可选路径点（裁判最多 4 个物资 → 文件名最多 4 个编号）：
# - B1/B3/B7/B8/B9/B15/B11/B12：按物资出现写入文件名
# - 文件名编号按从小到大排列，例如 11_12，而不是路径经过顺序的 12_11
# - 若物资中不含 11 且不含 12：路径仍经过 B12，但不写入文件名（静默 B12）
OPTIONAL_PATH_POINTS = {1, 3, 7, 8, 9, 11, 12, 15}

# 必经物资点对应路径点，不写入文件名
ALWAYS_PATH_POINTS = {4, 5, 6, 10, 13, 14}

# 无命名可选点时的路径（仅含必经点 + 静默 B12）
DEFAULT_PATH_FILE = "0.txt"

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path("auto_navigation")
PATH_ROOT = os.path.join(PKG_PATH, "Roads")


class GoalPublisher:
    def __init__(self):
        rospy.init_node("goal_publisher", anonymous=True)

        self.max_queue_size = rospy.get_param("~max_queue_size", 25)
        self.material_topic = rospy.get_param(
            "~referee_material_topic", "/all_Material_Number"
        )

        self.goal_pub = rospy.Publisher(
            "/auto_navigation/goal",
            NavigationGoal,
            queue_size=self.max_queue_size,
        )

        self.status_sub = rospy.Subscriber(
            "/auto_navigation/status",
            NavigationStatus,
            self.status_callback,
        )

        self.material_sub = rospy.Subscriber(
            self.material_topic,
            UInt8MultiArray,
            self.material_callback,
            queue_size=10,
        )

        self.current_status = None
        self._last_status = None
        self.waiting_for_completion = False
        self.current_goal_index = 0
        self.current_path_goals = []
        self.wait_start_time = None
        self.wait_duration = 300.0
        self.path_completed = False
        self.waiting_after_path = False

        # 例（物资先升序，再选路；文件名编号亦升序）：
        # [7, 8, 2, 4] -> 排序 [2,4,7,8] -> 1_7_8.txt
        # [4, 5, 6, 10] -> 0.txt
        # [11, 12, 8, 4] -> 排序 [4,8,11,12] -> 8_11_12.txt
        self.selected_path_file = None
        self._path_started = False

        rospy.sleep(1.0)
        rospy.loginfo(
            "导航目标发布器已启动，默认路径 %s，等待裁判系统通过 %s 发布物资点",
            DEFAULT_PATH_FILE,
            self.material_topic,
        )

    # ---------- 裁判物资点 -> 路径文件 ----------
    def material_callback(self, msg):
        """根据裁判系统发布的 4 个物资点选择路径文件并启动导航。"""
        if self.selected_path_file is not None:
            return

        materials = [int(v) for v in msg.data if int(v) > 0]
        if len(materials) < 4:
            rospy.logwarn_throttle(
                2.0,
                "物资点数量不足 4 个: %s，继续等待",
                materials,
            )
            return

        materials = sorted(materials[:4])
        if len(set(materials)) != 4:
            rospy.logwarn_throttle(2.0, "裁判系统物资点存在重复: %s", materials)
            return
        if any(n < 1 or n > 16 for n in materials):
            rospy.logwarn_throttle(2.0, "裁判系统物资点编号超出 1～16: %s", materials)
            return
        if 15 in materials and 16 in materials:
            rospy.logerr_throttle(2.0, "物资点 15 与 16 互斥: %s", materials)
            return

        path_file = self.select_path_file(materials)
        if path_file is None:
            path_file = DEFAULT_PATH_FILE
            rospy.logwarn(
                "未找到匹配路径，回退默认: %s (物资点=%s)",
                path_file,
                materials,
            )

        self.selected_path_file = path_file
        # 终端仅反馈一次：排序后的物资点 + 选定路径
        rospy.loginfo("裁判系统物资点: %s -> 路径文件 %s", materials, path_file)
        self.start_selected_path()

    def select_path_file(self, materials):
        """
        materials 须已按升序排序。
        从 Roads/ 选择覆盖所需可选路径点、且多余命名点最少的文件。
        文件名编号为升序；无 11/12 物资时路径含静默 B12（不出现在文件名）。
        """
        required = self.materials_to_required_points(materials)
        if len(required) > 4:
            rospy.logerr(
                "所需命名路径点超过 4 个: %s (物资=%s)",
                sorted(required),
                materials,
            )
            return None

        expected_name = self.tags_to_filename(required)
        candidates = self.list_path_files()
        if not candidates:
            rospy.logerr("Roads/ 中没有路径 txt: %s", PATH_ROOT)
            return None

        scored = []
        for fname in candidates:
            tags = self.parse_path_filename(fname)
            if tags is None:
                continue
            if len(tags) > 4:
                continue
            if not required.issubset(tags):
                continue

            extra = len(tags - required)
            # 无 11/12 需求时，优先文件名也不含 11/12（对应静默 B12）
            if 11 not in required and 12 not in required:
                prefer_112 = 0 if 11 not in tags and 12 not in tags else 1
            elif 12 in required and 11 not in required:
                prefer_112 = 0 if 12 in tags and 11 not in tags else 1
            elif 11 in required and 12 not in required:
                prefer_112 = 0 if 11 in tags and 12 not in tags else 1
            else:
                prefer_112 = 0 if 11 in tags and 12 in tags else 1

            # 同等条件下优先与升序期望文件名完全一致
            prefer_exact = 0 if fname == expected_name else 1
            scored.append((extra, prefer_112, prefer_exact, fname))

        if not scored:
            return None

        scored.sort()
        best = scored[0][3]
        return best

    @staticmethod
    def tags_to_filename(tags):
        """将路径点编号集合转为升序文件名，空集为 0.txt。"""
        if not tags:
            return "0.txt"
        return "_".join(str(n) for n in sorted(tags)) + ".txt"

    @staticmethod
    def materials_to_required_points(materials):
        """物资编号 -> 需要出现在文件名中的可选路径点集合。"""
        required = set()
        for mat in materials:
            if mat in ALWAYS_PATH_POINTS:
                continue
            path_id = MATERIAL_TO_PATH_POINT.get(mat)
            if path_id is None:
                rospy.logwarn("未知物资点 %s，已忽略", mat)
                continue
            if path_id in OPTIONAL_PATH_POINTS:
                required.add(path_id)
        return required

    @staticmethod
    def parse_path_filename(fname):
        """'8_11_12.txt' -> {8, 11, 12}；'0.txt' -> 空集。文件名编号应为升序。"""
        stem = os.path.splitext(fname)[0]
        if not stem or stem == "default":
            return None
        if stem == "0":
            return set()
        tags = set()
        parts = stem.split("_")
        nums = []
        for part in parts:
            if not part.isdigit():
                return None
            nums.append(int(part))
            tags.add(int(part))
        # 允许读取，但非升序命名视为非法，避免匹配到旧文件
        if nums != sorted(nums):
            return None
        return tags

    @staticmethod
    def list_path_files():
        if not os.path.isdir(PATH_ROOT):
            return []
        return sorted(
            f
            for f in os.listdir(PATH_ROOT)
            if f.endswith(".txt") and os.path.isfile(os.path.join(PATH_ROOT, f))
        )

    def start_selected_path(self):
        if self._path_started or not self.selected_path_file:
            return
        self._path_started = True
        goals = self.load_path_file(self.selected_path_file)
        if not goals:
            rospy.logerr("加载路径失败: %s", self.selected_path_file)
            self._path_started = False
            self.selected_path_file = None
            return
        self.publish_sequence_goals_with_wait(goals)

    # ---------- 消息发布封装 ----------
    def publish_goal(
        self, x, y, yaw=0.0, description="目标点", priority=5, timeout=60.0,
        goal_type="point"
    ):
        goal = NavigationGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.goal_pose.header = goal.header
        goal.goal_pose.pose.position.x = x
        goal.goal_pose.pose.position.y = y

        goal.goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal.goal_type = goal_type
        goal.priority = priority
        goal.auto_execute = True
        goal.timeout = timeout
        goal.description = description

        self.goal_pub.publish(goal)
        rospy.loginfo("已发布导航目标: %s (%.2f, %.2f)", description, x, y)

    def status_callback(self, status):
        """整路径已入队，这里只统计到达。"""
        self.current_status = status

        if self.waiting_for_completion and status.status == "reached":
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
        """检查等待是否完成，完成后返回起点。"""
        if self.wait_start_time is not None and self.waiting_after_path:
            elapsed = (rospy.Time.now() - self.wait_start_time).to_sec()
            if elapsed >= self.wait_duration:
                rospy.loginfo("等待完成，返回起点...")
                self.wait_start_time = None
                self.waiting_after_path = False
                self.path_completed = False

                self.publish_sequence_goals([ORIGIN])
                self.current_path_goals = []
                self.current_goal_index = 0

    def publish_sequence_goals(self, goals):
        n = len(goals)
        for i, (x, y, yaw, desc) in enumerate(goals):
            self.publish_goal(x, y, yaw, desc, priority=n - i)
            rospy.sleep(0.05)

    def publish_sequence_goals_with_wait(self, goals):
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
            goal_type = "route_end" if i == n - 1 else "point"
            self.publish_goal(
                x,
                y,
                yaw,
                desc,
                priority=n - i,
                goal_type=goal_type,
            )
            rospy.sleep(0.05)

        rospy.loginfo(
            "已一次性入队整条路径，共 %d 个目标点（最后一点已标记为 route_end）",
            n)

    # ---------- 路径加载 ----------
    def load_path_file(self, filename):
        """加载指定路径 txt，返回 [(x, y, yaw, desc), ...]。"""
        if os.path.basename(filename) != filename:
            rospy.logerr("非法路径文件名: %s", filename)
            return []

        path = os.path.join(PATH_ROOT, filename)
        if not os.path.isfile(path):
            rospy.logerr("路径文件不存在: %s", path)
            return []

        name = os.path.splitext(filename)[0]
        waypoints = []
        with open(path, encoding="utf-8") as fp:
            for ln, line in enumerate(fp, 1):
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                try:
                    parts = list(map(float, line.split()))
                    if len(parts) < 2:
                        raise ValueError("至少需要 x、y")
                    x, y = parts[:2]
                    yaw = parts[2] if len(parts) > 2 else 0.0
                    waypoints.append((x, y, yaw, "{}_{}".format(name, ln)))
                except ValueError as error:
                    rospy.logwarn("%s:%d 行格式错误: %s", path, ln, error)

        if waypoints:
            rospy.loginfo("已加载 %s，共 %d 个点", filename, len(waypoints))
        return waypoints

    def spin_until_done(self):
        """等待裁判选路并执行完毕。"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.check_wait_completion()
            rate.sleep()


if __name__ == "__main__":
    try:
        publisher = GoalPublisher()
        publisher.spin_until_done()
    except rospy.ROSInterruptException:
        pass
