#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仅云台的 1..16 物资点局部扫描测试。"""

import rospy
from std_msgs.msg import UInt8


class MaterialScanTest:
    def __init__(self):
        rospy.init_node('material_scan_test')
        self.dwell_seconds = rospy.get_param('~dwell_seconds', 5.0)
        self.startup_delay = rospy.get_param('~startup_delay', 1.0)
        self.topic = rospy.get_param(
            '~material_scan_target_topic', '/material_scan_target')
        self.publisher = rospy.Publisher(self.topic, UInt8, queue_size=1)
        self.point_id = 1

        if self.dwell_seconds <= 0.0:
            rospy.logfatal('~dwell_seconds 必须大于 0')
            raise rospy.ROSInitException('invalid dwell_seconds')

        rospy.loginfo(
            '云台扫描测试将在 %.1f 秒后开始：点位 1..16，每点 %.1f 秒',
            self.startup_delay, self.dwell_seconds)
        rospy.Timer(rospy.Duration(self.startup_delay), self.start_next_point,
                    oneshot=True)

    def start_next_point(self, _event):
        if rospy.is_shutdown():
            return

        if self.point_id <= 16:
            self.publisher.publish(UInt8(data=self.point_id))
            rospy.loginfo('测试扫描点位 %d/16，停留 %.1f 秒',
                          self.point_id, self.dwell_seconds)
            self.point_id += 1
            rospy.Timer(rospy.Duration(self.dwell_seconds),
                        self.start_next_point, oneshot=True)
            return

        # data=0 的正式语义是停止物资扫描、回中心并锁住，避免结束时遗留扫描。
        self.publisher.publish(UInt8(data=0))
        rospy.loginfo('1..16 点扫描测试完成，云台已收到回中心保持命令')


if __name__ == '__main__':
    MaterialScanTest()
    rospy.spin()
