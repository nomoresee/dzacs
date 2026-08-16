#!/usr/bin/env python3

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult


def main():
    rospy.init_node("fake_move_base_server")
    server = None

    def execute(_goal):
        rospy.loginfo("fake move_base received goal")
        rospy.sleep(0.1)
        server.set_succeeded(MoveBaseResult())

    server = actionlib.SimpleActionServer(
        "move_base", MoveBaseAction, execute_cb=execute, auto_start=False
    )
    server.start()
    rospy.loginfo("fake move_base ready")
    rospy.spin()


if __name__ == "__main__":
    main()
