#!/usr/bin/env python3

import sys
import time

import rospy
from auto_navigation.msg import NavigationGoal
from std_msgs.msg import String


switch_requests = []
current_models = []


def switch_callback(message):
    switch_requests.append(message.data)


def current_model_callback(message):
    current_models.append(message.data)


def main():
    rospy.init_node("route_switch_e2e_client")
    rospy.Subscriber("/switch_model", String, switch_callback, queue_size=10)
    rospy.Subscriber("/current_model", String, current_model_callback, queue_size=10)
    goal_pub = rospy.Publisher(
        "/auto_navigation/goal", NavigationGoal, queue_size=1
    )

    connection_deadline = time.monotonic() + 20.0
    while goal_pub.get_num_connections() == 0:
        if time.monotonic() >= connection_deadline:
            raise RuntimeError("auto_navigation goal subscriber did not connect")
        rospy.sleep(0.1)

    goal = NavigationGoal()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.goal_pose.header = goal.header
    goal.goal_pose.pose.orientation.w = 1.0
    goal.goal_type = "route_end"
    goal.priority = 1
    goal.auto_execute = True
    goal.timeout = 10.0
    goal.description = "isolated_e2e_route_end"
    goal_pub.publish(goal)

    result_deadline = time.monotonic() + 30.0
    while time.monotonic() < result_deadline and not rospy.is_shutdown():
        request_ok = "light_det2" in switch_requests
        model_ok = bool(current_models) and current_models[-1] == "light_det2"
        if request_ok and model_ok:
            print("navigation request: light_det2")
            print("detector current model: light_det2")
            print("isolated end-to-end test: OK")
            return
        rospy.sleep(0.1)

    print("switch requests: {}".format(switch_requests), file=sys.stderr)
    print("current models: {}".format(current_models), file=sys.stderr)
    raise RuntimeError("route completion did not switch detector to light_det2")


if __name__ == "__main__":
    main()
