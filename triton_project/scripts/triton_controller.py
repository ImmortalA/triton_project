#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from triton_project.msg import GestureCommand

vel_msg = Twist()
lock = False

def handle_gesture(msg):
    global vel_msg, lock

    rospy.loginfo(f"Gesture received: {msg.gesture_name} ({msg.gesture_id})")

    if msg.gesture_name == "move_forward":
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.0
        lock = True
    elif msg.gesture_name == "move_backward":
        vel_msg.linear.x = -0.3
        vel_msg.angular.z = 0.0
        lock = True
    elif msg.gesture_name == "turn_left":
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 1.0
        lock = True
    elif msg.gesture_name == "turn_right":
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = -1.0
        lock = True
    elif msg.gesture_name == "stop":
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        lock = False
    elif msg.gesture_name == "dance":
        rospy.loginfo("Dance mode coming soon!")
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        lock = False

def main():
    global vel_msg
    rospy.init_node('triton_controller')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gesture_command', GestureCommand, handle_gesture)
    rate = rospy.Rate(10)

    rospy.loginfo("Triton controller started. Listening for gesture commands...")

    while not rospy.is_shutdown():
        cmd_pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
