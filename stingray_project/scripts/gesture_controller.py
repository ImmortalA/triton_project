#!/usr/bin/env python3

import os
import cv2
import mediapipe as mp
import rospy
from geometry_msgs.msg import Twist
import threading
from std_srvs.srv import SetBool

# Hide TensorFlow/MediaPipe CUDA warnings
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# ROS init
rospy.init_node("gesture_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10)

rospy.wait_for_service('toggle_follower')
toggle = rospy.ServiceProxy('toggle_follower', SetBool)

# MediaPipe init
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

# Shared Twist and thread lock
shared_twist = Twist()
lock = threading.Lock()

# Velocity values
linear_vel = 0.3
angular_vel = 0.1

# Flag for human-following activation
human_following_active = False

def is_finger_up(tip, pip, wrist):
    return tip.y < pip.y < wrist.y

def classify_gesture(landmarks, handedness):
    wrist = landmarks[0]
    thumb_tip = landmarks[4]
    thumb_ip  = landmarks[3]
    index_tip = landmarks[8]
    index_pip = landmarks[6]
    middle_tip = landmarks[12]
    middle_pip = landmarks[10]
    ring_tip = landmarks[16]
    ring_pip = landmarks[14]
    pinky_tip = landmarks[20]
    pinky_pip = landmarks[18]

    # ✋ Open palm = stop
    if all([
        is_finger_up(index_tip, index_pip, wrist),
        is_finger_up(middle_tip, middle_pip, wrist),
        is_finger_up(ring_tip, ring_pip, wrist),
        is_finger_up(pinky_tip, pinky_pip, wrist),
        thumb_tip.x < wrist.x if thumb_tip.x < thumb_ip.x else thumb_tip.x > wrist.x
    ]):
        return "stop"

    # ✌️ V-sign = move forward
    if (
        is_finger_up(index_tip, index_pip, wrist) and
        is_finger_up(middle_tip, middle_pip, wrist) and
        not is_finger_up(ring_tip, ring_pip, wrist) and
        not is_finger_up(pinky_tip, pinky_pip, wrist)
    ):
        return "move_forward"

    # 👆 Point up (index only) = move backward
    if (
        is_finger_up(index_tip, index_pip, wrist) and
        not is_finger_up(middle_tip, middle_pip, wrist) and
        not is_finger_up(ring_tip, ring_pip, wrist)
    ):
        return "move_backward"

    # Thumb right (on right hand) = rotate right
    if handedness == "Right" and thumb_tip.x > index_tip.x + 0.05:
        return "rotate_right"

    # Thumb left (on left hand) = rotate left
    if handedness == "Left" and thumb_tip.x < index_tip.x - 0.05:
        return "rotate_left"

    # 👍 Thumbs up = activate human-following
    if (
        thumb_tip.y < thumb_ip.y and
        not is_finger_up(index_tip, index_pip, wrist) and
        not is_finger_up(middle_tip, middle_pip, wrist) and
        not is_finger_up(ring_tip, ring_pip, wrist) and
        not is_finger_up(pinky_tip, pinky_pip, wrist)
    ):
        return "activate_following"

    # 👎 Thumbs down = deactivate human-following
    if (
        thumb_tip.y > thumb_ip.y and
        not is_finger_up(index_tip, index_pip, wrist) and
        not is_finger_up(middle_tip, middle_pip, wrist) and
        not is_finger_up(ring_tip, ring_pip, wrist) and
        not is_finger_up(pinky_tip, pinky_pip, wrist)
    ):
        return "deactivate_following"

    return "none"

def publish_command(gesture):
    global shared_twist, human_following_active
    twist = Twist()

    if gesture == "activate_following":
        human_following_active = True
        rospy.loginfo("[GESTURE] Human-following ACTIVATED")
        toggle(True)
        return

    elif gesture == "deactivate_following":
        human_following_active = False
        rospy.loginfo("[GESTURE] Human-following DEACTIVATED")
        toggle(False)
        return

    if not human_following_active:
        return

    if gesture == "move_forward":
        twist.linear.x = linear_vel
    elif gesture == "move_backward":
        twist.linear.x = -linear_vel
    elif gesture == "rotate_left":
        twist.angular.z = angular_vel
    elif gesture == "rotate_right":
        twist.angular.z = -angular_vel * 5
    elif gesture == "stop":
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    else:
        return

    with lock:
        shared_twist = twist

    rospy.loginfo(f"[GESTURE] Sent: {gesture}")

# Background publisher thread
def continuous_publisher():
    while not rospy.is_shutdown():
        with lock:
            pub.publish(shared_twist)
        rate.sleep()

# Start webcam and publisher thread
cap = cv2.VideoCapture(0)
threading.Thread(target=continuous_publisher, daemon=True).start()

# Main loop
while not rospy.is_shutdown() and cap.isOpened():
    success, frame = cap.read()
    if not success:
        continue

    frame = cv2.flip(frame, 1)

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if results.multi_hand_landmarks:
        handedness = "Unknown"
        if results.multi_handedness:
            handedness = results.multi_handedness[0].classification[0].label

        for hand_landmarks in results.multi_hand_landmarks:
            landmarks = hand_landmarks.landmark
            gesture = classify_gesture(landmarks, handedness)
            publish_command(gesture)
            mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Show activation status on screen
    status_color = (0, 255, 0) if human_following_active else (0, 0, 255)
    cv2.putText(image, f"FOLLOWING: {'ON' if human_following_active else 'OFF'}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)

    cv2.imshow("Gesture Control", image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
