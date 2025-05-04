#!/usr/bin/env python3

import rospy
from threading import Thread
from pynput import keyboard
from triton_project.msg import GestureCommand
import sys

rospy.init_node("gesture_keyboard_listener")
pub = rospy.Publisher("/gesture_command", GestureCommand, queue_size=10)

key_state = {}
stop_display = False

gesture_map = {
    'w': ("move_forward", 1),
    's': ("move_backward", 2),
    'a': ("turn_left", 3),
    'd': ("turn_right", 4),
    'z': ("dance", 6)
}

def key_update(key, state):
    if key not in key_state or key_state[key] != state:
        key_state[key] = state
        return True
    return False

def key_press(key):
    global stop_display
    if key == keyboard.Key.esc:
        stop_display = True
        print("\nPress Ctrl+C to exit")
        return False

    try:
        k = key.char.lower()
    except:
        k = key.name

    if key_update(k, True) and k in gesture_map:
        gesture_name, gesture_id = gesture_map[k]
        msg = GestureCommand(gesture_name=gesture_name, gesture_id=gesture_id)
        pub.publish(msg)
        rospy.loginfo(f"[KEY DOWN] -> {gesture_name} ({gesture_id})")
    return True

def key_release(key):
    try:
        k = key.char.lower()
    except:
        k = key.name

    if key_update(k, False) and k in gesture_map:
        stop_msg = GestureCommand(gesture_name="stop", gesture_id=5)
        pub.publish(stop_msg)
        rospy.loginfo("[KEY UP] -> stop (5)")
    return True

def user_display():
    print('Use W/A/S/D to simulate gestures.\nZ = dance\nESC = exit\n')
    while not rospy.is_shutdown() and not stop_display:
        try:
            sys.stdout.write("\rWaiting for key input... ")
            sys.stdout.flush()
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            break

# Start listeners
key_listener = keyboard.Listener(on_press=key_press, on_release=key_release)
key_listener.start()

display_thread = Thread(target=user_display)
display_thread.start()

rospy.spin()
