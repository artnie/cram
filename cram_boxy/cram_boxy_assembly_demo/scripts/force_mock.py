#!/usr/bin/env python
import rospy
import time
import sys
import select
import termios
import tty
from geometry_msgs.msg._WrenchStamped import WrenchStamped
from geometry_msgs.msg import Vector3


manual = """
Reading from the keyboard and publishing to left_arm_kms40/wrench!
Apply force via numpad and wrench via x, y and z. Clear all forces with '5'.
Force:
       +x    9
        8   /+z
        ^  /
+y      | /    -y
4 <-----5-----> 6
       /|
      / |
     /  V 
 -z 1   2 -x
Wrench:
+x = x
+y = y
+z = z
Cancel with ESC
"""

key_bindings = {
    '8': ( 1, 0, 0, 0, 0, 0),
    '4': ( 0, 1, 0, 0, 0, 0),
    '9': ( 0, 0, 1, 0, 0, 0),
    '2': (-1, 0, 0, 0, 0, 0),
    '6': ( 0,-1, 0, 0, 0, 0),
    '1': ( 0, 0,-1, 0, 0, 0),
    'x': ( 0, 0, 0, 1, 0, 0),
    'y': ( 0, 0, 0, 0, 1, 0),
    'z': ( 0, 0, 0, 0, 0, 1),
    '5': ( 0, 0, 0, 0, 0, 0)}


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


old_settings = termios.tcgetattr(sys.stdin)
pub = rospy.Publisher('left_arm_kms40/wrench', WrenchStamped, queue_size=10)
rospy.init_node('kms40_left_force_mock')

try:
    tty.setcbreak(sys.stdin.fileno())
    print(manual)
    msg = WrenchStamped()
    msg.header.frame_id = "left_arm_kms40_frame_out"
    msg.wrench.force = Vector3(0, 0, 0)
    msg.wrench.torque = Vector3(0, 0, 0)
    while 1:
        msg.header.stamp = rospy.get_rostime()
        if is_data():
            key = sys.stdin.read(1)
            if key == '\x1b':  # x1b is ESC
                break
            if key in key_bindings.keys():
                msg.wrench.force = Vector3(key_bindings[key][0], key_bindings[key][1], key_bindings[key][2])
                msg.wrench.torque = Vector3(key_bindings[key][3], key_bindings[key][4], key_bindings[key][5])
                # outprint direction of force

        pub.publish(msg)
        time.sleep(.1)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
