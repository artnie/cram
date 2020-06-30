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
       +x    
        8   / 9 +z
        ^  /
+y      | /    -y
4 <-----5-----> 6
       /|
      / |
     /  V 
 -z 1   2 -x
Wrench:
+x = x
-x = X
+y = y
-y = Y
+z = z
-z = Z
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
    'X': ( 0, 0, 0,-1, 0, 0),
    'y': ( 0, 0, 0, 0, 1, 0),
    'Y': ( 0, 0, 0, 0,-1, 0),
    'z': ( 0, 0, 0, 0, 0, 1),
    'Z': ( 0, 0, 0, 0, 0,-1),
    '5': ( 0, 0, 0, 0, 0, 0)}

output_key_bindings = {
    '8': """Force:
       +x    
        8     9 +z
        ^  
+y      |      -y
4 <     5     > 6
        
      
        V 
 -z 1   2 -x
Wrench:
-
Cancel with ESC""",
    '4': """Force:
       +x    
        8     9 +z
        ^  
+y             -y
4 <-----5     > 6
        
      
        V 
 -z 1   2 -x
Wrench:
-
Cancel with ESC""",
    '9': """Force:
       +x    
        8   / 9 +z
        ^  / 
+y        /    -y
4 <     5     > 6
        
      
        V 
 -z 1   2 -x
Wrench:
-
Cancel with ESC""",
    '2': """Force:
       +x    
        8     9 +z
        ^   
+y             -y
4 <     5      > 6
        |
        |
        V 
 -z 1   2 -x
Wrench:
-
Cancel with ESC""",
    '6': """Force:
       +x    
        8     9 +z
        ^   
+y             -y
4 <     5-----> 6
         
         
        V 
 -z 1   2 -x
Wrench:
-
Cancel with ESC""",
    '1': """Force:
       +x    
        8     9 +z
        ^   
+y             -y
4 <     5     > 6
       /
      / 
     /  V 
 -z 1   2 -x
Wrench:
-
Cancel with ESC""",
    'x': """Force:
       +x    
        8     9 +z
        ^   
+y      |      -y
4 <     5     > 6
        |
        
        V 
 -z 1   2 -x
Wrench:
+x rot
Cancel with ESC""",
    'X': """Force:
       +x    
        8     9 +z
        ^   
+y      |      -y
4 <     5     > 6
        |
        
        V 
 -z 1   2 -x
Wrench:
-x rot
Cancel with ESC""",
    'y': """Force:
       +x    
        8     9 +z
        ^   
+y             -y
4 <   --5--   > 6
        
        
        V 
 -z 1   2 -x
Wrench:
+y rot
Cancel with ESC""",
    'Y': """Force:
       +x    
        8     9 +z
        ^   
+y             -y
4 <   --5--   > 6
        
        
        V 
 -z 1   2 -x
Wrench:
-y rot
Cancel with ESC""",
    'z': """Force:
       +x    
        8     9 +z
        ^  / 
+y        /    -y
4 <     5     > 6
       /
      / 
        V 
 -z 1   2 -x
Wrench:
+z rot
Cancel with ESC""",
    'Z': """Force:
       +x    
        8     9 +z
        ^  / 
+y        /    -y
4 <     5     > 6
       /
      / 
        V 
 -z 1   2 -x
Wrench:
-z rot
Cancel with ESC""",
    '5': """Force:
       +x    
        8   / 9 +z
        ^  /
+y      | /    -y
4 <-----5-----> 6
       /|
      / |
     /  V 
 -z 1   2 -x
Wrench:
+x = x
-x = X
+y = y
-y = Y
+z = z
-z = Z
Cancel with ESC"""}


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
                print(output_key_bindings[key])


        pub.publish(msg)
        time.sleep(.1)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
