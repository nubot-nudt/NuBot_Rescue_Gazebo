#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('nubot_teleop')#teleop_twist_keyboard
import rospy
from nubot_msgs.msg import base_auto_cmd


import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!

---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
---------------------------

anything else : stop

q/z : increase/decrease max speeds by 0.1
w/x : increase/decrease only linear speed by 0.1
e/c : increase/decrease only angular speed by 0.1

CTRL-C to quit
"""

moveBindings = {
        'i':(1.0,0,0,0),
        'o':(1,0,0,-0.5),
        'j':(1,0,0,1),
        'l':(1,0,0,-1),
        'u':(1,0,0,0.5),
        ',':(-1,0,0,0),
        '.':(-1,0,0,0.5),
        'm':(-1,0,0,-0.5),
    }

speedBindings={
        'q':(0.1,0.1),
        'z':(-0.1,-0.1),
        'w':(0.1,0),
        'x':(-0.1,0),
        'e':(0,0.1),
        'c':(0,-0.1),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/nubot_drive/base_auto_cmd', base_auto_cmd, queue_size = 1)
    rospy.init_node('teleop_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.5)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                print(vels(speed,turn))
                if (key == '\x03'):
                    break

            robot_cmd = base_auto_cmd()
            robot_cmd.vel_linear = x
            robot_cmd.vel_angular = -th
            robot_cmd.front_left = 3.1416
            robot_cmd.front_right = 3.1416
            robot_cmd.rear_left = 1
            robot_cmd.rear_right = 1
            pub.publish(robot_cmd)

    except Exception as e:
        print(e)

    finally:
            robot_cmd = base_auto_cmd()
            robot_cmd.vel_linear = 0
            robot_cmd.vel_angular = 0
            robot_cmd.front_left = 3.1416
            robot_cmd.front_right = 3.1416
            robot_cmd.rear_left = 1.5708
            robot_cmd.rear_right = 1.5708
            pub.publish(robot_cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
