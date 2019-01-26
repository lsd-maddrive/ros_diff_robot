#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import numpy as np
import threading

rospy.init_node('control_link')

debug_enabled = rospy.get_param('~debug', False)
if debug_enabled:
    print('Debug enabled')

steer_value = 0
speed_value = 0
current_mode = 0

# 0 - Blue
# 1 - Green
# 2 - Red
# 3 - Yellow
# 4 - LU
# 5 - RU
# 6 - LB
# 7 - RB
# 10 - Left stick
# 11 - Right stick

button_names = ['Blue', 
                'Green',
                'Red',
                'Yellow',
                'Left upper',
                'Right upper',
                'Left bottom',
                'Right bottom',
                '_',
                '_',
                'Left stick',
                'Right stick'
                ]

axes_names = [  'Lstick horz',
                'Lstick vert',
                'Rstick horz',
                'Rstick vert',
                'Btns horz',
                'Btns vert'
                ]

def show_clicked(msg):
    print('Buttons:')
    for i in range(len(msg.buttons)):
        if msg.buttons[i] != 0:
            print('\t' + button_names[i])

    print('Axes:')
    for i in range(len(msg.axes)):
        print('\t%s: %.2f' % (axes_names[i], msg.axes[i]))

speed = 0
turn = 0

def joy_cb(msg):
    global speed, turn

    if debug_enabled:
        show_clicked(msg)
        
    speed = msg.axes[5]
    turn = msg.axes[4]

rospy.Subscriber('joy', Joy, joy_cb, queue_size = 10)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

if __name__ == '__main__':

    print('Ready, go')

    while not rospy.is_shutdown():

        vel_msg = Twist()
        
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = turn

        pub.publish( vel_msg )

        sleep( 100 / 1000. )
