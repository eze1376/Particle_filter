#!/usr/bin/env python

#========================================#
#                                        #
#          IN THE NAME OF GOD            #
#          ------------------            #
#                                        #
#     Erfan Zekri Esfahani - 9815294     #
#              Question #1               #
#      Third Homework of Robotics        #
#                                        #
#========================================#

import getch
import rospy
from geometry_msgs.msg import Twist
import numpy as np


def Guassian_Sampler(mean, var):
    return np.random.normal(mean, var)


UP = 65
DOWN = 66
RIGHT = 67
LEFT = 68
QUIT = 100  # key q

MAX_VELOCITY = 1.1
MAX_ANGLE = 0.3
MIN_VELOCITY = -1.1
MIN_ANGLE = -0.3

forward = 0.0
angle = 0.0
PressedKey = 0

while(PressedKey != QUIT):
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.init_node('p3dx_mover')

	twist = Twist()

	PressedKey = getch.getArrow()

	if((PressedKey == UP) and (forward <= MAX_VELOCITY)):
	    forward += 0.1
	elif((PressedKey == DOWN) and (forward >= MIN_VELOCITY)):
	    forward -= 0.1
	elif((PressedKey == LEFT) and (angle >= MIN_ANGLE )):
	    angle += 0.1
	elif((PressedKey == RIGHT) and (angle <= MAX_ANGLE)):
	    angle -= 0.1

	twist.linear.x = forward
	twist.angular.z = angle
	pub.publish(twist)

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('p3dx_mover')
twist = Twist()
pub.publish(twist)
exit()
	
