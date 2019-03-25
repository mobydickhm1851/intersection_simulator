#!/usr/bin/env python

# 2018 11 13 LiuYC SOLab
# Add the command to control moving obstacle

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import numpy as np
import time


flag_move = 0

obsParam = {
        'f':(0.5), # diff increase 0.5 meter
        'd':(-0.5), # diff decrease 0.5 meter
        's':(0.2), # speed up * 1.2 
        'a':(0.2), # slow down * 1.2
           }

pub_rate = 100.0 # publish rate in Hz
speed = 50.0 # normal speed
diff = 2.0 # from -1 to 1, the moving range of obstacle


# this is return the keyboard input
def getKey():

    tty.setraw(sys.stdin.fileno())
    # the time-out argument ([3]) will affect the pub_rate since it stands for how long it will wait for the three list to have some value.([rlist] [wlist] [xlist])
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    # The fellowing if keep system from waiting for input
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# For now, only movement in y direction is set
def set_y_pose():

    # pub_x_joint_position = rospy.Publisher('/obs1/obs1_x_joint_controller/command', Float64, queue_size=1)
    # pode_x = data.linear.x
    # pub_x_joint_position.publish(throttle)
   
    count = 1
    global diff, speed

    pub_y_joint_position = rospy.Publisher('/obs1/obs1_y_joint_controller/command', Float64, queue_size=100)
    pub_y_joint_vel = rospy.Publisher('/obs1/obs1_y_vel', Float64, queue_size=100)
    rospy.init_node('obstacle_commands', anonymous=True)	
    rate = rospy.Rate(pub_rate) # default is 100
    
    while not rospy.is_shutdown():
        count = count +1 

	try:
	    key = getKey()
	    if key == 'f':
	        diff = diff + 0.5
	    elif key == 'd':
		diff = diff - 0.5
	    elif key == 's':
		speed = 1.2 * speed
	    elif key == 'a':
		speed = speed * 0.8
	    # ctrl+c will return '\x03'
	    elif key == '\x03':
		break
	finally:
	    # time_stamp, time.time() increase 1 for each second
            i = time.time() 
	    pose_y = np.sin( i / pub_rate * speed ) * diff
	    #rospy.loginfo(str)  #write to screen, node's log file and rosout
#            rospy.loginfo(pose_y)
	    vel_y = (speed / pub_rate) * np.cos(i / pub_rate * speed) * diff
 
            pub_y_joint_position.publish(pose_y)
            pub_y_joint_vel.publish(vel_y)
            rospy.loginfo(vel_y)

            rate.sleep()
#	    rospy.loginfo("count: %d",count)

if __name__ == '__main__':
    # part of getKeyUnix input
    settings = termios.tcgetattr(sys.stdin)
    try:
        set_y_pose()
    except rospy.ROSInterruptException:
        pass
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

