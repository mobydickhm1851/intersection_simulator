#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np


#--- Parameter Setting ---#

update_rate = 10   # in millisec
#style.use('seaborn-whitegrid')
style.use('bmh')

#-------------------------#

#--- Color Setting ---#

light_blue = '#103A90'
green_blue = '#87CEFA'
light_orange = '#EE5100'
light_yellow = '#EE9500'
apple_green = '#B2DF36'
black = '#000000'
#---------------------#

vel_profile_0 = np.array([[0, 0]])   # timestamp, car0_y
vel_profile_1 = np.array([[0, 0]])   # timestamp, car1_x
position_profile_0 = np.array([[0, -15]])   # timestamp, car0_y
position_profile_1 = np.array([[0, -15]])   # timestamp, car1_x
start_time = 0.0   # define as the timestamp when the cars begin to move
count = 0

def rosbag_callback(pose, i):
    global vel_profile_0, vel_profile_1, count, start_time, position_profile_0, position_profile_1

    time_vel=[0.0, 0.0]
    time_position = [0.0, 0.0]

    current_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9

    # Plot from the moment they start to move
    if pose.twist.twist.linear.y != 0 or pose.twist.twist.linear.x != 0:
        
        if count == 0:
            start_time = current_time
            count += 1

        if i == 0:  # car0
            time_vel[0] = current_time - start_time
            time_vel[1] = pose.twist.twist.linear.y
            time_position[0] = current_time - start_time
            time_position[1] = pose.pose.pose.position.y
            vel_profile_0 = np.append(vel_profile_0, [time_vel], 0)
            position_profile_0 = np.append(position_profile_0, [time_position], 0)

        elif i == 1: # car1
            time_vel[0] = current_time - start_time
            time_vel[1] = pose.twist.twist.linear.x
            vel_profile_1 = np.append(vel_profile_1, [time_vel], 0)
            time_position[0] = current_time - start_time
            time_position[1] = pose.pose.pose.position.x
            position_profile_1 = np.append(position_profile_1, [time_position], 0)
    
    

def animate_plot(i):

    # velocity profile plot
    x0 = list(vel_profile_0[:,0])
    x1 = list(vel_profile_1[:,0])
    y0 = list(vel_profile_0[:,1])
    y1 = list(vel_profile_1[:,1])

    # position profile plot
    x2 = list(position_profile_0[:,0])
    x3 = list(position_profile_1[:,0])
    y2 = list(position_profile_0[:,1])
    y3 = list(position_profile_1[:,1])
    

    ax1.clear()
    ax2.clear()

    line1, = ax1.plot(x0, y0, light_blue, label='car0_y_velocity')
    line2, = ax1.plot(x1, y1, light_orange, label='car1_x_velocity')
    line3, = ax2.plot(x2, y2, green_blue, label='car0_y_position')
    line4, = ax2.plot(x3, y3, light_yellow, label='car1_x_position')
    
    ax1.set_ylabel('velocity (m/s)')
    ax1.set_xlabel('time (s)')
    ax2.set_ylabel('position (m)')
    # to keep 0 of two axis aligned
    ax1.set_ylim(-6,6)
    ax2.set_ylim(-20,20)

    # dist to intersect difference
    # this is kind of a bug, the len() of two list might be different    
    if len(x2) > len(x3):
        diff = len(x2)-len(x3)
        y2 = y2[0:-diff]
        y4 = list(np.array(y3) - np.array(y2))
        line5, = ax2.plot(x3, y4, apple_green, label='dist_to_intersection_diff')
    

    elif len(x2) < len(x3) :
        diff = len(x3)-len(x2)
        y3 = y3[0:-diff]
        y4 = list(np.array(y3) - np.array(y2))
        line5, = ax2.plot(x2, y4, apple_green, label='dist_to_intersection_diff')
    
    elif len(x2) == len(x3) :
        y4 = list(np.array(y3) - np.array(y2))
        line5, = ax2.plot(x2, y4, apple_green, label='dist_to_intersection_diff')

    plt.legend(handles=[line1, line2, line3, line4, line5], loc='lower right')
    plt.title("Velocity and Position Profile in Simulation")



if __name__ == '__main__':

    # ROS subscriber Section
    rospy.init_node('rosbag_plot', anonymous=True)

    obs_list = ['car0', 'car1']
    for i in range(len(obs_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(obs_list[i]), Odometry,rosbag_callback,(i))


    # Plotting Section
    # figure_1
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    ax2 = ax1.twinx()  # share the same x axis and have different left and right y-axis
    ani = animation.FuncAnimation(fig, animate_plot, interval= update_rate)

    plt.show()


    
    while not rospy.is_shutdown():
        try:
            pass
            #print("shape of x:{0}".format(timestamp[:,0].shape))
            #print("shape of y0:{0}".format(velocity_profiles[:,0].shape))
        
        except rospy.ROSInterruptException:
            pass
    #rospy.Subscriber('/car0/base_pose_ground_truth', Odometry, rosbag_callback)
    #rospy.Subscriber('/car1/base_pose_ground_truth', Odometry, rosbag_callback)

