#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm


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
car_dirs=np.array([[0,0],[0,0]])




def rosbag_callback(pose, i):
    global vel_profile_0, vel_profile_1, count, start_time, position_profile_0, position_profile_1, car_dirs

    car_vel = np.ones([2,2])
    car_vel[i][0] = abs(pose.twist.twist.linear.x)
    car_vel[i][1] = abs(pose.twist.twist.linear.y)

    # determine the heading (x or y) of the car
    ### NOTE: Now ONLY x and y direction are considered
    ### TODO: Should use vector instead
    def dir_xy(car_num = 0):  # now we control only one car

        last_dir=[0,0]
        
        if car_vel[car_num][0] > car_vel[car_num][1] :   # value of twist.linear.x is greater than that in y
            last_dir = [1,0]
            car_dirs[car_num] = [1,0]

        elif car_vel[car_num][1] > car_vel[car_num][0] :   # value of twist.linear.y is greater than that in x
            last_dir = [0,1]
            car_dirs[car_num] = [0,1]

        return last_dir   # [0,0] for not moving


    time_vel=[0.0, 0.0]
    time_position = [0.0, 0.0]

    current_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9

    # Plot from the moment they start to move
    if abs(pose.twist.twist.linear.y) > 0.1 or abs(pose.twist.twist.linear.x) > 0.1:
        

        if count == 0:
            start_time = current_time
            count += 1

        if i == 0:  # car0
            time_vel[0] = current_time - start_time
            time_position[0] = current_time - start_time
            dir_check = dir_xy(0)
            if dir_check[0]:   # moving in x direction
                time_vel[1] = pose.twist.twist.linear.x
                time_position[1] = pose.pose.pose.position.x
            else:   # moving in y direction
                time_vel[1] = pose.twist.twist.linear.y
                time_position[1] = pose.pose.pose.position.y
            vel_profile_0 = np.append(vel_profile_0, [time_vel], 0)
            position_profile_0 = np.append(position_profile_0, [time_position], 0)

        elif i == 1: # car1
            time_vel[0] = current_time - start_time
            time_position[0] = current_time - start_time
            dir_check = dir_xy(1)
            if dir_check[0]:   # moving in x direction
                time_vel[1] = pose.twist.twist.linear.x
                time_position[1] = pose.pose.pose.position.x
            else:   # moving in y direction
                time_vel[1] = pose.twist.twist.linear.y
                time_position[1] = pose.pose.pose.position.y
            vel_profile_1 = np.append(vel_profile_1, [time_vel], 0)
            position_profile_1 = np.append(position_profile_1, [time_position], 0)
        # end when either one reaches the node ([0,0])
        # it's cheating now, car0 has to move along y-axis and car1 along x-axis
        if i == 0 and abs(pose.pose.pose.position.y) < 0.1:
            rospy.signal_shutdown('Node reached by car0')

        elif i == 1 and abs(pose.pose.pose.position.x) < 0.1:
            rospy.signal_shutdown('Node reached by car1')


#--- Parameters ---#

alpha = 0.3
R_min = 5    # meter
tau = 0.6    # s
a_dec = 6    # m/s^2
slope = 0.65   # y = 0.65x + 0.15
    

###-------------------------------###
###---Time to Action Estimation---###
###-------------------------------###

def CDF(ttc,v_car):

    #--- Get the estimated TTA ---#
    
    R_i = v_car**2 / (2 * a_dec)

    TTA_est = (R_i + v_car*tau + R_min ) / v_car

    TTA_act = TTA_est / slope   # mean of the PDF

    std = TTA_act * 0.375/2   # standard deviation of the PDF

    cdf = norm(TTA_act, std).cdf(ttc)
    
    return cdf
    
###----------------------------------- ###
###------Probability of Stopping------ ###
###----------------------------------- ###


def POS(min_TTC, TTC, TTC_p, time, time_p, v_car):
    

    #--- Variables ---#

    TTC_dif = (TTC - TTC_p) / (time - time_p)


    #--- gamma ---#
    if (TTC_dif + 1) < 0:
        
        gamma = 0

    else:
        gamma = (TTC_dif + 1) * alpha

    
    #--- Probability of Stopping ---#
    cdf_0 = CDF(min_TTC, v_car)
    judge_p_stop = (1 - cdf_0) * gamma

    if judge_p_stop > 1 :

        p_stop = 1

    else:
        p_stop = judge_p_stop


    return p_stop



def animate_plot(i):

    # velocity profile plot
    car0_t = list(vel_profile_0[:,0])
    car1_t = list(vel_profile_1[:,0])
    car0_vel = map(abs, list(vel_profile_0[:,1]))
    car1_vel = map(abs, list(vel_profile_1[:,1]))

    # position profile plot
    #x2 = list(position_profile_0[:,0])
    #x3 = list(position_profile_1[:,0])
    car0_pose = list(position_profile_0[:,1])
    car1_pose = list(position_profile_1[:,1])
    
    # time to node plot
    car0_t2n = list(abs(np.array(car0_pose)/np.array(car0_vel))) 
    car1_t2n = list(abs(np.array(car1_pose)/np.array(car1_vel))) 

    # POS
    car0_POS = []
    car1_POS = []
    car0_t_POS = []
    car1_t_POS = []

    
    if len(car0_t2n) > 2:
        for i in range(len(car0_t2n)-1):
            car0_POS.append(POS(min(car0_t2n), car0_t2n[i+1], car0_t2n[i], car0_t[i+1], car0_t[i], car0_vel[i+1]))
            car0_t_POS.append(car0_t[i+1])

    if len(car1_t2n) > 2:
        for i in range(len(car1_t2n)-1):
            car1_POS.append(POS(min(car1_t2n), car1_t2n[i+1], car1_t2n[i], car1_t[i+1], car1_t[i], car1_vel[i+1]))
            car1_t_POS.append(car1_t[i+1])
    ##!!! append time (x value) in this loop, or x and y won't match

    ax1.clear()
    ax2.clear()


    if car_dirs[0][0]:   # [1,0] if car0 moving in x-dir
        line1, = ax1.plot(car0_t, car0_pose, light_blue, label='car0_x_position')
        line2, = ax1.plot(car1_t, car1_pose, light_orange, label='car1_y_position')
    else:   # [0,1] or [0,0] if car0 moving in y-dir or not moving
        line1, = ax1.plot(car0_t, car0_pose, light_blue, label='car0_y_position')
        line2, = ax1.plot(car1_t, car1_pose, light_orange, label='car1_x_position')
    
    
    line3, = ax2.plot(car0_t_POS, car0_POS, green_blue, label='car0_POS')
    line4, = ax2.plot(car1_t_POS, car1_POS, light_yellow, label='car1_POS')

    ax1.set_ylabel('position (m)')
    ax1.set_xlabel('time (s)')
    ax2.set_ylabel('probability of stopping')
    # to keep 0 of two axis aligned
    ax1.set_ylim(-30,30)
    ax2.set_ylim(-0.2,1.2)




    plt.legend(handles=[line1, line2, line3, line4], loc='lower right')
    #plt.legend(handles=[line1, line2, line4], loc='lower right')
    plt.title("POS and Position Profile in Simulation")



if __name__ == '__main__':

    # ROS subscriber Section
    rospy.init_node('rosbag_plot', anonymous=True)

    obs_list = ['prius0', 'prius1']
    for i in range(len(obs_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(obs_list[i]), Odometry,rosbag_callback,(i))


    # Plotting Section
    # figure_1
    fig = plt.figure(figsize=(16,9))
    ax1 = fig.add_subplot(1,1,1)
    ax2 = ax1.twinx()  # share the same x axis and have different left and right y-axis
    ani = animation.FuncAnimation(fig, animate_plot, interval= update_rate)

    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        ani.save('/home/liuyc/moby_folder/Research/figures/intersection_simulator/gif/lukc_liuyc_4_posePOS.gif', writer='imagemagick', fps=4)

    else:
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

