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

update_rate = 10  # in millisec
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
        if i == 0 and abs(pose.pose.pose.position.y) < 0.1 or abs(pose.pose.pose.position.x) < 0.1:
            rospy.signal_shutdown('Node reached by car0')

        elif i == 1 and abs(pose.pose.pose.position.x) < 0.1 or abs(pose.pose.pose.position.y) < 0.1:
            rospy.signal_shutdown('Node reached by car1')


#--- Parameters ---#

alpha = 1
R_min = 7.745    # meter
tau = 0.6    # s
a_dec = 4.475    # m/s^2
slope = 0.778   # y = 0.65x + 0.15
STD = 0.081
    

###-------------------------------###
###---Time to Action Estimation---###
###-------------------------------###

def CDF(ttc,v_car):

    #--- Get the estimated TTA ---#
    
    R_i = v_car**2 / (2 * a_dec)

    TTA_est = (R_i + v_car*tau + R_min ) / v_car

    TTA_act = TTA_est * slope   # mean of the PDF

    std = TTA_act * STD    # standard deviation of the PDF

    cdf = norm(TTA_act, std).cdf(ttc)
    #print("TTA_act = {0} while std = {1}".format(TTA_act, std))
    
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
    #print("cdf is {0} when min_TTC is {1} and car_vel {2}".format(cdf_0, min_TTC, v_car))
    judge_p_stop = (1 - cdf_0) * gamma

    '''
    if judge_p_stop > 1 :

        p_stop = 1

    else:
        p_stop = judge_p_stop
    '''
    p_stop = judge_p_stop

    return p_stop



## Plotting Section

fig1 = plt.figure(1, figsize=(12,18))
ax11 = fig1.add_subplot(211)
ax12 = ax11.twinx()  # share the same x axis and have different left and right y-axis
ax21 = fig1.add_subplot(212)
ax22 = ax21.twinx()  # share the same x axis and have different left and right y-axis

line11, = ax11.plot([], [], light_blue, label='car0_y_velocity')
line12, = ax11.plot([], [], light_orange, label='car1_x_velocity')
line13, = ax12.plot([], [], green_blue, label='car0_POS')
line14, = ax12.plot([], [], light_yellow, label='car1_POS')
line21, = ax21.plot([], [], light_blue, label='car0_x_position')
line22, = ax21.plot([], [], light_orange, label='car1_y_position')
line23, = ax22.plot([], [], green_blue, label='car0_POS')
line24, = ax22.plot([], [], light_yellow, label='car1_POS')

ax11.set_ylabel('velocity (m/s)')
ax11.set_xlabel('time (s)')
ax12.set_ylabel('probability of stopping')
ax21.set_ylabel('position (m')
ax21.set_xlabel('time (s)')
ax22.set_ylabel('probability of stopping')
# to keep 0 of two axis aligned
ax11.set_xlim(0,8)
ax11.set_ylim(-6,6)
ax12.set_ylim(-0.2, 1.2)
#ax12.set_ylim(-0.2, max(car1_POS+car0_POS))
ax21.set_xlim(0,8)
ax21.set_ylim(-30,30)
ax22.set_ylim(-0.2, 1.2)
#ax22.set_ylim(-0.2, max(car1_POS+car0_POS))

handles1=[line11, line12, line13, line14]
labels1 = [h.get_label() for h in handles1]
ax11.legend(handles=handles1, labels=labels1, loc='lower right')
ax11.title.set_text("POS and Velocity Profile at real crossroads")

handles2=[line21, line22, line23, line24]
labels2 = [h.get_label() for h in handles2]
ax21.legend(handles=handles2, labels=labels2, loc='lower right')
ax21.title.set_text("POS and Position Profile at real crossroads")



def animate_plot(i):

########### BEGIN OF PROCESSING DATA #############


    ###THIS IS ORIGINAL DATA
    # time stamp
    car0_t = list(vel_profile_0[:,0])
    car1_t = list(vel_profile_1[:,0])
    car0_vel = map(abs, list(vel_profile_0[:,1]))
    car1_vel = map(abs, list(vel_profile_1[:,1]))
    car0_pose = list(position_profile_0[:,1])
    car1_pose = list(position_profile_1[:,1])


    '''
    ### HERE we TRY to SMOTTHEN the VELOCITY curve

    every_num = 7

    print("before fun. {0}".format(car0_t))
    
    car0_t = average_every(car0_t, every_num)
    print("after fun. {0}".format(car0_t))
    car0_pose = average_every(car0_pose, every_num)
    car0_vel = average_every(car0_vel, every_num)
    car1_t = average_every(car1_t, every_num)
    car1_pose = average_every(car1_pose, every_num)
    car1_vel = average_every(car1_vel, every_num)
    '''

    # time to node plot
    car0_t2n = list(abs(np.array(car0_pose)/np.array(car0_vel))) 
    car1_t2n = list(abs(np.array(car1_pose)/np.array(car1_vel))) 

    # POS
    car0_POS = [0]
    car1_POS = [0]
    car0_t_POS = [0]
    car1_t_POS = [0]

    if len(car0_t2n) > 2:
        for i in range(len(car0_t2n)-1):
            car0_POS.append(POS(min(car0_t2n[:i+1]), car0_t2n[i+1], car0_t2n[i], car0_t[i+1], car0_t[i], car0_vel[i+1]))
            car0_t_POS.append(car0_t[i+1])

    if len(car1_t2n) > 2:
        for i in range(len(car1_t2n)-1):
            car1_POS.append(POS(min(car1_t2n[:i+1]), car1_t2n[i+1], car1_t2n[i], car1_t[i+1], car1_t[i], car1_vel[i+1]))
            car1_t_POS.append(car1_t[i+1])
    ##!!! append time (x value) in this loop, or x and y won't match


########### END OF PROCESSING DATA #############

    
    if car_dirs[0][0]:   # [1,0] if car0 moving in x-dir
        line11.set_data(car0_t, car0_vel)
        line12.set_data(car1_t, car1_vel) 
        line21.set_data(car0_t, car0_pose)
        line22.set_data(car1_t, car1_pose)
    else:
        line11.set_data(car0_t, car0_vel)
        line12.set_data(car1_t, car1_vel)
        line21.set_data(car0_t, car0_pose)
        line22.set_data(car1_t, car1_pose)
    
    line13.set_data(car0_t_POS, car0_POS)
    line14.set_data(car1_t_POS, car1_POS)
    line23.set_data(car0_t_POS, car0_POS)
    line24.set_data(car1_t_POS, car1_POS)
    
    return line11, line12, line13, line14, line21, line22, line23, line24
    #plt.title("POS to Velocity and Position Profile at real crossroads")



if __name__ == '__main__':

    # ROS subscriber Section
    rospy.init_node('rosbag_plot', anonymous=True)

    obs_list = ['prius0', 'prius1']
    for i in range(len(obs_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(obs_list[i]), Odometry,rosbag_callback,(i))


    ani = animation.FuncAnimation(fig1, animate_plot, frames=np.arange(800), interval= update_rate)

    # sys.argv > 1 means at least one parameter is passed
    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        ani.save('/home/liuyc/moby_folder/Research/figures/intersection_simulator/gif/nothing.gif', writer='imagemagick', fps=30)
    # plt.show() will loop animation forever    
    else : 
        plt.show()
    
    while not rospy.is_shutdown():
        try:
            pass
        
        except rospy.ROSInterruptException:
            pass

