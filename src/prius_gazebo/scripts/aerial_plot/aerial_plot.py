#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm
#from pyexcel_ods import get_data
#data = get_data("/home/liuyc/moby_folder/ASME2019/data_analysis_python.ods")

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


car0_time_arr = np.array([])
car1_time_arr = np.array([])
car0_x_pose_arr = np.array([])
car0_y_pose_arr = np.array([])
car0_x_vel_arr = np.array([])
car0_y_vel_arr = np.array([])
car1_x_pose_arr = np.array([])
car1_y_pose_arr = np.array([])
car1_x_vel_arr = np.array([])
car1_y_vel_arr = np.array([])


#--- Parameters ---#

alpha = 0.001
slope = 0.7441   
STD = 0.09857   
a_dec = 2.44    # m/s^2
R_min = 10.4    # meter
tau = 0.6    # s
    

# GET car0 data

dir_name = "20190612_param_std"
file_name = "liuyc_prius1_2"

with open(r'/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/{1}_prius0'.format(dir_name, file_name), "r") as car0_original_file:
    #global time_arr, car0_x_pose_arr, car0_y_pose_arr

    #This should turn file contents into a list, by line, without \n
    car0_original_list  = car0_original_file.read().splitlines() 
    car0_first_timestamp = 0

    for row in car0_original_list[3:-1] : 
        #global first_timestamp
        # RESET time
        if row == car0_original_list[3]:
            car0_first_timestamp = float(row.split(",")[0])
            car0_time_arr = np.append(car0_time_arr, 0, )
        else : car0_time_arr = np.append(car0_time_arr, float(row.split(",")[0]) - car0_first_timestamp, )
        car0_x_pose_arr = np.append(car0_x_pose_arr, float(row.split(",")[1]), )
        car0_y_pose_arr = np.append(car0_y_pose_arr, float(row.split(",")[2]), )
        car0_x_vel_arr = np.append(car0_x_vel_arr, float(row.split(",")[3]), )
        car0_y_vel_arr = np.append(car0_y_vel_arr, float(row.split(",")[4]), )


# GET car1 data
with open(r'/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/{1}_prius1'.format(dir_name, file_name), "r") as car1_original_file:

    #This should turn file contents into a list, by line, without \n
    car1_original_list  = car1_original_file.read().splitlines() 
    car1_first_timestamp = 0

    for row in car1_original_list[3:-1] : 
        # RESET time
        if row == car1_original_list[3]:
            car1_first_timestamp = float(row.split(",")[0])
            car1_time_arr = np.append(car1_time_arr, 0, )
        else : car1_time_arr = np.append(car1_time_arr, float(row.split(",")[0]) - car1_first_timestamp, )
        car1_x_pose_arr = np.append(car1_x_pose_arr, float(row.split(",")[1]), )
        car1_y_pose_arr = np.append(car1_y_pose_arr, float(row.split(",")[2]), )
        car1_x_vel_arr = np.append(car1_x_vel_arr, float(row.split(",")[3]), )
        car1_y_vel_arr = np.append(car1_y_vel_arr, float(row.split(",")[4]), )

###-------------------------------###
###---Time to Action Estimation---###
###-------------------------------###

def CDF(ttc,v_car):

    #--- Get the estimated TTA ---#
    
    R_i = v_car**2 / (2 * a_dec)

    TTA_est = (R_i + v_car*tau + R_min ) / v_car

    TTA_act = TTA_est * slope   # mean of the PDF

    std = TTA_est * STD   # standard deviation of the PDF

    cdf = norm(TTA_act, std).cdf(ttc)
    
    print("ttc now is {0} where mu = {1}, std = {2}".format(ttc, TTA_act, std))
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


# average every n element in the list
# RETURN a new list
def average_every(dommy_list, n):
    
    rem = len(dommy_list) % n

    if rem == 0: pass
    else : dommy_list = dommy_list[:-rem]

    sum_arr = np.array([])

    for i in range(n):
        if i == 0:
            sum_arr = np.array(dommy_list[i::n])
        else : sum_arr += np.array(dommy_list[i::n])

    return list(sum_arr/n)



def plot_figs_together():

########### BEGIN OF PROCESSING DATA #############

    # CHECK which direction car* is moving
    dir_xy = [0,0]   # 0 for x-dir; 1 for y-dir; [car0, car1] 
    car0_x_displacement = abs(car0_x_pose_arr[-1] - car0_x_pose_arr[0])
    car0_y_displacement = abs(car0_y_pose_arr[-1] - car0_y_pose_arr[0])
    car1_x_displacement = abs(car1_x_pose_arr[-1] - car1_x_pose_arr[0])
    car1_y_displacement = abs(car1_y_pose_arr[-1] - car1_y_pose_arr[0])

    if car0_x_displacement > car0_y_displacement:
        dir_xy[0] = 0
    else : dir_xy[0] = 1

    if car1_x_displacement > car1_y_displacement:
        dir_xy[1] = 0
    else : dir_xy[1] = 1

    ###THIS IS ORIGINAL DATA
    # time stamp
    car0_t = list(car0_time_arr)
    car1_t = list(car1_time_arr)

    # position profile plot
    if not dir_xy[0] and dir_xy[1]:   # car0 in x-dir and car1 in y-dir
        car0_pose = list(car0_x_pose_arr)
        car1_pose = list(car1_y_pose_arr)
        car0_vel = map(abs, list(car0_x_vel_arr))
        car1_vel = map(abs, list(car1_y_vel_arr))
    elif dir_xy[0] and not dir_xy[1]:   # car0 in y-dir and car1 in x-dir
        car0_pose = list(car0_y_pose_arr)
        car1_pose = list(car1_x_pose_arr)
        car0_vel = map(abs, list(car0_y_vel_arr))
        car1_vel = map(abs, list(car1_x_vel_arr))
    else : 
        print("They have parallel pathes !")
    

    ### HERE we TRY to SMOTTHEN the VELOCITY curve

    every_num = 7

    
    car0_t = average_every(car0_t, every_num)
    car0_pose = average_every(car0_pose, every_num)
    car0_vel = average_every(car0_vel, every_num)
    car1_t = average_every(car1_t, every_num)
    car1_pose = average_every(car1_pose, every_num)
    car1_vel = average_every(car1_vel, every_num)


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
            car0_POS.append(POS(min(car0_t2n[:i+1]), car0_t2n[i+1], car0_t2n[i], car0_t[i+1], car0_t[i], car0_vel[i+1]))
            car0_t_POS.append(car0_t[i+1])

    if len(car1_t2n) > 2:
        for i in range(len(car1_t2n)-1):
            car1_POS.append(POS(min(car1_t2n[:i+1]), car1_t2n[i+1], car1_t2n[i], car1_t[i+1], car1_t[i], car1_vel[i+1]))
            car1_t_POS.append(car1_t[i+1])
    ##!!! append time (x value) in this loop, or x and y won't match


########### END OF PROCESSING DATA #############

    print(car0_t2n)

    fig1 = plt.figure(1, figsize=(12,18))
    ax11 = fig1.add_subplot(211)
    ax12 = ax11.twinx()  # share the same x axis and have different left and right y-axis
    ax21 = fig1.add_subplot(212)
    ax22 = ax21.twinx()  # share the same x axis and have different left and right y-axis

    ax11.clear()
    ax12.clear()
    ax21.clear()
    ax22.clear()

    
    if not dir_xy[0] and dir_xy[1]:   # car0 in x-dir and car1 in y-dir
        line11, = ax11.plot(car0_t, car0_vel, light_blue, label='car0_x_velocity')
        line12, = ax11.plot(car1_t, car1_vel, light_orange, label='car1_y_velocity')
        line21, = ax21.plot(car0_t, car0_pose, light_blue, label='car0_x_position')
        line22, = ax21.plot(car1_t, car1_pose, light_orange, label='car1_y_position')
    elif dir_xy[0] and not dir_xy[1]:   # car0 in y-dir and car1 in x-dir
        line11, = ax11.plot(car0_t, car0_vel, light_blue, label='car0_y_velocity')
        line12, = ax11.plot(car1_t, car1_vel, light_orange, label='car1_x_velocity')
        line21, = ax21.plot(car0_t, car0_pose, light_blue, label='car0_x_position')
        line22, = ax21.plot(car1_t, car1_pose, light_orange, label='car1_y_position')
    else : 
        print("They have parallel pathes !")
    
    line13, = ax12.plot(car0_t_POS, car0_POS, green_blue, label='car0_POS')
    line14, = ax12.plot(car1_t_POS, car1_POS, light_yellow, label='car1_POS')
    line23, = ax22.plot(car0_t_POS, car0_POS, green_blue, label='car0_POS')
    line24, = ax22.plot(car1_t_POS, car1_POS, light_yellow, label='car1_POS')
    
    ax11.set_ylabel('velocity (m/s)')
    ax11.set_xlabel('time (s)')
    ax12.set_ylabel('probability of stopping')
    ax21.set_ylabel('position (m')
    ax21.set_xlabel('time (s)')
    ax22.set_ylabel('probability of stopping')
    # to keep 0 of two axis aligned
    ax11.set_ylim(-6,6)
    ax12.set_ylim(-0.2, max(car0_POS+car1_POS)+0.2)
    ax21.set_ylim(-30,30)
    ax22.set_ylim(-0.2, max(car0_POS+car1_POS)+0.2)
    
    handles1=[line11, line12, line13, line14]
    labels1 = [h.get_label() for h in handles1]
    ax11.legend(handles=handles1, labels=labels1, loc='lower right')
    ax11.title.set_text("POS and Velocity Profile at real crossroads")

    handles2=[line21, line22, line23, line24]
    labels2 = [h.get_label() for h in handles2]
    ax21.legend(handles=handles2, labels=labels2, loc='lower right')
    ax21.title.set_text("POS and Position Profile at real crossroads")

    #plt.title("POS to Velocity and Position Profile at real crossroads")
    plt.show()

if __name__ == '__main__':


    plot_figs_together()
