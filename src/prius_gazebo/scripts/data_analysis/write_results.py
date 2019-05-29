#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm


#--- Parameters ---#

alpha = 0.3
R_min = 3    # meter
tau = 0.6    # s
a_dec = 3.4    # m/s^2
slope = 0.65   # y = 0.65x + 0.15

###-------------------------------###
###---Time to Action Estimation---###
###-------------------------------###

def CDF(ttc,v_car):

    #--- Get the estimated TTA ---#
    
    R_i = v_car**2 / (2 * a_dec)

    TTA_est = (R_i + v_car*tau + R_min ) / v_car

    TTA_act = TTA_est / slope   # mean of the PDF

    std = TTA_act * 0.375/2    # standard deviation of the PDF

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
    judge_p_stop = (1 - cdf_0) * gamma

    if judge_p_stop > 1 :
        p_stop = 1
    else:
        p_stop = judge_p_stop

    return p_stop



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


#######################################################
#######################################################
with open(r"/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/CAR_results_123", "a") as f:

    
    def write_results():

        for i in range(10):
            L = ["now we are at {0}\n".format(i), "hehehehhehe\n"]
            f.writelines(L) 

    write_results()

if __name__ == '__main__':
    pass
