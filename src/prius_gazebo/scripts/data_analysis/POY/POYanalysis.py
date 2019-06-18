#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm
import glob    # To get a list of all files in a directory
import errno    
from collections import OrderedDict
from pyexcel_ods import save_data    # Dave data to .ods file for Calc
from pyexcel_ods import get_data    # Dave data to .ods file for Calc
import itertools
import time



class DataAnalysis:
    
    def __init__(self, param_dict={'ALPHA':0.2, 'SLOPE':0.744, 'STD':0.09857, 'A_DEC':2.44, 'R_MIN':6.4, 'TAU':0.6}):

    #--- POS, CDF Parameters ---#
        self.ALPHA = param_dict['ALPHA']
        self.SLOPE = param_dict['SLOPE']   
        self.STD = param_dict['STD']   
        self.A_DEC = param_dict['A_DEC']    # m/s^2
        self.R_MIN = param_dict['R_MIN']    # meter
        self.TAU = param_dict['TAU']    # s

    # Analysis Parameters
        self.VISIBLE_DIST = 12    # (m)
        self.POS_YIELD_THRESH = 0.8
        self.POS_PASS_THRESH = 0.5

    # Analysis List
        self.ods_data = OrderedDict()
        self.ods_sub_data = []
        self.sum_list = []

    # Original data
        self.first_timestamp = 0.0
        self.time_arr = np.array([])
        self.x_pose_arr = np.array([])
        self.y_pose_arr = np.array([])
        self.x_vel_arr = np.array([])
        self.y_vel_arr = np.array([])
    # Final list
        self.car_t_list = []
        self.car_vel_list = []
        self.car_pose_list = []
        self.car_t2n_list = []
        self.car_POS_list = []
        self.car_POS_t_list = []

    # dir_xy = [x-dir, y-dir] 1 as True, 0 as False
        self.dir_xy = [0,0]   


    def reset_arrays(self):

        # Original data
        self.first_timestamp = 0.0
        self.time_arr = np.array([])
        self.x_pose_arr = np.array([])
        self.y_pose_arr = np.array([])
        self.x_vel_arr = np.array([])
        self.y_vel_arr = np.array([])
        # Final list
        self.car_t_list = []
        self.car_vel_list = []
        self.car_pose_list = []
        self.car_t2n_list = []
        self.car_POS_list = []
        self.car_POS_t_list = []

        # dir_xy = [x-dir, y-dir] 1 as True, 0 as False
        self.dir_xy = [0,0]   


    ###-------------------------------###
    ###---Time to Action Estimation---###
    ###-------------------------------###
    def CDF(self, TTC, v_car):

        #--- Get the estimated TTA ---#
        R_I = v_car**2 / (2 * self.A_DEC)
        TTA_est = (R_I + v_car*self.TAU + self.R_MIN ) / v_car
        TTA_act = TTA_est * self.SLOPE   # mean of the PDF
        std = TTA_est * self.STD    # standard deviation of the PDF
        cdf = norm(TTA_act, std).cdf(TTC)
        
        return cdf
    

    ###----------------------------------- ###
    ###------Probability of Stopping------ ###
    ###----------------------------------- ###
    def POS(self, min_TTC, TTC, TTC_p, time, time_p, v_car):
        
        #--- Variables ---#
        TTC_dif = (TTC - TTC_p) / (time - time_p)

        #--- GAMMA ---#
        if (TTC_dif + 1) < 0:
            self.GAMMA = 0
        else:
            self.GAMMA = (TTC_dif + 1) * self.ALPHA

        #--- Probability of Stopping ---#
        cdf_0 = self.CDF(min_TTC, v_car)
        judge_p_stop = (1 - cdf_0) * self.GAMMA
        if judge_p_stop > 1 :
            p_stop = 1
        else:
            p_stop = judge_p_stop

        return p_stop


    # average every n element in the list
    # RETURN a new list
    def average_every(self, dommy_list, n):
        
        rem = len(dommy_list) % n

        if rem == 0: pass
        else : dommy_list = dommy_list[:-rem]

        sum_arr = np.array([])

        for i in range(n):
            if i == 0:
                sum_arr = np.array(dommy_list[i::n])
            else : sum_arr += np.array(dommy_list[i::n])

        return list(sum_arr/n)


    def update_arrays(self, original_list):

        for row in original_list[3:-1] : 

            # SET time
            if row == original_list[3]:   # first row with data
                self.first_timestamp = float(row.split(",")[0])
                self.time_arr = np.append(self.time_arr, 0, )
            else : 
                self.time_arr = np.append(self.time_arr, float(row.split(",")[0]) - self.first_timestamp, )
            
            # SET pose and vel
            self.x_pose_arr = np.append(self.x_pose_arr, float(row.split(",")[1]), )
            self.y_pose_arr = np.append(self.y_pose_arr, float(row.split(",")[2]), )
            self.x_vel_arr = np.append(self.x_vel_arr, float(row.split(",")[3]), )
            self.y_vel_arr = np.append(self.y_vel_arr, float(row.split(",")[4]), )


    def get_final_list(self, every_num=1):

    # CHECK which direction car* is moving
        x_displacement = abs(self.x_pose_arr[-1] - self.x_pose_arr[0])
        y_displacement = abs(self.y_pose_arr[-1] - self.y_pose_arr[0])
        if x_displacement > y_displacement : self.dir_xy[0] = 1
        else : self.dir_xy[1] = 1

    # Get FINAL data
        # time stamp
        self.car_t_list = list(self.time_arr)

        # position profile plot
        if self.dir_xy[0] and not self.dir_xy[1]:   # car move in x-dir 
            self.car_pose_list = list(self.x_pose_arr)
            self.car_vel_list = map(abs, list(self.x_vel_arr))
        elif self.dir_xy[1] and not self.dir_xy[0]:   # car move in y-dir 
            self.car_pose_list = list(self.y_pose_arr)
            self.car_vel_list = map(abs, list(self.y_vel_arr))
        else : 
            print("This car is not moving ! {0}".format(self.dir_xy))

#### PRE-PROCESS #### 

    # Start counting from where the ego car can see the coming car
        for p in range(len(self.car_pose_list)):
            if abs(self.car_pose_list[p]) <= self.VISIBLE_DIST:
                self.car_t_list = self.car_t_list[p:]
                self.car_vel_list = self.car_vel_list[p:]
                self.car_pose_list = self.car_pose_list[p:]
                break

    # HERE we TRY to SMOTTHEN the VELOCITY curve
        self.car_t_list = self.average_every(self.car_t_list, every_num)
        self.car_vel_list = self.average_every(self.car_vel_list, every_num)
        self.car_pose_list = self.average_every(self.car_pose_list, every_num)

    # Get time to node
        self.car_t2n_list = list(abs(np.array(self.car_pose_list)/np.array(self.car_vel_list))) 

    # Probability of Stopping (yielding)
        self.car_POS_list = []
        self.car_POS_t_list = []

        # append time (x value) in this loop, or x and y might not match
        if len(self.car_t2n_list) > 2:
            for i in range(len(self.car_t2n_list)-1):
                self.car_POS_list.append(self.POS(min(self.car_t2n_list[:i+1]), self.car_t2n_list[i+1], self.car_t2n_list[i], self.car_t_list[i+1], self.car_t_list[i], self.car_vel_list[i+1]))
                self.car_POS_t_list.append(self.car_t_list[i+1])



    # every_num=n : average every n data
    def CAR_yield_analysis(self, file_name):
#### COUNTING CAR ####
    # Turn time into t-minus (copy one, avoide changing on the original list )
        # copy the list (list.copy() available in 3.3)
        time_line = self.car_POS_t_list[:]
        t_minus = time_line[::-1]   # reverse the list
        t_minus = list(np.array(t_minus) - t_minus[0])

        POS_list = self.car_POS_list[:]
        POS_list = POS_list[::-1]
        current_list = [] 
        current_list.append(file_name)
        for i in range(len(POS_list)):
            if POS_list[i] >= self.POS_YIELD_THRESH:
                current_list.append(1)
            else:
                current_list.append(0)

        self.ods_sub_data.append(current_list)  


    # every_num=n : average every n data
    def CAR_pass_analysis(self, file_name):
#### COUNTING CAR ####
    # Turn time into t-minus (copy one, avoide changing on the original list )
        # copy the list (list.copy() available in 3.3)
        time_line = self.car_POS_t_list[:]
        t_minus = time_line[::-1]   # reverse the list
        '''
        print("t_minus is {0}".format(t_minus))
        print("car_POS_t_list is {0}".format(self.car_POS_t_list))
        print("car_t_list is {0}".format(self.car_t_list))
        print("time_arr is {0}".format(self.time_arr))
        '''
        t_minus = list(np.array(t_minus) - t_minus[0])

        POS_list = self.car_POS_list[:]
        POS_list = POS_list[::-1]
        current_list = [] 
        current_list.append(file_name)
        for i in range(len(POS_list)):
            if POS_list[i] <= self.POS_PASS_THRESH:
                current_list.append(1)
            else:
                current_list.append(0)

        self.ods_sub_data.append(current_list)  


    def get_final_pose(self, path):

        temp_d2n = 0

        with open(path, "r") as f:
        # Turn file contents into a list, by line, without \n
            raw_list = f.read().splitlines()
        # Update the arrays
            self.update_arrays(raw_list)
        # Get the final lists (x or y)
            self.get_final_list()
        # Get the last distance
            temp_d2n = abs(self.car_pose_list[-1])

        return temp_d2n

