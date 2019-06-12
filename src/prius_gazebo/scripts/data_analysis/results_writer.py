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


time_start = time.time()

class TxtData:
    
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
        std = self.STD    # standard deviation of the PDF
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
 #       print("t_minus is {0}".format(t_minus))
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




if __name__ == '__main__':

##SETING01##
    dir_name = "20190612_param_std"
    path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*prius*'.format(dir_name)


    files = glob.glob(path)

# File seperated by names
    file_list = []
    for name in files:
        file_name_list = name.split('/')[-1].split("_")
        file_name = file_name_list[0]+"_"+file_name_list[1]
        if len(file_list) == 0:
            file_list.append(file_name)
        else:
            check = 0   # 1 need to add
            for j in file_list:
                if j == file_name:
                    check = 0
                    break
                else:
                    check = 1
            if check == 1 : file_list.append(file_name)


    real_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/'.format(dir_name)

# CAR classification

##SETING02##
    param_dict_0 = {'ALPHA':0.2, 'SLOPE':0.744, 'STD':0.09857, 'A_DEC':2.44, 'R_MIN':6.4, 'TAU':0.6}   # LiuYC's params
    #param_dict_0 = {'ALPHA':0.2, 'SLOPE':0.621, 'STD':0.058, 'A_DEC':1.956, 'R_MIN':7.23, 'TAU':0.6}   # WuCH's params
    #param_dict_1 = param_dict_0
    #param_dict_1 = {'ALPHA':.2, 'SLOPE':0.744, 'STD':0.09857, 'A_DEC':2.44, 'R_MIN':6.4, 'TAU':0.6}

# Loop for different params (sensitivity test)

    param_num_dict = {'ALPHA':0, 'SLOPE':1, 'STD':2, 'A_DEC':3, 'R_MIN':4, 'TAU':5}   # LiuYC's params
    ALPHA_list = np.arange(0.1, 2.3, 0.2)    #arange(upper, lower, step)
    SLOPE_list = np.arange(0.1, 1.2, 0.1)    #arange(upper, lower, step)
    STD_list = np.arange(0.01, 0.032, 0.002)    #arange(upper, lower, step)
    A_DEC_list = np.arange(1.0, 8.7, 0.7)    #arange(upper, lower, step)
    R_MIN_list = np.arange(2.0, 10.8, 0.8)    #arange(upper, lower, step)
    TAU_list = np.arange(0.1, 1.2, 0.1)    #arange(upper, lower, step)

##SETING03##
    set_param = "A_DEC"
    FILE_NUM = 20
    final_ods_list = [] 
    final_ods_list_0 = [] 
    final_ods_list_1 = [] 

    if set_param == "ALPHA" : temp_list = ALPHA_list
    elif set_param == "SLOPE" : temp_list = SLOPE_list
    elif set_param == "STD" : temp_list = STD_list
    elif set_param == "A_DEC" : temp_list = A_DEC_list
    elif set_param == "R_MIN" : temp_list = R_MIN_list
    elif set_param == "TAU" : temp_list = TAU_list


    for param in temp_list:

        param_dict_0[set_param] = param
        param_dict_1 = param_dict_0

        final_CAR_list_0 = []
        final_CAR_list_1 = []

        for driver_names in file_list:
        # Default as 20 files (e.g. liuyc_lukc_1_prius*)
            for i in range(FILE_NUM):   

                print("Processing {0}_{1} using {2}={3}.....".format(driver_names, i+1, set_param, param))

##SETING04##
            # prius01 
                prius0 = TxtData(param_dict_0)
                prius0_temp_d2n = 0
                path0 = real_path + driver_names +"_"+"{0}".format(i+1)+"_"+"prius0"
                with open(path0, "r") as f:
                # Turn file contents into a list, by line, without \n
                    raw_list = f.read().splitlines()
                # Update the arrays
                    prius0.update_arrays(raw_list)
                # Get the final lists (x or y)
                    prius0.get_final_list()
                # Get the last distance
                    prius0_temp_d2n = abs(prius0.car_pose_list[-1])
                # Reset (not needed in a for loop)
                #    prius0.reset_arrays()


            # prius1    
                prius1 = TxtData(param_dict_1)
                prius1_temp_d2n = 0
                path1 = real_path + driver_names +"_"+"{0}".format(i+1)+"_"+"prius1"
                with open(path1, "r") as f:
                # Turn file contents into a list, by line, without \n
                    raw_list = f.read().splitlines()
                # Update the arrays
                    prius1.update_arrays(raw_list)
                # Get the final lists (x or y)
                    prius1.get_final_list()
                # Get the last distance
                    prius1_temp_d2n = abs(prius1.car_pose_list[-1])
                # Reset (not needed in a for loop)
                #    prius1.reset_arrays()


                prius0_file_name = path0.split('/')[-1]
                prius1_file_name = path1.split('/')[-1]

                if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                    prius0.CAR_yield_analysis(prius0_file_name)
                    prius1.CAR_pass_analysis(prius1_file_name)

                elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                    prius0.CAR_pass_analysis(prius0_file_name)
                    prius1.CAR_yield_analysis(prius1_file_name)

                final_CAR_list_0.append(prius0.ods_sub_data[0])
                final_CAR_list_1.append(prius1.ods_sub_data[0])

                print("{0}_{1} CAR analysis done.\n".format(driver_names, i+1))


    # Add the sumation of final_CAR_list_0 into itself
        # copy the list (list.copy() available in 3.3)
        print("Adding {0} prius0 CAR results together.".format(FILE_NUM))
        dummy_CAR_list_0 = final_CAR_list_0[:]
        summed_CAR_list_0 = []
        for i in final_CAR_list_0:
            i = i[1:]    # remove the name tag
            summed_CAR_list_0 = map(sum,itertools.izip_longest(summed_CAR_list_0, i, fillvalue = 0))   # list has different length

        summed_CAR_list_0 = list(np.array(summed_CAR_list_0) / float(FILE_NUM))
        summed_CAR_list_0 = [float(j) for j in summed_CAR_list_0]
    
        summed_CAR_list_0.insert(0, "prius0_{0}-{1}-{2}-{3}-{4}-{5}".format(param_dict_0['ALPHA'], param_dict_0['SLOPE'], param_dict_0['STD'], param_dict_0['A_DEC'], param_dict_0['R_MIN'], param_dict_0['TAU']))
        #####final_CAR_list_0.append(summed_CAR_list_0)

    # Add the sumation of final_CAR_list_1 into itself
        # copy the list (list.copy() available in 3.3)
        print("Adding {0} prius1 CAR results together.".format(FILE_NUM))
        dummy_CAR_list_1 = final_CAR_list_1[:]
        summed_CAR_list_1 = []
        for i in final_CAR_list_1:
            i = i[1:]    # remove the name tag
            summed_CAR_list_1 = map(sum,itertools.izip_longest(summed_CAR_list_1, i, fillvalue = 0))   # list has different length

        summed_CAR_list_1 = list(np.array(summed_CAR_list_1) / float(FILE_NUM))
        summed_CAR_list_1 = [float(j) for j in summed_CAR_list_1]
    
        summed_CAR_list_1.insert(0, "prius1_{0}-{1}-{2}-{3}-{4}-{5}".format(param_dict_1['ALPHA'], param_dict_1['SLOPE'], param_dict_1['STD'], param_dict_1['A_DEC'], param_dict_1['R_MIN'], param_dict_1['TAU']))
        #####final_CAR_list_0.append(summed_CAR_list_0)

        final_ods_list_0.append(summed_CAR_list_0)
        final_ods_list_1.append(summed_CAR_list_1)
        print("Done adding CAR results together.")

    
    final_ods_list = final_ods_list_0 + final_ods_list_1 

    print("Opening file CAR_results.ods .")

    data = get_data("CAR_results.ods")
    #####data["liuyc_{2}_{1}_test{0}".format(dir_name.split('_')[1:], param, set_param)] = final_CAR_list
    data["liuyc_{0}_sensitivity_test".format(set_param)] = final_ods_list
    print("Saving param {0} to ods file.".format(set_param))
    save_data("CAR_results.ods", data)
    print("Done saving to ods file.")
    print("The whole process takes {0:.2f} seconds.".format(time.time()-time_start))

