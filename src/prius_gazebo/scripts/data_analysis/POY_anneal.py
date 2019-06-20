#!/usr/bin/env python
from __future__ import print_function
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
from POY.AnnealPOYanalysis import DataAnalysis
import math
import random
from simanneal import Annealer


POS_YIELD_THRESH = 0.8
POS_PASS_THRESH = 0.5


def CAR_yield_analysis(car_POS_list):
#### COUNTING CAR ####
# Turn time into t-minus (copy one, avoide changing on the original list )
    # copy the list (list.copy() available in 3.3)
    POS_list = car_POS_list[:]
    POS_list = POS_list[::-1]
    current_list = [] 
    for i in range(len(POS_list)):
        if POS_list[i] >= POS_YIELD_THRESH:
            current_list.append(1)
        else:
            current_list.append(0)

    return current_list


def CAR_pass_analysis(car_POS_list):
#### COUNTING CAR ####
# Turn time into t-minus (copy one, avoide changing on the original list )
    # copy the list (list.copy() available in 3.3)
    POS_list = car_POS_list[:]
    POS_list = POS_list[::-1]
    current_list = [] 
    for i in range(len(POS_list)):
        if POS_list[i] <= POS_PASS_THRESH:
            current_list.append(1)
        else:
            current_list.append(0)

    return current_list


class POYOptimizer(Annealer):

    def __init__(self, state):
        super(POYOptimizer, self).__init__(state)
        self.TAU = 0.6
        self.pass_dict = {}
        self.yield_dict = {}
        self.total_file_num = 0

    ###-------------------------------###
    ###---Time to Action Estimation---###
    ###-------------------------------###
    def CDF(self, TTC, v_car):
        ALPHA = self.state[0]
        SLOPE = self.state[1]
        STD = self.state[2]
        A_DEC = self.state[3]
        R_MIN = self.state[4]
        #--- Get the estimated TTA ---#
        R_I = 0.0
        if A_DEC == 0: pass
        else: R_I = v_car**2 / (2 * A_DEC)
        TTA_est = (R_I + v_car*self.TAU + R_MIN ) / v_car
        TTA_act = TTA_est * SLOPE   # mean of the PDF
        std = TTA_est * STD    # standard deviation of the PDF
        cdf = norm(TTA_act, std).cdf(TTC)
        
        return cdf
    

    ###----------------------------------- ###
    ###------Probability of Stopping------ ###
    ###----------------------------------- ###
    def POS(self, min_TTC, TTC, TTC_p, time, time_p, v_car):
        ALPHA = self.state[0]

        #--- Variables ---#
        TTC_dif = (TTC - TTC_p) / (time - time_p)

        #--- GAMMA ---#
        if (TTC_dif + 1) < 0:
            GAMMA = 0
        else:
            GAMMA = (TTC_dif + 1) * ALPHA

        #--- Probability of Stopping ---#
        cdf_0 = self.CDF(min_TTC, v_car)
        judge_p_stop = (1 - cdf_0) * GAMMA
        if judge_p_stop > 1 :
            p_stop = 1
        else:
            p_stop = judge_p_stop

        return p_stop


    def SaveTxtData(self):

    # Open each file and save them as array

        dir_name = "liuyc_0"
        driver_name = "lukc"

        path0 = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*{1}*prius0*'.format(dir_name, driver_name)
        path1 = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*{1}*prius1*'.format(dir_name, driver_name)

        files = glob.glob(path0) + glob.glob(path1)

    # Checking files status
        if len(files)%2 != 0 or len(glob.glob(path0))!=len(glob.glob(path1)): 
            print("Missing file(s)! There must be equal number of *prius0 and *prius1 to process.")
            raise EOFError  

    # File seperated by names e.g. liuyc_lukc...
        file_names_list = []
        self.total_file_num = 0

        for name in files:
            file_name_list = name.split('/')[-1].split("_")
            file_name = file_name_list[0]+"_"+file_name_list[1]
            if len(file_names_list) == 0:
                file_names_list.append(file_name)
            else:
                check = 0   # 1 need to add
                for j in file_names_list:
                    if j == file_name:
                        check = 0
                        break
                    else:
                        check = 1
                if check == 1 : file_names_list.append(file_name)

        for names in file_names_list:

            final_CAR_list = []
            the_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/{1}*'.format(dir_name, names)
            FILE_NUM = int(len(glob.glob(the_path))/2 )
            self.total_file_num += FILE_NUM

            for i in range(FILE_NUM):   
                real_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/'.format(dir_name)

                first_name = names.split("_")[0]
                second_name = names.split("_")[1]

            # prius0 
                prius0 = DataAnalysis()
                file_name0 = names +"_"+"{0}".format(i+1)+"_"+"prius0"
                path0 = real_path + file_name0
                # every_num=n : average every n data
                prius0_temp_d2n = prius0.get_final_pose(path0, 1)
            # prius1    
                prius1 = DataAnalysis()
                file_name1 = names +"_"+"{0}".format(i+1)+"_"+"prius1"
                path1 = real_path + file_name1
                # every_num=n : average every n data
                prius1_temp_d2n = prius1.get_final_pose(path1, 1)

                if first_name == driver_name : 
                    if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                        self.yield_dict['{0}'.format(file_name0)] = [prius0.car_t_list, prius0.car_vel_list, prius0.car_t2n_list]
                    elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                        self.pass_dict['{0}'.format(file_name0)] = [prius0.car_t_list, prius0.car_vel_list, prius0.car_t2n_list]

                elif second_name == driver_name : 
                    if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                        self.pass_dict['{0}'.format(file_name1)] = [prius1.car_t_list, prius1.car_vel_list, prius1.car_t2n_list]
                    elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                        self.yield_dict['{0}'.format(file_name1)] = [prius1.car_t_list, prius1.car_vel_list, prius1.car_t2n_list]
                    
                    #final_CAR_list.append(prius0.ods_sub_data[0])


    def get_POS_list(self, car_t_list=None, car_vel_list=None, car_t2n_list=None):
    # Probability of Stopping (yielding)
        car_POS_list = []
        # append time (x value) in this loop, or x and y might not match
        for i in range(len(car_t2n_list)-1):
            car_POS_list.append(self.POS(min(car_t2n_list[:i+1]), car_t2n_list[i+1], car_t2n_list[i], car_t_list[i+1], car_t_list[i], car_vel_list[i+1]))
         
        return car_POS_list


    def move(self):
        self.state = {'ALPHA':0.0, 'SLOPE':0.0, 'STD':0.0, 'A_DEC':0.0,'R_MIN':0.0, 'TAU':0.6}

        #arange(upper, lower, step)
        ALPHA_list = np.arange(0.1, 2.3, 0.2)  
        SLOPE_list = np.arange(0.1, 2.3, 0.2)  
        STD_list = np.arange(0.01, 0.23, 0.02) 
        A_DEC_list = np.arange(1.0, 8.7, 0.7)  
        R_MIN_list = np.arange(5.0, 16, 1.0)   
        #TAU_list = np.arange(0.1, 1.2, 0.1)    
        #thresh_list = np.arange(0.1, 1.2, 0.1)    

        r1 = random.randint(0 , len(ALPHA_list) - 1)
        r2 = random.randint(0 , len(SLOPE_list) - 1)
        r3 = random.randint(0 , len(STD_list) - 1)
        r4 = random.randint(0 , len(A_DEC_list) - 1)
        r5 = random.randint(0 , len(R_MIN_list) - 1)
        #r6 = random.randint(0 , len(TAU_list) - 1)
        #r7 = random.randint(0 , len(thresh_list) - 1)
        
        self.state = [ALPHA_list[r1], SLOPE_list[r2], STD_list[r3], A_DEC_list[r4], R_MIN_list[r5]]


    def energy(self):

        CARarea = 0.0
        final_CAR_list = []

    # States(parameters) are updated every time self.POS is called

    # PASS cases
        for key in self.pass_dict:
            pass_POS_list = self.get_POS_list(self.pass_dict[key][0], self.pass_dict[key][1], self.pass_dict[key][2])
            final_CAR_list.append(CAR_pass_analysis(pass_POS_list))
    # YIELD cases
        for key in self.yield_dict:
            yield_POS_list = self.get_POS_list(self.yield_dict[key][0], self.yield_dict[key][1], self.yield_dict[key][2])
            final_CAR_list.append(CAR_yield_analysis(yield_POS_list))

        # Add the sumation of final_CAR_list into itself
            # copy the list (list.copy() available in 3.3)
        dummy_CAR_list = final_CAR_list[:]
        summed_CAR_list = []
        for i in dummy_CAR_list:
            summed_CAR_list = map(sum,itertools.izip_longest(summed_CAR_list, i, fillvalue = 0))   # list has different length

    # Final calculation for area
        summed_CAR_list = list(np.array(summed_CAR_list) /float(self.total_file_num))
        #summed_CAR_list = [float(j) for j in summed_CAR_list]
        CARarea = sum(summed_CAR_list)

        '''No weighting to different T-minus yet
        '''
        ''' SimAnnealing is for minimum, we are looking got mazimum hence the '-'
        '''
        return -CARarea


if __name__ == '__main__':
    # initial state, a randomly-ordered itinerary
    init_state = [0.3, 0.81, 0.101, 3.076, 7.73]   # LiuYC's params

    poyop = POYOptimizer(init_state)
    poyop.steps = 1000
    # since our state is just a list, slice is the fastest way to copy
    poyop.copy_strategy = "slice"

# Data Collection (to read txt only once)
    poyop.SaveTxtData()
    state, e = poyop.anneal()


    print()
    print(" Final area under CAR curve : {0}\n".format(e) )
    param_name = ['ALPHA', 'SLOPE', 'STD', 'A_DEC', 'R_MIN']
    for i in range(len(state)):
        print("    {0} : {1}\t".format(param_name[i], state[i]))

    print()

    init_t = time.time()
    current_t = init_t
    count = 0
    for _ in range(10):
        count+=1
        current_t = time.time()



    '''

        print("\rTrial No.{2}:result from list spent {1:.4f} secs,Total spent time={0:.4f}".format(time.time()-init_t, time.time()-current_t, count), file=sys.stderr, end='' )
        sys.stderr.flush()

    '''

