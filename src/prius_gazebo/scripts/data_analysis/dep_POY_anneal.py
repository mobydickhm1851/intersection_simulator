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
POS_PASS_THRESH = 0.2


###---- Using 1-D Arrays as input ---- ###
def CAR_yield_analysis(car_POS_list):
#### COUNTING CAR ####
# Turn time into t-minus (copy one, avoide changing on the original list )
    # copy the list (list.copy() available in 3.3)
    POS_list = car_POS_list[:]
    POS_list = POS_list[::-1]
    current_list = np.array([])

    CAR_list = (POS_list >= POS_YIELD_THRESH).astype(int)
    diff_list = abs(POS_list - 0)
    #diff_list = abs(POS_list - POS_YIELD_THRESH)


    return [CAR_list, diff_list]


###---- Using 1-D Arrays as input ---- ###
def CAR_pass_analysis(car_POS_list):
#### COUNTING CAR ####
# Turn time into t-minus (copy one, avoide changing on the original list )
    # copy the list (list.copy() available in 3.3)
    POS_list = car_POS_list[:]
    POS_list = POS_list[::-1]
    current_list = np.array([])

    CAR_list = (POS_list <= POS_PASS_THRESH).astype(int)
    diff_list = abs(POS_list - 0)
    #diff_list = abs(POS_list - POS_PASS_THRESH)

    return [CAR_list, diff_list]


class POYOptimizer(Annealer):

    def __init__(self, state):
        super(POYOptimizer, self).__init__(state)
        self.TAU = 0.6
        self.pass_dict = {}
        self.yield_dict = {}
        self.total_file_num = 0
        self.summed_list = []

    ###-------------------------------###
    ###---Time to Action Estimation---###
    ###-------------------------------###
    def CDF(self, min_TTC, v_car, TTC_dif, TTC):
        SLOPE = self.state[0]
        STD = self.state[1]
        A_DEC = self.state[2]
        R_MIN = self.state[3]
        ALPHA = np.array([])

        GAMMA = self.state[4]
        #--- Get the estimated TTA ---#

        R_I = 0.0
        if A_DEC == 0: pass
        else: R_I = v_car**2 / (2 * A_DEC)
        TTA_est = (R_I + v_car*self.TAU + R_MIN ) / v_car * SLOPE
        std = TTA_est * STD    # standard deviation of the PDF

        ALPHA = (abs( min_TTC - TTA_est+ 0.01) ) * np.log(abs(TTC_dif+1) * np.exp(1) + np.exp(GAMMA))   
        #ALPHA = abs(1+ min_TTC - TTA_est) * np.log(abs(TTC_dif+1) * np.exp(1) + np.exp(1))   

        for i in range(len(TTC_dif)):
            if (TTC_dif[i] + 1) < 0 : ALPHA[i] = -ALPHA[i] 

        TTA_act = TTA_est + ALPHA   # mean of the PDF
        cdf = norm(TTA_act, std).cdf(min_TTC)
        
        return cdf
    

    ###----------------------------------- ###
    ###------Probability of Stopping------ ###
    ###----------------------------------- ###
    def POS(self, min_TTC, TTC, TTC_p, time, time_p, v_car):
        
        #--- Variables ---#
        TTC_dif = (TTC - TTC_p) / (time - time_p)
        
        '''
        #--- GAMMA ---#
        if (TTC_dif + 1) < 0:
            self.GAMMA = 0
        else:
            self.GAMMA = (TTC_dif + 1) * self.ALPHA
        '''

        #--- Probability of Stopping ---#
        cdf_0 = self.CDF(min_TTC, v_car, TTC_dif, TTC)
        judge_p_stop = (1 - cdf_0)

        return judge_p_stop



    def SaveTxtData(self):

    # Open each file and save them as array

        dir_name = "lukc"
        self.driver_name = "lukc"

        path0 = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*{1}*prius0'.format(dir_name, self.driver_name)
        path1 = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*{1}*prius1'.format(dir_name, self.driver_name)

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

                if first_name == self.driver_name : 
                    if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                        self.yield_dict['{0}'.format(file_name0)] = [prius0.car_t_list, prius0.car_vel_list, prius0.car_t2n_list]
                    elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                        self.pass_dict['{0}'.format(file_name0)] = [prius0.car_t_list, prius0.car_vel_list, prius0.car_t2n_list]

                elif second_name == self.driver_name : 
                    if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                        self.pass_dict['{0}'.format(file_name1)] = [prius1.car_t_list, prius1.car_vel_list, prius1.car_t2n_list]
                    elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                        self.yield_dict['{0}'.format(file_name1)] = [prius1.car_t_list, prius1.car_vel_list, prius1.car_t2n_list]
                    
                    #final_CAR_list.append(prius0.ods_sub_data[0])

    # input should bei 1-D ARRAYs !!! #
    def get_POS_list(self, car_t_list=None, car_vel_list=None, car_t2n_list=None):
    # Probability of Stopping (yielding)
        car_POS_list = np.array([])
        # append time (x value) in this loop, or x and y might not match

        car_POS_list = self.POS( min(car_t2n_list[1:]), car_t2n_list[1:], car_t2n_list[:-1], car_t_list[1:], car_t_list[:-1], car_vel_list[1:])
         
        return car_POS_list


    def move(self):
        #self.state = {'ALPHA':0.0, 'SLOPE':0.0, 'STD':0.0, 'A_DEC':0.0,'R_MIN':0.0, 'TAU':0.6}

        #arange(upper, lower, step)
        #ALPHA = [0.01, 3.0] 
        SLOPE = [0.01, 1.2] 
        STD   = [0.01, 0.5] 
        A_DEC = [1.0, 8.0] 
        R_MIN = [4.48, 15.0 ]
        #TAU = np.arange(0.1, 1.2, 0.1)    
        #thresh = np.arange(0.1, 1.2, 0.1)    
        GAMMA = [0, 10]    

        
        ''' semi-continuous random 
        '''
        #r1 = random.uniform(ALPHA[0], ALPHA[1])
        r2 = random.uniform(SLOPE[0], SLOPE[1])
        r3 = random.uniform(STD[0], STD[1])
        r4 = random.uniform(A_DEC[0], A_DEC[1])
        r5 = random.uniform(R_MIN[0], R_MIN[1])
        #r6 = random.uniform(TAU[0], TAU[1])
        #r7 = random.uniform(thresh[0], thresh[1])
        r8 = random.uniform(GAMMA[0], GAMMA[1])
        
        ''' discrete random 
        #r1_list = np.arange(ALPHA[0], ALPHA[1], 0.2)
        r2_list = np.arange(SLOPE[0], SLOPE[1], 0.2)
        r3_list = np.arange(STD[0], STD[1], 0.02)
        r4_list = np.arange(A_DEC[0], A_DEC[1], 0.2)
        r5_list = np.arange(R_MIN[0], R_MIN[1], 0.2)
        #r6_list = np.arange(TAU[0], TAU[1], 0.1)
        #r7_list = np.arange(thresh[0], thresh[1], 0.1)
        #r1 = random.choice(r1_list)
        r2 = random.choice(r2_list)
        r3 = random.choice(r3_list)
        r4 = random.choice(r4_list)
        r5 = random.choice(r5_list)
        #r6 = random.choice(r6_list)
        #r7 = random.choice(r7_list)
        '''

        #self.state = [0.01, r2, r3, r4, r5]
        self.state = [r2, r3, r4, r5, 1.1]



    def energy(self):

        CARarea = 0.0
        final_CAR_list = []
        summed_CAR_list = []
        summed_diff_list = []

    # States(parameters) are updated every time self.POS is called

    # PAupdatesSS cases
        for key in self.pass_dict:
            pass_POS_list = self.get_POS_list( np.array(self.pass_dict[key][0]), np.array(self.pass_dict[key][1]), np.array(self.pass_dict[key][2]) )
            summed_CAR_list = map( sum, itertools.izip_longest(summed_CAR_list, CAR_pass_analysis(pass_POS_list)[0], fillvalue=0) )
            summed_diff_list = map( sum, itertools.izip_longest(summed_diff_list, CAR_pass_analysis(pass_POS_list)[1], fillvalue=0) )
            #final_CAR_list.append(CAR_pass_analysis(pass_POS_list))
    # YIELD cases
        for key in self.yield_dict:
            yield_POS_list = self.get_POS_list( np.array(self.yield_dict[key][0]), np.array(self.yield_dict[key][1]), np.array(self.yield_dict[key][2]) )
            summed_CAR_list = map( sum, itertools.izip_longest(summed_CAR_list, CAR_yield_analysis(yield_POS_list)[0], fillvalue=0) )
            summed_diff_list = map( sum, itertools.izip_longest(summed_diff_list, CAR_yield_analysis(yield_POS_list)[1], fillvalue=0) )
            #final_CAR_list.append(CAR_yield_analysis(yield_POS_list))

        ''' Deprecated, might be useful when debugging
        # Add the sumation of final_CAR_list into itself
            # copy the list (list.copy() available in 3.3)
        dummy_CAR_list = final_CAR_list[:]
        for i in dummy_CAR_list:
            summed_CAR_list = map(sum,itertools.izip_longest(summed_CAR_list, i, fillvalue = 0))   # list has different length
        '''

    # Final calculation for area
        summed_CAR_list = np.array(summed_CAR_list) / float(self.total_file_num)
        summed_diff_list = np.array(summed_diff_list) / float(self.total_file_num)
        self.summed_list = summed_CAR_list    # for DEBUG

        '''Weighting for T-minus from 1.5 to 2.5 secs''' 
        LEN = len(summed_CAR_list)
        WEIGHT = 1   # weight value 12
        PENALTY = 1    # penalty value-4
        self.WEIGHT = WEIGHT
        self.PENALTY = PENALTY
        weight_arr = np.append(np.append(np.ones(100)*PENALTY, np.ones(300)*WEIGHT,) ,np.ones(LEN-400)*PENALTY,)
        summed_CAR_list = weight_arr*summed_CAR_list

        CARarea = -sum(summed_CAR_list)    # larger the better
        CARareadiff = sum(summed_diff_list)   # smaller the better

        ''' SimAnnealing is for minimum, mazimum with negative sign
        '''
        #return  CARarea
        self.CARareadiff = CARareadiff
        return (CARarea + 3 * CARareadiff)   # * 0.01


if __name__ == '__main__':
    # initial state, a randomly-ordered itinerary
    #init_state = [0.01, 0.3, 0.08, 8.5, 3.0]   # LiuYC's params
    init_state = [0.3, 0.08, 8.5, 3.0, 1]   # average params


    poyop = POYOptimizer(init_state)
    poyop.steps = 20077
    # Optimized Tmax, Tmin:steps:260.0, tmax:110.0, tmin:0.073, updates: 100
    trial = "1area3diff(4params)"
    poyop.Tmax = 115
    poyop.Tmin = 0.05
    # since our state is just a list, slice is the fastest way to copy
    poyop.copy_strategy = "slice"

# Data Collection (to read txt only once)
    poyop.SaveTxtData()


    state, e = poyop.anneal()
    print()
    print(" Final area under CAR curve : {0}\n".format(e) )
    #param_name = ['ALPHA', 'SLOPE', 'STD', 'A_DEC', 'R_MIN']
    param_name = [ 'SLOPE', 'STD', 'A_DEC', 'R_MIN', 'GAMMA', 'WEIGHT', 'PENALTY',"energy"]

    for i in range(len(state)):
        print(" {0} : {1}\t".format(param_name[i], state[i]))
    print("Tmax={0}, Tmin={1}, steps={2}".format(poyop.Tmax, poyop.Tmin, poyop.steps))
    print("Saving Optimized data into ods file.")

    param_list = [state[0], state[1] ,state[2], state[3], state[4], poyop.WEIGHT, poyop.PENALTY,float(e)]


    data = get_data("SAparam.ods")

    if "{0}-{1}_{2}-{3}_{4}".format(poyop.driver_name, poyop.WEIGHT, poyop.PENALTY, poyop.steps, trial) in data:
        dum_data = data["{0}-{1}_{2}-{3}_{4}".format(poyop.driver_name, poyop.WEIGHT, poyop.PENALTY, poyop.steps, trial)]
        for j in range(len(dum_data)):
            dum_data[j].append(param_list[j])
        data["{0}-{1}_{2}-{3}_{4}".format(poyop.driver_name, poyop.WEIGHT, poyop.PENALTY, poyop.steps, trial)] = dum_data
    else:
        dum_data = []
        for k in range(len(param_list)):
            dum_data.append([param_name[k], param_list[k]])
        data["{0}-{1}_{2}-{3}_{4}".format(poyop.driver_name, poyop.WEIGHT, poyop.PENALTY, poyop.steps, trial)] = dum_data

    save_data("SAparam.ods", data)


    '''
    schedule = poyop.auto(10, 2000)
    print(schedule)

    '''

    '''
    print()

    init_t = time.time()
    current_t = init_t
    count = 0
    for _ in range(10):
        count+=1
        current_t = time.time()



        print("\rTrial No.{2}:result from list spent {1:.4f} secs,Total spent time={0:.4f}".format(time.time()-init_t, time.time()-current_t, count), file=sys.stderr, end='' )
        sys.stderr.flush()

    '''

    '''
    ###-------------------------------###
    ###---Time to Action Estimation---###
    ###- Using 1-D Arrays as input -- ###
    def CDF(self, TTC, v_car, TTC_dif):
        SLOPE = self.state[0]
        STD = self.state[1]
        A_DEC = self.state[2]
        R_MIN = self.state[3]
        ALPHA = np.array([])
        #--- Get the estimated TTA ---#
        R_I = 0.0
        for ttc_dif in TTC_dif: 
            if (ttc_dif + 1) >= 0 : ALPHA = np.append(ALPHA, math.log(ttc_dif + 1 + 4, 4), )
            else : ALPHA = np.append(ALPHA, -999999, )   # -inf
        
        if A_DEC == 0: pass
        else: R_I = v_car**2 / (2 * A_DEC)
        TTA_est = (R_I + v_car*self.TAU + R_MIN ) / v_car * SLOPE
        std = TTA_est * STD    # standard deviation of the PDF

        TTA_act = TTA_est * ALPHA   # mean of the PDF
        cdf = norm(TTA_act, std).cdf(TTC)
        
        return cdf
    

    ###----------------------------------- ###
    ###------Probability of Stopping------ ###
    ###---- Using 1-D Arrays as input ---- ###
    def POS(self, MIN_TTC, TTC, TTC_p, time, time_p, v_car):
        #ALPHA = self.state[0]
        p_stop = np.array([])

        #--- Variables ---#
        TTC_dif = (TTC - TTC_p) / (time - time_p)

        #--- Probability of Stopping ---#
        cdf_0 = self.CDF(MIN_TTC, v_car, TTC_dif)
        judge_p_stop = (1 - cdf_0)
        
        for jps in judge_p_stop: 
            if jps > 1 :
                p_stop = np.append(p_stop, 1,)
            else:
                p_stop = np.append(p_stop, jps,)

        return p_stop
    '''
