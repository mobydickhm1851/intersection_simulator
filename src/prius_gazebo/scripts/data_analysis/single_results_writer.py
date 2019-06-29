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
from POY.POYanalysis import DataAnalysis


time_start = time.time()




if __name__ == '__main__':

##SETING01##
    
    param_dict_0 = {'SLOPE':0.95837 , 'STD':0.011, 'A_DEC':1.433 ,'R_MIN':9.449, 'TAU':0.6}   # optimized params
    #param_dict_0 = {'SLOPE':0.816 , 'STD':0.350, 'A_DEC':2.851 ,'R_MIN':7.277, 'TAU':0.6}   # Paul's params
    #param_dict_0 = {'SLOPE':0.82 , 'STD':0.285, 'A_DEC':3.099 ,'R_MIN':8.581, 'TAU':0.6}   # LiuYC's params
    #param_dict_0 = {'SLOPE':0.779 , 'STD':0.212, 'A_DEC':4.475 ,'R_MIN':7.547, 'TAU':0.6}   # LuKC's params
    #param_dict_0 = {'SLOPE':0.8, 'STD':0.25, 'A_DEC':3.5 ,'R_MIN':7.0, 'TAU':0.6}   # average params
    step = "addALPHA_optimized"
    weight = "measures_0"
    # SHEET name 
    trial_num = "3"

    dir_name = "liuyc"
    driver_name = "liuyc"

    path0 = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*{1}*prius0'.format(dir_name, driver_name)
    path1 = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/*{1}*prius1'.format(dir_name, driver_name)
    files = glob.glob(path0) + glob.glob(path1)

# Checking files status
    if len(files)%2 != 0 or len(glob.glob(path0))!=len(glob.glob(path1)): 
        print("Missing file(s)! There must be equal number of *prius0 and *prius1 to process.")
        raise EOFError  

# File seperated by names e.g. liuyc_lukc...
    file_names_list = []
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


##SETING02##

    yield_count = 0
    pass_count = 0
    final_ods_list = [] 


# CAR classification


    file_num_list = []
    for names in file_names_list:

        param_dict_1 = param_dict_0

        final_CAR_list = []
        the_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/{1}*'.format(dir_name, names)
        FILE_NUM = int(len(glob.glob(the_path))/2 )
        file_num_list.append(FILE_NUM)

        for i in range(FILE_NUM):   
            print("\rProcessing {0}_{1}.....".format(names, i+1), file=sys.stderr, end='')
            sys.stderr.flush()
##SETING04##
            real_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/'.format(dir_name)

        # prius0 
            prius0 = DataAnalysis()
            prius0.set_params(param_dict_0)
            path0 = real_path + names +"_"+"{0}".format(i+1)+"_"+"prius0"
            prius0_temp_d2n = prius0.get_final_pose(path0)
            prius0.get_POS_list()

            #print("prius0: {0}".format(prius0.car_POS_list))

        # prius1    
            prius1 = DataAnalysis()
            prius1.set_params(param_dict_1)
            path1 = real_path + names +"_"+"{0}".format(i+1)+"_"+"prius1"
            prius1_temp_d2n = prius1.get_final_pose(path1)
            prius1.get_POS_list()

            #print("prius1: {0}".format(prius1.car_POS_list))

            first_name = names.split("_")[0]
            second_name = names.split("_")[1]
            if first_name == driver_name:

                prius0_file_name = path0.split('/')[-1]

                if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                    prius0.CAR_yield_analysis()
                    yield_count += 1

                elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                    prius0.CAR_pass_analysis()
                    pass_count += 1

                final_CAR_list.append(prius0.ods_sub_data[0])
                print("\r{0}_{1} . Done !!!!!!!".format(names, i+1), file=sys.stderr, end='')
                sys.stderr.flush()

            elif second_name == driver_name:

                prius1_file_name = path1.split('/')[-1]

                if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                    prius1.CAR_pass_analysis()
                    yield_count += 1

                elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                    prius1.CAR_yield_analysis()
                    pass_count += 1

                final_CAR_list.append(prius1.ods_sub_data[0])
                print("\r{0}_{1} . Done !!!!!!!".format(names, i+1), file=sys.stderr, end='')
                sys.stderr.flush()


        # Add the sumation of final_CAR_list into itself
            # copy the list (list.copy() available in 3.3)
        print()
        print("Adding {0}'s {1} CAR results together.".format(driver_name,FILE_NUM))
        dummy_CAR_list = final_CAR_list[:]
        summed_CAR_list = []
        for i in dummy_CAR_list:
            summed_CAR_list = map(sum,itertools.izip_longest(summed_CAR_list, i, fillvalue = 0))   # list has different length
        summed_CAR_list = list( np.array(summed_CAR_list)/float(FILE_NUM) )
        summed_CAR_list = [float(j) for j in summed_CAR_list]
        #summed_CAR_list.insert(0, "{6}({7})_{0}-{1}-{2}-{3}-{4}-{5}".format(param_dict_0['ALPHA'], param_dict_0['SLOPE'], param_dict_0['STD'], param_dict_0['A_DEC'], param_dict_0['R_MIN'], param_dict_0['TAU'], driver_name, names))
        #####final_CAR_list_0.append(summed_CAR_list_0)

        final_ods_list.append(summed_CAR_list)
        print("Done adding CAR results together.")

    ave_list = []
    for i in range(len(final_ods_list)):
        ave_list = map(sum,itertools.izip_longest(ave_list, np.array(final_ods_list[i])*float(file_num_list[i]), fillvalue = 0))   # list has different length
    ave_list = list(np.array(ave_list) /float(sum(file_num_list)))
    ave_list = [float(j) for j in ave_list]
    AREA = sum(ave_list)
    ave_list.insert(0, "{5}({6}/{7:.2f})_{0}-{1}-{2}-{3}-{4}".format(param_dict_0['SLOPE'], param_dict_0['STD'], param_dict_0['A_DEC'], param_dict_0['R_MIN'], param_dict_0['TAU'], driver_name, step,AREA))
    #ave_list.insert(0, "{6}({7}/{8})_{0}-{1}-{2}-{3}-{4}-{5}".format(param_dict_0['ALPHA'], param_dict_0['SLOPE'], param_dict_0['STD'], param_dict_0['A_DEC'], param_dict_0['R_MIN'], param_dict_0['TAU'], driver_name, step,weight))

    final_ods_list.append(ave_list)

    print("Opening file CAR_results.ods .")
    data = get_data("CAR_results_plot.ods")
    print("CAR_results.ods opened .")
    if "{0}_optimumParam_{1}".format(driver_name, trial_num) in data:
        dum_data = data["{0}_optimumParam_{1}".format(driver_name, trial_num)]
        dum_data.append(ave_list)
        data["{0}_optimumParam_{1}".format(driver_name, trial_num)] = dum_data
    else:
        data["{0}_optimumParam_{1}".format(driver_name, trial_num)] = [ave_list]


    print("Saving param to ods file.")
    save_data("CAR_results_plot.ods", data)
    print("Done saving to ods file.")

    print("The whole process takes {0:.2f} seconds.".format(time.time()-time_start))
    print("prius0 yielded {0} times; passed {1} times.".format(yield_count, pass_count))



