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
    dir_name = "lukc"
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
    param_dict_0 = {'ALPHA':2.21, 'SLOPE':2.41, 'STD':0.19, 'A_DEC':0.81,'R_MIN':9.61, 'TAU':0.6}   # Driver's params
    #param_dict_1 = {'ALPHA':.2, 'SLOPE':0.744, 'STD':0.09857, 'A_DEC':2.44, 'R_MIN':6.4, 'TAU':0.6}
    trial_num = 8

    yield_count = 0
    pass_count = 0
    final_ods_list = [] 


# CAR classification


    for names in file_names_list:

        param_dict_1 = param_dict_0

        final_CAR_list = []
        the_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/{1}*'.format(dir_name, names)
        FILE_NUM = int(len(glob.glob(the_path))/2 )

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

        # prius1    
            prius1 = DataAnalysis()
            prius1.set_params(param_dict_1)
            path1 = real_path + names +"_"+"{0}".format(i+1)+"_"+"prius1"
            prius1_temp_d2n = prius1.get_final_pose(path1)
            prius1.get_POS_list()


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
        summed_CAR_list = list(np.array(summed_CAR_list) /float(FILE_NUM))
        summed_CAR_list = [float(j) for j in summed_CAR_list]
        summed_CAR_list.insert(0, "{6}({7})_{0}-{1}-{2}-{3}-{4}-{5}".format(param_dict_0['ALPHA'], param_dict_0['SLOPE'], param_dict_0['STD'], param_dict_0['A_DEC'], param_dict_0['R_MIN'], param_dict_0['TAU'], driver_name, names))
        #####final_CAR_list_0.append(summed_CAR_list_0)

        final_ods_list.append(summed_CAR_list)
        print("Done adding CAR results together.")


    print("Opening file CAR_results.ods .")
    data = get_data("CAR_results.ods")
    print("CAR_results.ods opened .")
    data["{0}_optimumParam_test_{1}".format(driver_name, trial_num)] = final_ods_list

    print("Saving param to ods file.")
    save_data("CAR_results.ods", data)
    print("Done saving to ods file.")

    print("The whole process takes {0:.2f} seconds.".format(time.time()-time_start))
    print("prius0 yielded {0} times; passed {1} times.".format(yield_count, pass_count))
