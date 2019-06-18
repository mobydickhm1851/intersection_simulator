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
from POY.POYanalysis import DataAnalysis


time_start = time.time()




if __name__ == '__main__':

##SETING01##
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
    param_dict_0 = {'ALPHA':0.3, 'SLOPE':0.81, 'STD':0.101, 'A_DEC':3.076,'R_MIN':7.73, 'TAU':0.6}   # LiuYC's params
    #param_dict_1 = {'ALPHA':.2, 'SLOPE':0.744, 'STD':0.09857, 'A_DEC':2.44, 'R_MIN':6.4, 'TAU':0.6}

# Loop for different params (sensitivity test)

    ALPHA_list = np.arange(0.1, 2.3, 0.2)    #arange(upper, lower, step)
    SLOPE_list = np.arange(0.1, 2.3, 0.2)    #arange(upper, lower, step)
    STD_list = np.arange(0.01, 0.23, 0.02)    #arange(upper, lower, step)
    A_DEC_list = np.arange(1.0, 8.7, 0.7)    #arange(upper, lower, step)
    R_MIN_list = np.arange(5.0, 16, 1.0)    #arange(upper, lower, step)
    TAU_list = np.arange(0.1, 1.2, 0.1)    #arange(upper, lower, step)
    thresh_list = np.arange(0.1, 1.2, 0.1)    #arange(upper, lower, step)

##SETING03##
    set_param = "ALPHA"

    yield_count = 0
    pass_count = 0
    final_ods_list = [] 

    if set_param == "ALPHA" : temp_list = ALPHA_list
    elif set_param == "SLOPE" : temp_list = SLOPE_list
    elif set_param == "STD" : temp_list = STD_list
    elif set_param == "A_DEC" : temp_list = A_DEC_list
    elif set_param == "R_MIN" : temp_list = R_MIN_list
    elif set_param == "TAU" : temp_list = TAU_list
    elif set_param == "threshold" : temp_list = thresh_list


# CAR classification


    for names in file_names_list:

        for param in temp_list:

            if set_param != "threshold":
                param_dict_0[set_param] = param
            param_dict_1 = param_dict_0

            final_CAR_list = []
            the_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/{1}*'.format(dir_name, names)
            FILE_NUM = int(len(glob.glob(the_path))/2 )

            for i in range(FILE_NUM):   
                print("Processing {0}_{1} using {2}={3}.....".format(names, i+1, set_param, param))
##SETING04##
                real_path = '/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{0}/'.format(dir_name)
            # prius0 
                prius0 = DataAnalysis(param_dict_0)
                path0 = real_path + names +"_"+"{0}".format(i+1)+"_"+"prius0"
                prius0_temp_d2n = prius0.get_final_pose(path0)
            # prius1    
                prius1 = DataAnalysis(param_dict_1)
                path1 = real_path + names +"_"+"{0}".format(i+1)+"_"+"prius1"
                prius1_temp_d2n = prius1.get_final_pose(path1)

                if set_param == "threshold":
                    prius0.POS_YIELD_THRESH=param
                    prius0.POS_PASS_THRESH=param

                first_name = names.split("_")[0]
                second_name = names.split("_")[1]
                if first_name == driver_name:

                    prius0_file_name = path0.split('/')[-1]

                    if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                        prius0.CAR_yield_analysis(prius0_file_name)
                        yield_count += 1

                    elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                        prius0.CAR_pass_analysis(prius0_file_name)
                        pass_count += 1

                    final_CAR_list.append(prius0.ods_sub_data[0])
                    print("{0}_{1} CAR analysis done.\n".format(names, i+1))

                elif second_name == driver_name:

                    prius1_file_name = path1.split('/')[-1]

                    if abs(prius0_temp_d2n) > abs(prius1_temp_d2n):
                        prius1.CAR_pass_analysis(prius1_file_name)
                        yield_count += 1

                    elif abs(prius0_temp_d2n) < abs(prius1_temp_d2n):
                        prius1.CAR_yield_analysis(prius1_file_name)
                        pass_count += 1

                    final_CAR_list.append(prius1.ods_sub_data[0])
                    print("{0}_{1} CAR analysis done.\n".format(names, i+1))


        # Add the sumation of final_CAR_list into itself
            # copy the list (list.copy() available in 3.3)
            print("Adding {0}'s {1} CAR results together.".format(driver_name,FILE_NUM))
            dummy_CAR_list = final_CAR_list[:]
            summed_CAR_list = []
            for i in dummy_CAR_list:
                i = i[1:]    # remove the name tag
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
    data["{0}_{1}_sensitivity_test".format(driver_name,set_param)] = final_ods_list

    print("Saving param {0} to ods file.".format(set_param))
    save_data("CAR_results.ods", data)
    print("Done saving to ods file.")

    print("The whole process takes {0:.2f} seconds.".format(time.time()-time_start))
    print("prius0 yielded {0} times; passed {1} times.".format(yield_count, pass_count))
