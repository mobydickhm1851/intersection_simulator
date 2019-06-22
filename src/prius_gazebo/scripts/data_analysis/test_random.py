#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm
from collections import OrderedDict
from pyexcel_ods import save_data
from pyexcel_ods import get_data
import random
import time
'''
    TTA_est_arr = []
    TTA_est_arr_2 = []

    for _ in range(10000):
        TTA_est_arr.append(random.uniform(0.01, 5.0))

    dummy = np.arange(0.01, 5.00, 0.1)
    for _ in range(10000):
        TTA_est_arr_2.append(random.choice(dummy))


# Plotting
    fig1 = plt.figure(1, figsize=(12,18))
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)

    ax1.clear()
    ax2.clear()
    
    # normed=True is for older version
    n1, bins1, ignored = ax1.hist(list(TTA_est_arr), 100, normed=True)
    n2, bins2, ignored = ax2.hist(list(TTA_est_arr_2), 100, normed=True)

    plt.show()
'''

if __name__ == '__main__':
    
    from POY_anneal import POYOptimizer
    from POY_anneal import CAR_pass_analysis
    from POY_anneal import CAR_yield_analysis
    
    state = [1.0,2.0,3.0,4.0,5.0]
    test = POYOptimizer(state)
    
    a0 = np.array([1.0,2.0])
    a1 = np.array([2.0,4.0])
    a2 = np.array([1.0,2.0])
    a3 = np.array([7.0,9.0])
    a4 = np.array([1.0,2.0])

    dum = np.random.random_sample(300)

    t0 = time.time() 
    result1 = CAR_pass_analysis(dum)
    T1 = time.time()-t0
    print("using for loop and list takes {0} secs".format(T1))

    t0 = time.time() 
    result2 = CAR_yield_analysis(dum)
    T2 = time.time()-t0
    print("using Numpy array takes {0} secs".format(T2))
    print("array is {0} times faster".format(T1/T2))

