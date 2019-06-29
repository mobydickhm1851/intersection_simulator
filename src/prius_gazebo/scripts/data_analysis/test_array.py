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
    
    state0 = [0.82, 0.285, 3.099, 8.581]  # measured param of LIUYC
    state1 = [0.95837, 0.011, 1.433, 9.449]  # below are optimized
    state2 = [0.9643, 0.0439, 2.57, 14.94]
    state3 = [0.8568, 0.0114, 1.489, 12.446]
    state4 = [0.889, 0.068, 1.5, 14.8]
    state5 = [0.992, 0.011, 2.282, 11.89]

    states = [state0, state1, state2, state3, state4, state5]
    
    fig1 = plt.figure(1, figsize=(14,7))
    ax1 = fig1.add_subplot(111)   # (211)2 figs in 1 plot
    #ax2 = fig1.add_subplot(212)

    ax1.clear()
    #ax2.clear()
    ax1.set_xlim(0,5)

    title = "Classification Accuracy Rate versus T-minus (LIUYC)"    
    ax1.set_title(title, fontsize=15)
    ax1.set_xlabel('T-minus (s)')
    ax1.set_ylabel('CAR')

    width = 10
    for st in states:
        width /= 1.3
        test = POYOptimizer(st)
        test.SaveTxtData()
        e = test.energy()
        label_name = "{0}-{1}-{2}-{3}".format(st[0],st[1],st[2],st[3])
        t_minus = list(np.arange(0,0.01*(len(test.summed_list)),0.01))
        ax1.plot(t_minus, test.summed_list, label = label_name, linewidth = width, alpha=1)
        print(type(test.summed_list))
        print(np.shape(test.summed_list))
        print("energ is {0}".format(e))

    ax1.legend(loc='best', frameon=True)
    plt.show()
    
    ''' Test is array is faster
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
    '''
