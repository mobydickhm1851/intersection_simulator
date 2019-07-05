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

if __name__ == '__main__':
    
    from POY_anneal import POYOptimizer
    from POY_anneal import CAR_pass_analysis
    from POY_anneal import CAR_yield_analysis
    
    
    x = 1.1
    '''
    state0 = [0.779, 0.081, 4.47, 7.547,x]  # measured param of LUKC
    state1 = [0.779, 0.081, 2.13, 7.547,x]  # measured param of LUKC
    state2 = [0.919, 0.461, 6.616, 9.967, x]  # below are optimized
    state3 = [0.806, 0.4399, 7.1195, 12.156,x]  # measured param of LUKC
    state4 = [0.6748, 0.4435, 5.419, 14.906, x]
    state5 = [0.4327, 0.3653, 1.141, 4.987, x]
    state6 = [0.457, 0.1333, 1.5655, 5.2051, x]
    state7 = [0.457, 0.0316, 2.1923, 4.7638, x]


    '''

    state0 = [0.05, 0.09, 3.099, 8.581, x]  # measured param of LIUYC
    state1 = [0.05, 0.09, 1.956, 8.581, x]  # below are optimized
    state2 = [0.86359, 0.413, 1.117, 10.438, x]
    state3 = [1.177, 0.44823, 5.168, 11.908, x]
    state4 = [0.01026, 0.082, 6.27, 4.85, x]  
    state5 = [0.629, 0.169, 5.419, 14.84, x]  
    state6 = [0.8913, 0.1135, 4.804, 8.829, x]  
    state7 = [0.8913, 0.1135, 1.804, 8.829, x]  
    

    
    #states = [state0, state1, state2, state3, state4]
    #states = [state0, state1,state2, state3, state4, state5, state6]
    states = [state0, state1, state2, state3, state4, state5, state6, state7]
    
    fig1 = plt.figure(1, figsize=(14,7))
    ax1 = fig1.add_subplot(111)   # (211)2 figs in 1 plot
    #ax2 = fig1.add_subplot(212)

    ax1.clear()
    #ax2.clear()
    ax1.set_xlim(0,7)

    title = "Classification Accuracy Rate versus T-minus (LIUYC)"    
    ax1.set_title(title, fontsize=15)
    ax1.set_xlabel('T-minus (s)')
    ax1.set_ylabel('CAR')

    width = 10
    for st in states:
        width /= 1.5
        test = POYOptimizer(st)
        test.SaveTxtData()
        e = test.energy()
        label_name = "[{4:.2f}]{0}-{1}-{2}-{3}".format(st[0],st[1],st[2],st[3],e)
        t_minus = list(np.linspace(0,0.01*(len(test.summed_list)-1),len(test.summed_list)))
        ax1.plot(t_minus, test.summed_list, label = label_name, linewidth = width, alpha=1)
        print(type(test.summed_list))
        print(np.shape(test.summed_list))
        print("energ is {0}".format(e))
        print("CARareadiff is {0}".format(test.CARareadiff))
        print("CARare is {0}".format(e - test.CARareadiff))

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
