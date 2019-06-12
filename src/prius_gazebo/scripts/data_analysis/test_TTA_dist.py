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

class DrawNorm:

    def __init__(self, mu, sigma):
        self.MU = mu
        self.SIGMA = sigma
        self.NUM = 0    # How many random points

    def getPoints(self, number):
        self.NUM = number
        points = np.random.normal(self.MU, self.SIGMA, self.NUM)
        
        return points


def TTA(A_DEC, R_MIN):
    V_CAR = 5   #(m/s)
    TAU = 0.6  #(s)

    R_I = V_CAR**2 / (2 * A_DEC)
    TTA_est = (R_I + V_CAR * TAU + R_MIN ) / V_CAR

    return TTA_est   


if __name__ == '__main__':
    
# Normal distributions
    #TTA_real = DrawNorm(2,2)
    A_DEC = DrawNorm(2.2,0.4)
    R_MIN = DrawNorm(6.4,0.6)

    slope = DrawNorm(0.75,0.5)

    N = 1000000

    #TTA_real_arr = TTA_real.getPoints(N)
    A_DEC_arr = A_DEC.getPoints(N)
    R_MIN_arr = R_MIN.getPoints(N)

    slope_arr = slope.getPoints(N)

    TTA_est_arr = TTA(A_DEC_arr, R_MIN_arr)

    #slope_arr = TTA_real_arr/TTA_est_arr   # get the slope
    #TTA_real_arr = TTA_est_arr*SLOPE
    TTA_real_arr = TTA_est_arr*slope_arr
    

# Plotting
    fig1 = plt.figure(1, figsize=(12,18))
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)

    ax1.clear()
    ax2.clear()
    
    # normed=True is for older version
    n1, bins1, ignored = ax1.hist(list(TTA_est_arr), 100, normed=True)
    n2, bins2, ignored = ax2.hist(list(TTA_real_arr), 100, normed=True)

    plt.show()
