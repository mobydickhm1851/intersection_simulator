#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import numpy as np
from scipy.stats import norm
from collections import OrderedDict
from pyexcel_ods import save_data
from pyexcel_ods import get_data
import random

style.use('seaborn-whitegrid')

data = get_data("CAR_results_plot.ods")

driver_name = 'liuyc'
trial_num = "3"
sheet_name = "{0}_optimumParam_{1}".format(driver_name, trial_num)

TTA_list = data["{0}".format(sheet_name)]




if __name__ == '__main__':
    fig1 = plt.figure(1, figsize=(14,7))
    ax1 = fig1.add_subplot(111)   # (211)2 figs in 1 plot
    #ax2 = fig1.add_subplot(212)

    ax1.clear()
    #ax2.clear()
    ax1.set_xlim(0,5)

    title = "Classification Accuracy Rate versus T-minus ({0})".format(driver_name.upper())    
    ax1.set_title(title, fontsize=15)
    ax1.set_xlabel('T-minus (s)')
    ax1.set_ylabel('CAR')

    width = 10
    for row in TTA_list:
        width /= 1.2
        t_minus = list(np.arange(0,0.01*(len(row[1:])),0.01))
        ax1.plot(t_minus, row[1:], label=row[0], linewidth = width, alpha=1)
        print(width)

    ax1.legend(loc='best', frameon=True)
    path = "/home/liuyc/moby_folder/Research/MasterThesis/figures/intersection_simulator/CAR/"
    plt.savefig(path+'CAR_AVEnMEA_ALPHA_{0}_{1}.pdf'.format(driver_name.upper(), trial_num))
    plt.show()

