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

style.use('seaborn-whitegrid')

data = get_data("param_results.ods")

driver_name = 'paul'
param = 'A_DEC'
params_dict = {'SLOPE': 1, 'A_DEC':2, 'R_MIN':3, 'TTA':4}

TTA_list = data["{0}_param_9".format(driver_name)][params_dict[param]]
TTA_list = TTA_list[1:-2]
#TTA_list_2 = data["{0}_param_9".format(driver_name_2)][7]
#TTA_list_2 = TTA_list_2[1:21]

# Fit a normal distribution to the data:
mu1, std1 = norm.fit(TTA_list)
#mu2, std2 = norm.fit(TTA_list_2)



if __name__ == '__main__':
    fig1 = plt.figure(1, figsize=(8,7))
    ax1 = fig1.add_subplot(111)   # (211)2 figs in 1 plot
    #ax2 = fig1.add_subplot(212)

    ax1.clear()
    #ax2.clear()

    #weight1 = np.ones(len(TTA_list1))/len(TTA_list1)

    n1, bins1, ignored = ax1.hist(TTA_list, 20, normed=True, color = "skyblue", alpha=0.7)
    # Plot the PDF.
    xmin1, xmax1 = plt.xlim()
    #xmin1, xmax1 = 1.5, 3.5
    #print("xmin1 {0} and xmax1 {1}".format(xmin1, xmax1))
    x1 = np.linspace(xmin1, xmax1, 100)
    p1 = norm.pdf(x1, mu1, std1)
    ax1.plot(x1, p1, 'k', linewidth=2, label="PDF of {1}({0})".format(driver_name.upper(), param.upper()))
    #print("this is x1 {0}, p1 {1} ".format(x1, p1))
    title = "Fit results for Driver {1} Distribution ({0})".format(driver_name.upper(), param.upper())    
    ax1.legend(loc='best', frameon=True)
    ax1.set_title(title, fontsize=15)
    ax1.set_ylabel('Probability (P)')
    ax1.set_xlabel('{0}'.format(param.upper()))
    ax1.text(2.75, 0.85,r"$(\mu = {0:.2f}, \sigma = {1:.2f}, N = {2})$".format(mu1, std1, len(TTA_list[1:-2])), fontsize=12)
    
    '''
    n2, bins2, ignored = ax2.hist(TTA_list_2, 50, normed=True, color = "skyblue", alpha=0.7)
    # Plot the PDF.
    xmin2, xmax2 = plt.xlim()
    x2 = np.linspace(xmin2, xmax2, 100)
    p2 = norm.pdf(x2, mu2, std2)
    ax2.plot(x2, p2, 'k', linewidth=2, label="PDF of {0}".format(driver_name_2.upper()))
    ax2.text(2.4,2.5,r"$(\mu = {0:.2f}, \sigma = {1:.2f})$".format(mu2, std2), fontsize=12)
    '''

    path = "/home/liuyc/moby_folder/Research/figures/intersection_simulator/driver_param/"
    plt.savefig(path+'{0}_{1}_fitting.pdf'.format(driver_name.upper(), trial_num))
    plt.show()

'''
Problem concerning the usage of "normed" or "density" arguments in hist method, check this discussion thread here : https://github.com/matplotlib/matplotlib/issues/10398/.

Long story short, histagram should use Probability Mass Function(PMF) rather than Probability Density Function(PDF). Both argument are making the sum of n (return of the hist method) as the area of the bins, similar to the area under the curve in PDF.
'''
