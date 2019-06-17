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

TTA_list_lukc = data["liuyc_param_8"][7]
TTA_list_lukc = TTA_list_lukc[1:-2]
TTA_list_liuyc = data["liuyc_param_8"][7]
TTA_list_liuyc = TTA_list_liuyc[1:21]
#TTA_list = np.random.normal(2.5, 0.37,1000)

# Fit a normal distribution to the data:
mu1, std1 = norm.fit(TTA_list_liuyc)
mu2, std2 = norm.fit(TTA_list_lukc)



if __name__ == '__main__':
    fig1 = plt.figure(1, figsize=(8,12))
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)

    ax1.clear()
    ax2.clear()
    #plt.rcParams.update({'font.size': 22})

    #weight1 = np.ones(len(TTA_list1))/len(TTA_list1)

    n1, bins1, ignored = ax1.hist(TTA_list_liuyc, 15, normed=True, color = "skyblue", alpha=0.7)
    # Plot the PDF.
    #xmin1, xmax1 = plt.xlim()
    xmin1, xmax1 = 1.5, 3.5
    print("xmin1 {0} and xmax1 {1}".format(xmin1, xmax1))
    x1 = np.linspace(xmin1, xmax1, 100)
    p1 = norm.pdf(x1, mu1, std1)
    ax1.plot(x1, p1, 'k', linewidth=2, label="PDF of LiuYC")
    print("this is x1 {0}, p1 {1} ".format(x1, p1))
    title = "Fit results for Driver TTA Distribution (LiuYC)"    
    ax1.legend(loc='best', frameon=True)
    ax1.set_title(title, fontsize=15)
    ax1.set_ylabel('Probability (P)')
    ax1.set_xlabel('TTA (s)')
    ax1.text(2.4,2.5,r"$(\mu = {0:.2f}, \sigma = {1:.2f})$".format(mu1, std1), fontsize=12)

    n2, bins2, ignored = ax2.hist(TTA_list_lukc, 15, normed=True, color = "skyblue", alpha=0.7)
    # Plot the PDF.
    xmin2, xmax2 = plt.xlim()
    print("xmin2 {0} and xmax2 {1}".format(xmin2, xmax2))
    x2 = np.linspace(xmin2, xmax2, 100)
    p2 = norm.pdf(x2, mu2, std2)
    ax2.plot(x2, p2, 'k', linewidth=2, label="PDF of LuKC")
    ax2.text(2.4,2.5,r"$(\mu = {0:.2f}, \sigma = {1:.2f})$".format(mu2, std2), fontsize=12)


    path = "/home/liuyc/moby_folder/Research/figures/intersection_simulator/driver_param/"
    #plt.savefig(path+'liuy_TTA_fitting.pdf')
    plt.show()

'''
Problem concerning the usage of "normed" or "density" arguments in hist method, check this discussion thread here : https://github.com/matplotlib/matplotlib/issues/10398/.

Long story short, histagram should use Probability Mass Function(PMF) rather than Probability Density Function(PDF). Both argument are making the sum of n (return of the hist method) as the area of the bins, similar to the area under the curve in PDF.
'''
