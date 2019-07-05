#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import numpy as np
from scipy.stats import norm
from scipy.stats import probplot
from collections import OrderedDict
from pyexcel_ods import save_data
from pyexcel_ods import get_data

style.use('seaborn-whitegrid')

data = get_data("param_results.ods")

driver_name = 'all'
trial_num = 0
param = 'TTA'
params_dict = {'SLOPE': 1, 'A_DEC':2, 'R_MIN':3, 'TTA':4}
TTA_list = []

if driver_name != 'all':
    TTA_list = data["{0}_param_9".format(driver_name)][params_dict[param]]
    TTA_list = TTA_list[1:-2]
    #TTA_list_2 = data["{0}_param_9".format(driver_name_2)][7]
    #TTA_list_2 = TTA_list_2[1:21]


else:
    names=['liuyc', 'lukc', 'paul']
    for name in names:
        dum_list = data["{0}_param_9".format(name)][params_dict['TTA']]
        TTA_list += dum_list[1:-2]





# Fit a normal distribution to the data:
mu1, std1 = norm.fit(TTA_list)
#mu2, std2 = norm.fit(TTA_list_2)

plot_name = ['Participant_1', 'Participant_2', 'Participant_3', 'Combined_Data']



if driver_name == 'liuyc' : driver_name = plot_name[0]
elif driver_name == 'paul' : driver_name = plot_name[1]
elif driver_name == 'lukc' : driver_name = plot_name[2]
elif driver_name == 'all' : driver_name = plot_name[3]

if __name__ == '__main__':
    fig1 = plt.figure(1, figsize=(20,8))
    ax1 = fig1.add_subplot(121)   # (211)2 figs in 1 plot
    ax2 = fig1.add_subplot(122)

    ax1.clear()
    ax2.clear()

    #weight1 = np.ones(len(TTA_list1))/len(TTA_list1)

    n1, bins1, ignored = ax1.hist(TTA_list, 20, normed=True, color = "skyblue", alpha=0.7)
    # Plot the PDF.
    #xmin1, xmax1 = ax1.xlim()
    xmin1, xmax1 = 1., 4.
    #print("xmin1 {0} and xmax1 {1}".format(xmin1, xmax1))
    x1 = np.linspace(xmin1, xmax1, 100)
    p1 = norm.pdf(x1, mu1, std1)
    ax1.plot(x1, p1, 'k', linewidth=2, label="PDF of {1}({0})".format(driver_name, param.upper()))
    #print("this is x1 {0}, p1 {1} ".format(x1, p1))
    title1 = "Fit results for Driver {1} Distribution ({0})".format(driver_name, 'TFA')    
    ax1.legend(loc='best', frameon=True)
    ax1.set_title(title1, fontsize=18)
    ax1.set_ylabel('Probability (P)', fontsize = 15)
    ax1.set_xlabel('{0}'.format('TFA (s)'), fontsize = 15)
    ax1.text(2.85, 0.7,r"$(\mu = {0:.2f}, \sigma = {1:.2f}, N = {2})$".format(mu1, std1, len(TTA_list[1:-2])), fontsize=12)
    
# QQplot    
    res = probplot(TTA_list, plot=ax2)
    title2 = "Normal Quantile-Quantile Plot for {1}s of {0} ".format(driver_name, 'TFA')    
    ax2.set_title(title2, fontsize=18)
    ax2.set_ylabel('TFA data of {0} Quantiles'.format(driver_name), fontsize = 15)
    ax2.set_xlabel('Normal Theoretical Quantiles', fontsize = 15)
    ax2.get_lines()[0].set_marker('.')
    ax2.get_lines()[0].set_markerfacecolor('skyblue')
    ax2.get_lines()[0].set_markersize(10.0)
    ax2.get_lines()[1].set_linewidth(2.0)

    path = "/home/liuyc/moby_folder/Research/MasterThesis/figures/figs/"
    plt.savefig(path+'{0}_{1}_fitting.pdf'.format(driver_name, trial_num))
    plt.show()

'''
Problem concerning the usage of "normed" or "density" arguments in hist method, check this discussion thread here : https://github.com/matplotlib/matplotlib/issues/10398/.

Long story short, histagram should use Probability Mass Function(PMF) rather than Probability Density Function(PDF). Both argument are making the sum of n (return of the hist method) as the area of the bins, similar to the area under the curve in PDF.
'''
