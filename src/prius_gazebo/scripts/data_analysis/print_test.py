#!/usr/bin/env python
from __future__ import print_function
import sys, os, time
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
    prevT = time.time()

    for i in range(10):
        while time.time() - prevT < 0.5:
            pass

        print("{0}\r".format(i), file=sys.stderr, end='')
        sys.stderr.flush()
        prevT = time.time()

    print()

    for i in range(10):
        while time.time() - prevT < 0.5:
            pass
        print("{0}\r".format(i), file=sys.stderr, end='')
        sys.stderr.flush()
        prevT = time.time()
    '''
    def timer(t=50):
        t0 = time.time()
        now = t0
        while now-t0 < t:
            now = time.time()
            timestr = '\r%%%i\t' %(100*(now-t0)/t)
            sys.stdout.write(timestr)
            sys.stdout.flush()
            time.sleep(0.1)
    '''
