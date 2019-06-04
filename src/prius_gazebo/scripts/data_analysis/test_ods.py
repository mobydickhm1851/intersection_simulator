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

sub_data = []
data = OrderedDict()

sub_data = [range(20), [4,5,6], [7,8,9]]
sub_data_arr = np.array(sub_data)
print(sub_data_arr.shape)

print(sub_data_arr)


data.append({"hahahahha":sub_data_arr})
save_data("your_file.ods", data)

print(data)

if __name__ == '__main__':
    pass
