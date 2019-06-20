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


class This:
    a = 1
    def method1(self):
        self.a = self.a + 1

if __name__ == '__main__':
 
    test = This()
    print("test.a = ", test.a)
    test.method1()
    print("test.a = ", test.a)
    print("This.a = ", This.a)
 
