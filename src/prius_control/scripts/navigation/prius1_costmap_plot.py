#!/usr/bin/env python

# 2018 12 18 LiuYC SOLab
# Plotting costmap

import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import Float32MultiArray
# import modult for n-d array
from nav_msgs.msg import Odometry
from costmap_module.numpy_nd_msg import numpy_nd_msg
from costmap_module import update
from matplotlib.figure import figaspect
import matplotlib.gridspec as gridspec

# debug
import pdb


# initialize the map
map_res = rospy.get_param('/prius1_costmap_plot/cmap_res', 1) # default is 1.0
map_size = rospy.get_param('/prius1_costmap_plot/cmap_size', 45) # default is 45
update.init_map(map_res, map_size)

costmap = np.array(0.2 * np.random.randn(2, map_size/map_res, map_size/map_res) + 0.5, dtype=np.float32)

car_pose_update = update.car_pose

car_state = np.zeros((8,), dtype=np.float32)


def initial_plot(arr):
# using costmap_plot() to plot the background first (with resolution of (map_size/map_res)^2), then in update_plot the color(zz, as probability) is updated.(FASTER than replotting all over)
    
    plt.subplot(gs[0])
    
    
    #--------------------------#
    #-- Costmap size default --#
    #--------------------------#
    x_plt = np.zeros((1, arr.shape[1]))[0]
    y_plt = np.arange( arr.shape[1])

    for i in range(arr.shape[0]-1):
        x_plt = np.append(x_plt, np.ones((1,arr.shape[1]))[0]*i) 
        y_plt = np.append(y_plt, np.arange(arr.shape[1])[::-1])
    
    zz = arr.transpose().reshape(1,-1)[0]


    #------------------------------#
    #-- Scatter initial plotting --#
    #------------------------------#
    scat = plt.scatter(x_plt, y_plt, c=zz, marker="s",edgecolors="none", s = 10, alpha = 0.5, cmap='Reds', vmin=0, vmax=1) 

    scat_car_pose = plt.scatter(car_pose_update[0][0], car_pose_update[0][1], c='blue', marker='o', s = 20, alpha = 1, label='car1')


    #---------------#
    #-- Animation --#
    #---------------#
    # last ',' in fargs is to prevent the arg been passed unpacked, instead of as an arg
    ani = animation.FuncAnimation(fig, plot_update, fargs=(scat, scat_car_pose,), interval = 10)


    #---------------------------------#
    #-- Plotting titles and legends --#
    #---------------------------------#
    plt.title("car1's Costmap for Obsticles", fontsize=20)
    plt.legend(scatterpoints = 1, loc='lower left')
    cbar = plt.colorbar(scat)
    cbar.set_label('Risks')

    sub1 = plt.subplot(gs[1])

    plt.show()



def plot_update(i, scat, scat_car_pose):
    global car_pose_update, costmap , tryme

    ###----------------###
    ### Costmap update ###
    ###----------------###
    arr = costmap[0]
    zz = arr.transpose().reshape(1,-1)[0]

    # set_array control the "color array"
    scat.set_array(zz)

    ###-----------------###
    ### Car pose update ###
    ###-----------------###
    # get car_pose from subscriber
    car_pose_update = update.car_pose
    car_idx = update.pose_to_costcor(car_pose_update)

    # inverse y
    car_idx[0][1] = map_size/map_res - car_idx[0][1] 
    scat_car_pose.set_offsets([car_idx])

    # debug
    #pdb.set_trace()

    #plt.delaxes(gs[1])

    plt.subplot(gs[1])
    plt.cla()
    plt.subplot(gs[1])

    ###---------------------------------------------------------###
    ### Showing ego vehicle state, risk and probability of stop ###
    ###---------------------------------------------------------###
    text_range = "Lookahead range: [{0:.2f},{1:.2f}] - [{2:.2f},{3:.2f}]".format(car_state[0], car_state[1], car_state[2], car_state[3])
    text_final_pose = "Final lookahead pose: [{0:.2f},{1:.2f}]".format(car_state[4], car_state[5])
    text_risk = "Risk at the pose : {0:.2f}".format(car_state[6])
    text_prob = "Car0's probability of stopping : {0:.2f}".format(car_state[7])

    plt.text(0.05, 0.20, text_range, va='center')
    plt.text(0.05, 0.40, text_final_pose, va='center')
    plt.text(0.05, 0.60, text_risk, va='center')
    plt.text(0.05, 0.80, text_prob, va='center', fontsize = 14)

    #---------------------------------#
    #-- Plotting titles and legends --#
    #---------------------------------#
    plt.title("car1's States", fontsize=18)
    plt.axis('off')


def costmap_update(raw_arr):
    global costmap, map_size, map_res
    
    costmap = raw_arr.data


def car_state_update(raw_data):
    global car_state
    
    car_state = raw_data.data




if __name__ == '__main__':
    
    rospy.init_node('costmap_plot', anonymous=True)	
    robot_ns = rospy.get_param('/prius1_costmap_plot/robot_ns') 
    rospy.Subscriber('/costmap1', numpy_nd_msg(Float32MultiArray), costmap_update)
    rospy.Subscriber('/{0}/state'.format(robot_ns), numpy_nd_msg(Float32MultiArray), car_state_update)
    rospy.Subscriber('/{0}/base_pose_ground_truth'.format(robot_ns), Odometry, update.update_car_odom)

    #---------------------------#
    #-- Initialize the figure --#
    #---------------------------#
    
    fig = plt.figure(figsize=(6,7))
    

    #-----------------------------#
    #-- Subplot default setting --#
    #-----------------------------#
    gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])


    initial_plot(costmap[0])


    while not rospy.is_shutdown():

        try:
            pass 

        except rospy.ROSInterruptException:
            pass

