#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm


vel_profile_0 = np.array([[0, 0]])   # timestamp, car0_y
vel_profile_1 = np.array([[0, 0]])   # timestamp, car1_x
position_profile_0 = np.array([[0, -15]])   # timestamp, car0_y
position_profile_1 = np.array([[0, -15]])   # timestamp, car1_x
start_time = 0.0   # define as the timestamp when the cars begin to move
count = 0
car_dirs=np.array([[0,0],[0,0]])



class CarStates:

    def __init__(self, name):
        
        self.name = name
        # timestamp, car_vel in major direction
        self.vel_profile = np.array([[0, 0]])   # velocity profile 
        self.posi_profile = np.array([[0, 0]])   # position profile
        # CURRENT car states 
        self.car_vel = [0,0]
        self.car_posi = [0,0]
        # define as the timestamp when the cars begin to move
        self.start_time = 0.0   
        # record state, becomes 1 if start to add to profiles
        self.count = 0


    # determine the heading (x or y) of the car
    ### NOTE: Now ONLY x and y direction are considered
    ### TODO: Should use vector instead
    def dir_xy(self):  # now we control only one car

        last_dir=[0,0]
        
        # value of twist.linear.x is greater than that in y
        if abs(self.car_vel[0]) > abs(self.car_vel[1]) :   
            last_dir = [1,0]

        # value of twist.linear.y is greater than that in x
        elif abs(self.car_vel[1]) > abs(self.car_vel[0]) :   
            last_dir = [0,1]

        return last_dir   # [0,0] for not moving


    def rosbag_callback(self, pose):

        start_time = .0
        time_vel=[0.0, 0.0]
        time_position = [0.0, 0.0]

        self.car_vel[0] = pose.twist.twist.linear.x
        self.car_vel[1] = pose.twist.twist.linear.y
        self.car_posi[0] = pose.pose.pose.position.x
        self.car_posi[1] = pose.pose.pose.position.y

        current_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9

        # Plot from the moment they start to move
        if abs(pose.twist.twist.linear.y) > 0.1 or abs(pose.twist.twist.linear.x) > 0.1:
            
            if self.count == 0:
                start_time = current_time
                self.count = 1

            time_vel[0] = current_time - start_time
            time_position[0] = current_time - start_time
            dir_check = self.dir_xy()

            if dir_check[0] and not dir_check[1]:   # moving in x direction
                time_vel[1] = self.car_vel[0]
                time_position[1] = self.car_posi[0]
            elif dir_check[1] and not dir_check[0]:   # moving in y direction
                time_vel[1] = self.car_vel[1]
                time_position[1] = self.car_posi[1]
            else:   # not moving
                print("{0} is not moving yet.".format(self.name))

            self.vel_profile = np.append(self.vel_profile, [time_vel], 0)
            self.posi_profile = np.append(self.posi_profile, [time_position], 0)

            # end when either one reaches the node ([0,0])
            # it's cheating now, car0 has to move along y-axis and car1 along x-axis
            if abs(self.car_posi[0]) < 0.1 or abs(self.car_posi[1]) < 0.1:
                self.count = 2

        
    def terminate(self):

        if self.count == 2 : 
            print("{0} position profile : {1}".format(self.name, self.posi_profile))
            rospy.signal_shutdown('Node reached by {0}'.format(self.name))




# Instance, two cars
prius0 = CarStates("prius0")
prius1 = CarStates("prius1")


def dummy_callback(pose, i):
    # callback from ros subscriber to instances
    if i == 0 : prius0.rosbag_callback(pose)
    elif i ==1 : prius1.rosbag_callback(pose)




if __name__ == '__main__':

    # ROS subscriber Section
    rospy.init_node('rosbag_plot', anonymous=True)
    
    car_list = [prius0.name, prius1.name]
    for i in range(len(car_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(car_list[i]), Odometry,dummy_callback,(i))


    
    while not rospy.is_shutdown():
        try:
            prius0.terminate()
            prius1.terminate()
        
        except rospy.ROSInterruptException:
            pass

