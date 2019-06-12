#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm




class CarStates:

    def __init__(self, name):
        
        self.name = name
        self.file_name = "defult_file_name"
        self.dir_name = "defult_dir_name"
        # timestamp, car_vel in major direction
        self.vel_profile = np.zeros([1,3])   # velocity profile [t, vx, vy] 
        self.posi_profile = np.zeros([1,3])   # position profile [t, px, py]
        # CURRENT car states 
        self.car_vel = [0,0,0]
        self.car_posi = [0,0,0]
        # define as the timestamp when the cars begin to move
        self.start_time = 0.0   
        # record state, becomes 1 if start to add to profiles
        self.count = 0
        self.exported = False


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

        time_vel=[0.0, 0.0, .0]
        time_position = [0.0, 0.0, .0]

        self.car_vel[1] = pose.twist.twist.linear.x
        self.car_vel[2] = pose.twist.twist.linear.y
        self.car_posi[1] = pose.pose.pose.position.x
        self.car_posi[2] = pose.pose.pose.position.y

        current_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9
        
        # Plot from the moment they start to move
        if abs(pose.twist.twist.linear.y) > 0.1 or abs(pose.twist.twist.linear.x) > 0.1 or self.count == 1:
            
            if self.count == 0:
                self.start_time = current_time
                self.count = 1

            elif self.count == 1:
                time_vel[0] = current_time - self.start_time
                time_position[0] = current_time - self.start_time
                time_vel[1] = self.car_vel[1]
                time_vel[2] = self.car_vel[2]
                time_position[1] = self.car_posi[1]
                time_position[2] = self.car_posi[2]
                self.vel_profile = np.append(self.vel_profile, [time_vel], 0)
                self.posi_profile = np.append(self.posi_profile, [time_position], 0)


    # for param_test, car0 is stopped near the intersection in X-DIR
            if abs(self.car_posi[1]) < 10  and abs(self.car_vel[1]) < 0.01:
                self.count = 2
                print("Ending as prius0 stopped moving")


    def set_file_name(self, file_name, dir_name):
        self.file_name = file_name
        self.dir_name = dir_name


    def export_profile(self):
        print("{0}'s export_profile called".format(self.name)) 
        with open(r"/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/{2}/{0}_{1}".format(self.file_name, self.name, self.dir_name), "a") as f:
            # INCASE cought in loops
            if not self.exported :

                first_row = ["timestamp, {0} x posi, {0} y posi, {0} x vel, {0} y vel.\n".format(self.name)]
                f.writelines(first_row) 
                print("{0}'s first row written".format(self.name)) 

                time_list = list(self.vel_profile[:,0])
                car_x_posi = list(self.posi_profile[:,1])
                car_y_posi = list(self.posi_profile[:,2])
                car_x_vel = list(self.vel_profile[:,1])
                car_y_vel = list(self.vel_profile[:,2])
                for i in range(len(time_list)):
                    print("{0}'s {1} row written".format(self.name, i)) 
                    dummy_row = ["{0}, {1}, {2}, {3}, {4}\n".format(time_list[i], car_x_posi[i], car_y_posi[i], car_x_vel[i], car_y_vel[i])]
                    f.writelines(dummy_row) 
                
                self.exported = True


    def terminate(self):
        if self.count == 2 : 
            return True



if __name__ == '__main__':

    # ROS subscriber Section
    rospy.init_node('rosbag_plot', anonymous=True)
    file_name = rospy.get_param('/rosbag_data_writer/file_name', "param_not_exist") 
    dir_name = rospy.get_param('/rosbag_data_writer/dir_name', "param_not_exist") 

    # Instance, two cars
    prius0 = CarStates("prius0")
    prius0.set_file_name(file_name, dir_name)

    def dummy_callback(pose):
        # callback from ros subscriber to instances
        prius0.rosbag_callback(pose)

    car_list = [prius0.name]
    rospy.Subscriber('/{0}/base_pose_ground_truth'.format(car_list[0]), Odometry,dummy_callback)
    
    while not rospy.is_shutdown():
        try:
            print(prius0.count)
            if prius0.terminate() :
                prius0.export_profile()
                rospy.signal_shutdown('The param test is done.')
        
        except rospy.ROSInterruptException:
            pass
