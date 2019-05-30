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


    #--- Parameters ---#
    ALPHA = 0.3
    R_MIN = 3    # meter
    TAU = 0.6    # s
    A_DEC = 3.4    # m/s^2
    SLOPE = 0.65   # y = 0.65x + 0.15
    #--- ========== ---#


    ###-------------------------------###
    ###---Time to Action Estimation---###
    ###-------------------------------###
    def CDF(self, TTC, v_car):

        #--- Get the estimated TTA ---#
        R_I = v_car**2 / (2 * A_DEC)
        TTA_est = (R_I + v_car*TAU + R_MIN ) / v_car
        TTA_act = TTA_est / SLOPE   # mean of the PDF
        std = TTA_act * 0.375/2    # standard deviation of the PDF
        cdf = norm(TTA_act, std).cdf(TTC)
        
        return cdf
    

    ###----------------------------------- ###
    ###------Probability of Stopping------ ###
    ###----------------------------------- ###
    def POS(self, min_TTC, TTC, TTC_p, time, time_p, v_car):
        
        #--- Variables ---#
        TTC_dif = (TTC - TTC_p) / (time - time_p)

        #--- GAMMA ---#
        if (TTC_dif + 1) < 0:
            GAMMA = 0
        else:
            GAMMA = (TTC_dif + 1) * ALPHA

        #--- Probability of Stopping ---#
        cdf_0 = CDF(min_TTC, v_car)
        judge_p_stop = (1 - cdf_0) * GAMMA
        if judge_p_stop > 1 :
            p_stop = 1
        else:
            p_stop = judge_p_stop

        return p_stop


    def rosbag_callback(self, pose):

        time_vel=[0.0, 0.0, .0]
        time_position = [0.0, 0.0, .0]

        self.car_vel[1] = pose.twist.twist.linear.x
        self.car_vel[2] = pose.twist.twist.linear.y
        self.car_posi[1] = pose.pose.pose.position.x
        self.car_posi[2] = pose.pose.pose.position.y

        current_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9
        
        # Plot from the moment they start to move
        if abs(pose.twist.twist.linear.y) > 0.1 or abs(pose.twist.twist.linear.x) > 0.1:
            
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

            # end when either one reaches the node ([0,0])
            # it's cheating now, car0 has to move along y-axis and car1 along x-axis
            if abs(self.car_posi[1]) < 0.1 or abs(self.car_posi[2]) < 0.1:
                self.count = 2


    def set_file_name(self, file_name, dir_name):
        self.file_name = file_name
        self.dir_name = dir_name


    def export_profile(self):
        print("{0}'s export_profile called".format(self.name)) 
        with open(r"/home/liuyc/moby_ws/intersection_simulator/src/prius_gazebo/scripts/data_analysis/txt_datas/20190523/{0}_{1}".format(self.file_name, self.name), "a") as f:
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
    prius1 = CarStates("prius1")
    prius0.set_file_name(file_name, dir_name)
    prius1.set_file_name(file_name, dir_name)

    def dummy_callback(pose, i):
        # callback from ros subscriber to instances
        if i == 0 : prius0.rosbag_callback(pose)
        elif i ==1 : prius1.rosbag_callback(pose)

    car_list = [prius0.name, prius1.name]
    for i in range(len(car_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(car_list[i]), Odometry,dummy_callback,(i))
    
    while not rospy.is_shutdown():
        try:
            if prius0.terminate() or prius1.terminate():
                prius1.export_profile()
                prius0.export_profile()
                rospy.signal_shutdown('The point of intersection has been reached.')
        
        except rospy.ROSInterruptException:
            pass

