#!/usr/bin/env python

# 2018 12 14 LiuYC SOLab
# Simple linear navigation for solabot

import roslib
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
### import modult for n-d array
from std_msgs.msg import Float32MultiArray
from costmap_module.numpy_nd_msg import numpy_nd_msg
from costmap_module import update 
#import costmap.update_costmap 
import time
### for debugging
import pdb



### global variables from "update.py" in costmap_module
car_vel = np.zeros((1, 2))
car_pose = update.car_pose
map_size = update.map_size
map_res = update.map_res
t_res = update.t_res
car_abs_vel = 0
last_dir = [0,0]

#################################################################################
# ================================ Navigation ================================= #
#################################################################################

### NOTE: not sure how to set the dimension of the matrix. For now, use the width of the intersection as the dimensions. (which is 80)

### DEF=> col_prob_matrix stands for: (1, 80)array in y-direction (col_prob_matrix[1]) from the current car_pose of every unit "future costmaps" (col_prob_matrix[0])


### Some default settings
col_prob_matrix = np.zeros((80, 80))

# determine the heading (x or y) of the car
### NOTE: Now ONLY x and y direction are considered
### TODO: Should use vector instead
def dir_xy(car_num = 0):  # now we control only one car
    global last_dir

    car_vel = update.car_vel

    if abs(car_vel[car_num][0]) > abs(car_vel[car_num][1]) and abs(car_vel[car_num][0]) > 0.01 :   # value of twist.linear.x is greater than that in y
        last_dir = [1,0]

    elif abs(car_vel[car_num][1]) > abs(car_vel[car_num][0]) and abs(car_vel[car_num][1]) > 0.01:   # value of twist.linear.y is greater than that in x
        last_dir = [0,1]

    return last_dir


def reset_prob_matrix():
    global col_prob_matrix 

    col_prob_matrix = np.zeros((80, 80))



def update_prob_matrix():
    
    # reset the matrix
    reset_prob_marix()

    pass



# Given a realworld coordinate return the collision probability
def get_col_prob(t, cor_lst):

    idx = update.pose_to_costcor(cor_lst)
    costmap = update.costmap

    col_prob = costmap[t][idx[0][1]][idx[0][0]]

    return col_prob


### how long to look into in the future, remember the time resolution t_res
# calculate the differentiate of collision probability on a coordinate in time t_res
def col_prob_diff(t0, cor_lst):
    
    prob0 = get_col_prob(t0, cor_lst)
    prob1 = get_col_prob(t0 + 20, cor_lst)  # +20 for 2 sec in the future

    prob_diff = prob1 - prob0
    
    return prob_diff



# the distance to cross the obstacle ahead (in meter)
def get_cross_dist(cor_list):

    cor_list_temp = cor_list.copy()
    dist2cross = 0
    notfindit = True

    while notfindit:
        
        # just make sure loop can end
        if dist2cross < int(np.ceil(map_size/map_res)):

            dist2cross += 1
            ### NOTE 1-D only!!!! ([0, map_res]), to +Y direction
            cor_list_temp[0] = cor_list_temp[0] + [0, map_res]
            
            ### no obstacle ahead or obstacle is leaving
            ### NOTE: and one more or : if obstacle is approaching but we can pass it too
            if get_col_prob(0, cor_list_temp) == 0 or col_prob_diff(0, cor_list_temp) < 0:

                notfindit = False

                return dist2cross * map_res 
            
            else:
                pass

        else:
            # raise error
            return -1



# the time from now to actual impact (col_prob == 1)
def get_impact_time(cor_list):

    costmap = update.costmap
    cor_list_temp = cor_list.copy()
    time_impact = 0
    notfindit = True

    while notfindit:

        if time_impact + 1 < costmap.shape[0]:

            time_impact += 1

        # NOTE 1-D only!!!! horizontal search only, search direction should be along vel-vector
            if get_col_prob(time_impact, cor_list_temp) == 1:
                notfindit = False

                return time_impact * t_res 

            else:
                pass

        else:
            return 0



#################################################################################
##===================== cmd_vel control setting ===============================##
#################################################################################

### Some default settings
# HERE we control the pub_rate as 10 while in joy_teleop it's 100
# SO the acceleration here is 10 times
accele = 0.5*t_res  #(0.05 m/s^2)
brake = accele*2.86
max_vel = 5.0
min_vel = 0.0 #(no reverse)



### Speed Functions

def decelerate(brake_rate=1):
    accele, min_vel
    global car_vel

    brake2 = brake*brake_rate

    if car_vel[0][1] - brake2 >= min_vel:
        car_vel[0][1] -= brake2

    else:   
        car_vel[0][1] = min_vel


def accelerate(accele_rate = 1):
    accele, max_vel
    global car_vel


    acc = accele * accele_rate
    if car_vel[0][1] + acc <= max_vel:
        car_vel[0][1] += acc

    else:
        car_vel[0][1] = max_vel


#################################################################################
##====================== END of cmd_vel control ===============================##
#################################################################################




def main():
    
    global car_vel, car_abs_vel
    
    obs_list = ['prius0']


# initialize the map
    robot_ns = 'prius1'

# car's initial movement 
    car_init_y_vel = rospy.get_param('/{0}_const_vel/init_vel'.format(robot_ns), 0.5) # default is 0.0
    car_vel[0][1] = car_init_y_vel

# Initialize the node    
    rospy.init_node('prius1_const_vel', anonymous=True)	
    
    # data of the "car" 
    pub_car_vel = rospy.Publisher('/prius1/cmd_vel', Twist, queue_size=5)


    rate = rospy.Rate(1/t_res) # default is 100


    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = car_init_y_vel; twist.linear.y =0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        
        pub_car_vel.publish(twist)

        rate.sleep()




if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
