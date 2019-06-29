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
    
    if car_vel[car_num][0] > 0.01 and car_vel[car_num][1] < 0.01:   # value of twist.linear.x is greater than that in y
        last_dir = [1,0]
        return [1,0]   # [x,y]

    elif car_vel[car_num][1] > 0.01 and car_vel[car_num][0] < 0.01:   # value of twist.linear.y is greater than that in x
        last_dir = [0,1]
        return [0,1]   # [x,y]

    else: return last_dir


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

accele = 0.005  #(0.075 m/s^2)
brake = accele*2.86
max_vel = 1.0
min_vel = 0.0 #(no reverse)



### Functions

def decelerate():
    accele, min_vel
    global car_vel

    if car_vel[0][1] - brake >= min_vel:
        car_vel[0][1] -= brake

    else:   
        car_vel[0][1] = min_vel


def accelerate():
    accele, max_vel
    global car_vel

    if car_vel[0][1] + accele <= max_vel:
        car_vel[0][1] += accele

    else:
        car_vel[0][1] = max_vel


#################################################################################
##====================== END of cmd_vel control ===============================##
#################################################################################




def main():
    
    global car_vel, car_abs_vel
    
    obs_list = ['prius0']


# initialize the map
    robot_ns = rospy.get_param('/prius1_nav/robot_ns') 
    map_res = rospy.get_param('/{0}_nav/cmap_res'.format(robot_ns), 1) # default is 1.0
    map_size = rospy.get_param('/{0}_nav/cmap_size'.format(robot_ns), 45) # default is 25
    update.init_map(map_res, map_size)

# car's initial movement 
    car_init_y_vel = rospy.get_param('/{0}_nav/init_vel'.format(robot_ns), 0.5) # default is 0.0
    car_vel[0][1] = car_init_y_vel

# Initialize the node    
    rospy.init_node('solabot_commands', anonymous=True)	
    
# ROS Publishers
    # publish as numpy array using numpy_msg
    pub_costmap = rospy.Publisher('/costmap1', numpy_nd_msg(Float32MultiArray), queue_size=5)
    # data of the "car" 
    pub_car_vel = rospy.Publisher('/{0}/cmd_vel'.format(robot_ns), Twist, queue_size=5)
    # pub the state of the ego vehicle
    pub_car_state = rospy.Publisher('/{0}/state'.format(robot_ns), numpy_nd_msg(Float32MultiArray), queue_size=5)
    
# ROS Subscribers
    rospy.Subscriber('/{0}/base_pose_ground_truth'.format(robot_ns), Odometry, update.update_car_odom)

    for i in range(len(obs_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(obs_list[i]), Odometry,update.update_obs_odom,(i))


    rate = rospy.Rate(100) # default is 100

    
    while not rospy.is_shutdown():

        try:
            pass

        finally:
            
#################################################################################
##====================== BEGIN of POS Algorithm ===============================##
#################################################################################
            ### update the costmap
            #pdb.set_trace()
            update.update_costmap()
            costmap = update.costmap
            pub_costmap.publish(data = costmap)

                
            ### Calculate the abs() of car_vel
            car_abs_vel = np.sqrt( np.sum( np.power(car_vel[0], 2)))
            


            ### NOTE different drive_mode ?  
            prob_thresh = 0.0  # smaller the thresh, more careful the driver is	    
            
            ### Avoid by prediction (lh_dist in meters)
            TTA_area = 3.5
            # TTA_area is the average TTA of the region; 
            #This lookahead distance is to mimic human's decision concerning TTC   
            #lh_dist = car_abs_vel*TTA_area + 0.5   #  + 0.5 just keep lh_pose become 0
            
            ### Optimal lookahead distance, human can't do this
            # look ahead distance: able to stop with full break
            lh_dist = 0.5 * car_abs_vel**2 / brake + 0.5 


            ### Get the col_prob
            ### NOTE: define car_vel[:][1] is throttle
            ### NOTE: so we have to know which direction it's heading
            
            direction = dir_xy()
            car_pose = update.car_pose
            
            if direction[0]:    #  moving in x-direction
                lh_pose = car_pose[0] + [lh_dist, 0]  
            
                ### Expand the lh_pose from a point to a range (from car to lh_pose), 
                ### to find out the closest lh_pose to obstacle (on costmap => prob > 1 ) 
                min_lh_pose = car_pose[0]
                max_lh_pose = lh_pose
                
                lh_x_range = np.arange(min_lh_pose[0], max_lh_pose[0], map_res)
                lh_pose_range = np.zeros((len(lh_x_range), 2))

                for x_num in range(len(lh_x_range)):

                    lh_pose_range[x_num][0] = lh_x_range[x_num]
                    lh_pose_range[x_num][1] = car_pose[0][1]

            

            else:    # moving in y-direction

                lh_pose = car_pose[0] + [0, lh_dist]        
            
                ### Expand the lh_pose from a point to a range (from car to lh_pose), 
                ### to find out the closest lh_pose to obstacle (on costmap => prob > 0 ) 
                min_lh_pose = car_pose[0]
                max_lh_pose = lh_pose
                
                lh_y_range = np.arange(min_lh_pose[1], max_lh_pose[1], map_res)
                lh_pose_range = np.zeros((len(lh_y_range), 2))

                for y_num in range(len(lh_y_range)):

                    lh_pose_range[y_num][0] = car_pose[0][0]
                    lh_pose_range[y_num][1] = lh_y_range[y_num]


            prob = 0
            unit_lh_pose = lh_pose_range[-1]

            for lh_p in lh_pose_range:

                cand_prob = get_col_prob(0, np.array([lh_p]))

                if cand_prob > 0:

                    prob = cand_prob
                    unit_lh_pose = lh_p

                    break
                
                else:
                    pass

            #--- slow down when approaching crossroad ---#
            
            obs_pose = update.obs_pose
            obs_vel = update.obs_vel

            d_node =np.sqrt( np.sum( np.power( np.subtract( obs_pose[0],[0,0]), 2)))
            v_obs =np.sqrt( np.sum( np.power(obs_vel[0], 2)))
            
#            if d_node < 5 and v_obs > 0.1:
                
#               decelerate()


            ### NOTE: Different drive mode??? now is energy-saving mode
            ### To avoid obstacle while try to maintain the initial(target)speed

            ### IF obstacle is coming
            if col_prob_diff(0, np.array([unit_lh_pose])) > 0: 
                print("Obstacle coming!")

                if costmap.shape[0] > 1:    # prediction mode
                    
                   # if col_prob_diff(0, np.array([unit_lh_pose])) > 0:    # obstacle approaching: is it ok to accelerate?

                        pos = update.POS()
                        if pos > 0.8:

                            accelerate(2)

                        ### TODO: checl there is an unavoidable collision 
                        else:
                            ### NOTE: +3 for the distance left afetr fully stopped  
                            cross_dist = get_cross_dist(np.array([unit_lh_pose])) + 5  # in meters

                            time_impact = get_impact_time(np.array([unit_lh_pose]))  # in seconds
                        
                            ### The car will collide with obs if it keeping at this car_vel
                            if cross_dist  - car_vel[0][1] * time_impact > 0:


                                ### NOTE: assume const. acceleration ! a = 1m/s (0.1m/t_res)
                                ### calculate the root of t: 1/2*a*t**2 + V_0*t - S = 0
                                roots = np.roots([1.0/2.0 * (accele), car_vel[0][1], - cross_dist])
                                ### Only one positive root since S > 0
                                root_t = np.amax(roots)

                                ### accelerate OR decelerate
                                if root_t < time_impact:
                                    
                                    accelerate()     # accelerate

                                else :
                                   
                                    decelerate()    # decelerate

                            else:
                                ### Keep moving can pass the obstacle
                                pass


                else:   ###NOTE: local planner!!! we don't have this yet

                    if prob == 1:
                        
                        decelerate()     # decelerate

            ### TODO: This is not a safe action!!!!!!!!
            elif col_prob_diff(0, np.array([unit_lh_pose])) < 0:    # obstacle leaving

                accelerate()     # accelerate

            elif prob == 1:
                
                decelerate()   # not going to move a bit
            # NO OBSTACLE AHEAD
            else:
                print("no obstacle ahead!!!!!!!!!!!!")
                
                if car_vel[0][1] + accele <= car_init_y_vel:

                    accelerate()   # accelerate

                elif car_vel[0][1] - brake >= car_init_y_vel:
                    
                    decelerate()   # decelerate

            ### Get car states
            # [0]: min_lh_pose; [1]: max_lh_pose; [2]: final lh_pose
            # [3]: prob at that final lh_pose; [4]: probability of stoppiong

            car_state = np.zeros((8,), dtype=np.float32)
            car_state[0] = min_lh_pose[0] 
            car_state[1] = min_lh_pose[1] 
            car_state[2] = max_lh_pose[0] 
            car_state[3] = max_lh_pose[1] 
            car_state[4] = unit_lh_pose[0] 
            car_state[5] = unit_lh_pose[1] 
            car_state[6] = prob  # it's risk actually 
            car_state[7] = update.POS()  # probability of stopping 


###############################################################################
##====================== END of POS Algorithm ===============================##
###############################################################################


            twist = Twist()
            twist.linear.x = car_vel[0][1]; twist.linear.y =0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            
            if True:  
                    pub_car_vel.publish(twist)
                    pub_car_state.publish(data = car_state)

            else:
                pass

            rate.sleep()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
