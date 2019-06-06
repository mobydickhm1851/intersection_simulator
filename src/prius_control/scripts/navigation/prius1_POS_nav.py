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
accele = 0.5*t_res  #(0.075 m/s^2)
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


    rate = rospy.Rate(1/t_res) # default is 100



    def cruise(accele_rate = 1):
        global car_vel
        
        if car_vel[0][1] + accele <= car_init_y_vel:
            accelerate(accele_rate)   # accelerate

        elif car_vel[0][1] - brake >= car_init_y_vel:
            decelerate()   # decelerate

    
    while not rospy.is_shutdown():
        ### update the costmap
        #pdb.set_trace()
        update.update_costmap()
        costmap = update.costmap
        pub_costmap.publish(data = costmap)
                    
        # FIRST check if any obstacle is around (within some distance)
        ## Here we assume there is always an obs "prius0"
        ## Otherwise there should be an FOR LOOP for this
        if len(obs_list) == 0:
            # Keep the default speed

            print("no obstacle around!")
            
            cruise()

            twist = Twist()
            twist.linear.x = car_vel[0][1]; twist.linear.y =0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            
            pub_car_vel.publish(twist)

            rate.sleep()

            continue   # No need to calculate POS and change speed
            

        else:

            ####################################
            #CHECK if there is any intersection#
            ####################################
            # if there is any point of intersection:
            #   FIND THE INTERSECT     (we pass this step here)
            print ("Obs exists!!!!!")

            car_pose = update.car_pose
            car_vel_twist = update.car_vel  # GLOBAL car_vel is positive
            
            # IS the obs near us?
            # CHECK if there is obstacle within critical distance
            ## Calculate the abs() of car_vel
            car_abs_vel = np.sqrt( np.sum( np.power(car_vel_twist[0], 2)))

            # TTA_area is the average TTA of the region; 
            TTA_area = 3.5

            #1# This lookahead distance is to mimic human's decision using TTC   
            #  + 0.5 just keep lh_pose become 0
            #lh_dist = car_abs_vel*TTA_area + 0.5   
            
            #2# Optimal lookahead distance, human can't do this 
            #(able to stop with full break)
            lh_dist = 0.5 * car_abs_vel**2 / (brake/t_res) + 1.8 + .5 
            # 0.3 is to compansate the inaccurate car_dim and obs_dim
            # 0.5 is the safety margin 
            
            ## Get the pose to get the col_prob
            # NOTE: define car_vel[:][1] is throttle
            # NOTE: so we have to know which direction it's heading
            direction = dir_xy()

            #print("car_pose[0]={0}; lh_dist={1}; car_vel[0]={2}; car_abs_vel = {3}".format(car_pose[0], lh_dist, car_vel[0], car_abs_vel)) 
            #print("last_dir = {0}".format(last_dir)) 
            if direction[0]:    #  moving in x-direction
                if car_abs_vel > 0.01:   # car_vel is not 0
                    lh_pose = car_pose[0] + np.array([lh_dist, 0])*(car_vel_twist[0][0]/car_abs_vel)  
                else:   # car_vel is 0
                    lh_pose = car_pose[0] + 0.1*(car_vel_twist[0][0]/car_abs_vel)
            
                # Expand the lh_pose from a point to a range 
                #(from car to lh_pose)
                #to find out the closest lh_pose to obstacle 
                #(on costmap => prob > 1 ) 
                min_lh_pose = car_pose[0]
                max_lh_pose = lh_pose
                ## SEE who is smaller, first element in arange() is smaller 
                if min_lh_pose[0] < max_lh_pose[0]:
                    lh_x_range = np.arange(min_lh_pose[0], max_lh_pose[0], map_res)
                else :
                    lh_x_range = np.arange(max_lh_pose[0], min_lh_pose[0], map_res)

                lh_pose_range = np.zeros((len(lh_x_range), 2))

                for x_num in range(len(lh_x_range)):

                    lh_pose_range[x_num][0] = lh_x_range[x_num]
                    lh_pose_range[x_num][1] = car_pose[0][1]


            else:    # moving in y-direction
                
                #print("car_abs_vel {0}; lh_dist {1}".format(car_abs_vel, lh_dist))
                #print("car_vel_twist {0}".format(car_vel_twist[0]))
                #print("car_twist/car {0}".format((car_vel_twist[0][1]/car_abs_vel)))
                #print("car_pose[0] {0}".format(car_pose[0]))
                if car_abs_vel > 0.01:   # car_vel is not 0
                    lh_pose = car_pose[0] + np.array([0, lh_dist])*(car_vel_twist[0][1]/car_abs_vel) 
                else:   # car_vel is 0
                    lh_pose = car_pose[0] + 0.1*(car_vel_twist[0][0]/car_abs_vel)
                #Expand the lh_pose from a point to a range 
                #(from car to lh_pose), 
                #to find out the closest lh_pose to obstacle
                #(on costmap => prob > 0 ) 
                min_lh_pose = car_pose[0]
                max_lh_pose = lh_pose
                
                ## SEE who is smaller, first element in arange() is smaller 
                if min_lh_pose[1] < max_lh_pose[1]:
                    lh_y_range = np.arange(min_lh_pose[1], max_lh_pose[1], map_res)
                else :
                    lh_y_range = np.arange(max_lh_pose[1], min_lh_pose[1], map_res)

                lh_pose_range = np.zeros((len(lh_y_range), 2))

                for y_num in range(len(lh_y_range)):

                    lh_pose_range[y_num][0] = car_pose[0][0]
                    lh_pose_range[y_num][1] = lh_y_range[y_num]

        
            # AFTER we got where to look at
            # check if we need to slow down
            ## DEFINE the emergency brake prob
            emerg_prob = 0.8   
            prob = 0.0
            prob_future = 0.0
            unit_lh_pose = car_pose[0]
            lh_time = car_abs_vel/(brake/t_res)
            #lh_time = 0


            for lh_p in lh_pose_range:
                cand_prob = get_col_prob(0, np.array([lh_p]))   #2 secs:sec/t_res
                if cand_prob > 0.0:
                    prob = cand_prob
                    unit_lh_pose = lh_p
                    break
                else:
                    pass

            # also look into the future
            for lh_p in lh_pose_range:
                cand_prob = get_col_prob(lh_time, np.array([lh_p]))   #2 secs:sec/t_res
                if cand_prob > 0.0:
                    prob_future = cand_prob
                    break
                else:
                    pass



            #print("ego car pose {0} and vel {1}".format(car_pose[0], car_abs_vel))
            #print("[{0}] Obs at {1} with prob {2}.".format(time.time(), unit_lh_pose, prob))
            #print("Final cmd speed {0}; lh_dist {1}; lh_time {2}".format(car_vel[0], lh_dist, lh_time))


            # CRITICAL situation, Need brake to avoide collision
            if prob or prob_future >= emerg_prob:
                decelerate(2)
                print("Braking Just in Case")

            else:
                # THEN check if the obstacle is coming or leaving 
                obs_vel = update.obs_vel 
                obs_pose = update.obs_pose 
                # If it's coming, vel*pose < 0
                dir_mat = obs_vel[0]*obs_pose[0] 
                dir_test = 0     # 1 for obs coming; 0 for obs leaving the node


                if dir_mat[0]<-0.1 or dir_mat[1]<-0.1:
                    dir_test = 1   # car is coming
                
                elif dir_mat[0]>-0.1 and dir_mat[0]<0.1 and dir_mat[1]>-0.1 and dir_mat[1]<0.1:   # car is not moving
                    dir_test = 2

                # TTC_thresh as how careful
                TTC_thresh = 2
                obs_pose = update.obs_pose
                obs_vel = update.obs_vel
                obs_d_node =np.sqrt( np.sum( np.power( np.subtract( obs_pose[0],[0,0]), 2)))
                obs_abs_vel =np.sqrt( np.sum( np.power(obs_vel[0], 2)))
                car_d_node =np.sqrt( np.sum( np.power( np.subtract( car_pose[0],[0,0]), 2)))
                obs_TTC = obs_d_node / obs_abs_vel
                car_TTC = car_d_node / car_abs_vel


                # SLOW DOWN when it's near the crossroad
                #print("car_d_node is {0}; obs_d_node is {1}".format(car_d_node, obs_d_node))
                if car_d_node < 15 and car_d_node > 12 and car_abs_vel >= 2.5:
                    decelerate(2)
                    print("Slow down near the crossroad!")


                if dir_test == 1 and car_d_node < 16:    # dir_test == TRUE if obs is coming
                    

                    # CHECK the d-TTC here to see if collision is about to happen
                    if abs(obs_TTC - car_TTC) < TTC_thresh:   # collision might happen
                        
                        ## USING POS to predict what maneuver the obs will take 
                        pos = update.POS()
                        ## DEFINE a confidence index we believe it will STOP
                        pos_conf_idx = 0.8
                        
                        if pos > pos_conf_idx:
                            cruise(3)
                            print("POS = {1} is higher than {0}".format(pos_conf_idx, pos))
    
                        else : 
                            # BRAKE a little bit to see what will happend
                            decelerate(2)
                            print("Brake due to POS is {0}".format(pos))

                    else:    # 
                        cruise(3)
                        print("Obstacle is coming!!!!")



                elif dir_test == 0 :   # dir_test == FALSE if obs is leaving

                    if prob >= 0.5:   # DONt wanna frighten human
                        decelerate()
                        print("Brake to easy humans! prob = {0}".format(prob))
                    
                    else:    
                        cruise()
                        print("Obstacle is leaving!")


                elif dir_test == 2 :    # car is not moving

                    cruise(5)
                    print("Obstacle is not MOVING")



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

            twist = Twist()
            twist.linear.x = car_vel[0][1]; twist.linear.y =0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            pub_car_vel.publish(twist)
            pub_car_state.publish(data = car_state)

            rate.sleep()




if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
