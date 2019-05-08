#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time



maxVel = 1.0
minVel = -maxVel
accele = 0.005
brake = accele*1.3
friction = accele/3

car_vel = 0.0
car_ang = 0.0

minAng = -1.5
maxAng = 1.5

# state of joy
throttleInit = False
brakeInit = False

# gear state (D/N/R, sum(gear)=1/sum(gear)=0/sum(gear)=-1)
gear = [1,-1]

# we set the input of acceleration and deceleration to have two input for multiple cars in the future

# dir=true: shift up; dir=false: shift down
def shift_gear(dir_gear):
    global gear

    if dir_gear:   # shift up

        if sum(gear) == 1 : gear = [1,0]
        elif sum(gear) == 0 : gear = [1,0]
        elif sum(gear) == -1 : gear = [1,-1]

    else:   # shift down

        if sum(gear) == 1 : gear = [1,-1]
        elif sum(gear) == 0 : gear = [0,-1]
        elif sum(gear) == -1 : gear = [0,-1]



def fmap (toMap, in_min, in_max, out_min, out_max):

    return (toMap - in_min)*(out_max - out_min) / (in_max -in_min) + out_min;


def linearAcc(v, joy_acc):

    if sum(gear) == 1 :  # gear in D

        accele_mapped = fmap(joy_acc, 0, 1.0, 0, accele)
        v_cand = v + accele_mapped
        
        return min(v_cand, maxVel)


    elif sum(gear) == 0 :  # gear in N

        
        return v


    elif sum(gear) == -1 :  # gear in R

        accele_mapped = fmap(joy_acc, 0, 1.0, 0, accele)
        v_cand = v - accele_mapped
        
        return max(v_cand, minVel)


def stepBrake(v, joy_brake):
    
    brake = 2.86*accele
    # Left-axes down as brake, otherwise wont affect the cmd_vel 
    if joy_brake < 0:

        brake_mapped = fmap(joy_brake, -1.0, 0, -brake, 0)

        if v >= 0:
            v_cand = v + brake_mapped
            return max(v_cand, 0)
        
        elif v < 0:
            v_cand = v - brake_mapped
            return min(v_cand, 0)
        
    
    else:
        return v


def goSlide(v):

    if v >= 0:
        v_cand = v - friction
        return max(v_cand, 0)

    elif v < 0:
        v_cand = v + friction
        return min(v_cand, 0)

# initialize time stamp
last_time_stamp = time.time()

def carMotion(joy_data):
    global car_vel, car_ang, throttleInit, brakeInit, last_time_stamp

    # Check if axes are triggered to avoid NoneType passing

    #print("throttleInit: {0}; brakeInit: {1}".format(joy_data.axes[3],joy_data.axes[1]))

    #===========================#
    #=== additional function ===#
    #===========================#


    #=====================#
    #=== Shifting Gear ===#
    #=====================#

    # not continuous publish
    now_time_stamp = time.time() 
    time_diff = now_time_stamp - last_time_stamp

    min_time = 0.3

    if joy_data.buttons[6] and time_diff >= min_time :

        shift_gear(False)
        last_time_stamp = time.time()

    elif joy_data.buttons[7] and time_diff >= min_time :

        shift_gear(True)
        last_time_stamp = time.time()

    


    #======================#
    #=== Linear Control ===#
    #======================#

    throttleInit = joy_data.axes[3] > 0
    brakeInit = joy_data.axes[3] < 0

    if throttleInit:
        car_vel = linearAcc(car_vel, joy_data.axes[3])

    # It's hard to step on both throttle and brake
    elif brakeInit:
        car_vel = stepBrake(car_vel, joy_data.axes[3])

    else:
        car_vel = goSlide(car_vel)

    #=======================#
    #=== Angular Control ===#
    #=======================#
    
    # axes[0] is the left-horizontal axis

    car_ang = fmap(joy_data.axes[0], -1.0, 1.0, minAng, maxAng)
    


def main():
    global pub, maxVel, accele

    robot_ns = rospy.get_param('/prius0_teleop_joy/robot_ns') 
    maxVel = rospy.get_param('/{0}_teleop_joy/max_velocity'.format(robot_ns), 5.0) # default is 5.0
    accele = rospy.get_param('/{0}_teleop_joy/acceleration'.format(robot_ns), 0.035) # default is 0.035

    rospy.init_node('{0}_teleop_joy'.format(robot_ns), anonymous=True)
    
    pub = rospy.Publisher('/{0}/cmd_vel'.format(robot_ns), Twist, queue_size=5)
    joy_sub = rospy.Subscriber("/{0}/joy".format(robot_ns), Joy, carMotion)

    rate = rospy.Rate(100) # default is 10
   

    while not rospy.is_shutdown():
    
        #=======================#
        #=== Publish cmd_vel ===#
        #=======================#

        twist0 = Twist()
        twist0.linear.x = car_vel; twist0.linear.y = 0; twist0.linear.z = 0
        twist0.angular.x = 0; twist0.angular.y = 0; twist0.angular.z = car_ang

        pub.publish(twist0)


        if sum(gear) == 1: mode = 1
        elif sum(gear) == 0: mode = 0
        elif sum(gear) == -1: mode = -1

        #rospy.loginfo("Now the gear mode is:".format(mode))
        print("Now the gear mode of {1} is:{0}".format(mode, robot_ns))

        rate.sleep()


if __name__ == '__main__':
    
    try:
        main()

    except rospy.ROSInterruptException:
        pass
