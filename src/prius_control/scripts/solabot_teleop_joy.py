#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy




maxVel = 1.0
minVel = -maxVel
accele = 0.005
brake = accele*2.86
friction = accele/3
car_vel = 0.0

minAng = -1.5
maxAng = 1.5

# state of joy
throttleInit = False
brakeInit = False



# we set the input of acceleration and deceleration to have two input for multiple cars in the future

def fmap (toMap, in_min, in_max, out_min, out_max):

    return (toMap - in_min)*(out_max - out_min) / (in_max -in_min) + out_min;


def linearVel(v, joy_acc):

    accele_mapped = fmap(joy_acc, -1.0, 1.0, -accele, accele)
    v_cand = v + accele_mapped
    
    if accele_mapped >= 0.0:
        return min(v_cand, maxVel)

    else:
        return max(v_cand, minVel)


def stepBrake(v, joy_brake):
    
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



def carMotion(joy_data):
    global car_vel, throttleInit, brakeInit

    # Check if axes are triggered to avoid NoneType passing

    #print("throttleInit: {0}; brakeInit: {1}".format(joy_data.axes[3],joy_data.axes[1]))

    #===========================#
    #=== additional function ===#
    #===========================#



    #======================#
    #=== Linear Control ===#
    #======================#

    throttleInit = joy_data.axes[3] != 0
    brakeInit = joy_data.axes[1] != 0

    if throttleInit:
        car_vel = linearVel(car_vel, joy_data.axes[3])

    # It's hard to step on both throttle and brake
    elif brakeInit:
        car_vel = stepBrake(car_vel, joy_data.axes[1])

    else:
        car_vel = goSlide(car_vel)

    #=======================#
    #=== Angular Control ===#
    #=======================#
    
    # axes[0] is the left-horizontal axis

    car_ang = fmap(joy_data.axes[0], -1.0, 1.0, minAng, maxAng)
    

    #=======================#
    #=== Publish cmd_vel ===#
    #=======================#

    twist0 = Twist()
    twist0.linear.x = car_vel; twist0.linear.y = 0; twist0.linear.z = 0
    twist0.angular.x = 0; twist0.angular.y = 0; twist0.angular.z = car_ang

    pub.publish(twist0)



def main():
    global pub, maxVel, accele

    maxVel = rospy.get_param('/teleop_joy/max_velocity', 5.0) # default is 5.0
    accele = rospy.get_param('/toeleop_joy/acceleration', 0.035) # default is 0.035

    rospy.init_node('teleop_joy', anonymous=True)
    
    pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=5)
    joy_sub = rospy.Subscriber("/joy", Joy, carMotion)

    rate = rospy.Rate(100) # default is 10
   

    while True:
       

        rate.sleep()


if __name__ == '__main__':
    
    try:
        main()

    except rospy.ROSInterruptException:
        pass
