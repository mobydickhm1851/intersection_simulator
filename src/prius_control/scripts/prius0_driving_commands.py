#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
#from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

flag_move = 0
# HERE the publish rate is same as subscribe rate(which is t_res in update.py)


def set_throttle_steer(data):

    global flag_move


    pub_vel_left_rear_wheel = rospy.Publisher('/prius0/rear_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/prius0/rear_right_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/prius0/front_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/prius0/front_right_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/prius0/front_left_steer_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/prius0/front_right_steer_position_controller/command', Float64, queue_size=1)

    throttle = data.linear.x/0.3
    steer = data.angular.z/2.3

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)


def servo_commands():
    

    rospy.init_node('prius0_dirving_commands', anonymous=True)

    rospy.Subscriber("/prius0/cmd_vel", Twist, set_throttle_steer)

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
