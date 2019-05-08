#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
from solabot_challenge import *


minVel = -7.0
maxVel = 7.0
accele = 0.075
brake = accele*1.86
friction = accele/2
keyPressed = {}
car0_vel = 0
car1_vel = 0

def goForward(v):
    v_cand = v + accele
    return min(v_cand, maxVel)
    
def stepBrake(v):
    v_cand = v - brake
    return max(v_cand, 0)

def goBackward(v):
    v_cand = v - accele
    return max(v_cand, minVel)

def goSlide(v):
    v_cand = v - friction
    return max(v_cand, 0)


def carMotion():

    global car0_vel, car1_vel
    # true if there is no key been pressed
    checkDict = all(value == False for value in keyPressed.values())

    print(checkDict)

    if  checkDict:
        car0_vel = goSlide(car0_vel) 
        car1_vel = goSlide(car1_vel) 

    else:
        if keyPressed["w"] : car0_vel = goForward(car0_vel)
        if keyPressed["s"] : car0_vel = stepBrake(car0_vel)
        if keyPressed["x"] : car0_vel = goBackward(car0_vel)
        if keyPressed["o"] : car1_vel = goForward(car1_vel)
        if keyPressed["l"] : car1_vel = stepBrake(car1_vel)
        if keyPressed["."] : car1_vel = goBackward(car1_vel)


    twist0 = Twist()
    twist0.linear.x = 0; twist0.linear.y = car0_vel; twist0.linear.z = 0
    twist0.angular.x = 0; twist0.angular.y = 0; twist0.angular.z = 0
    pub0.publish(twist0)

    twist1 = Twist()
    twist1.linear.x = 0; twist1.linear.y = car1_vel; twist1.linear.z = 0
    twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = 0
    pub1.publish(twist1)

def main():

    global keyPressed, pub0, pub1

    keyPressed={"w": False, "s": False, "x": False, "o": False, "l": False, ".": False}

    rospy.init_node('solabot_challenge_control', anonymous=True)
    pub0 = rospy.Publisher('/car0/cmd_vel', Twist, queue_size=5)
    pub1 = rospy.Publisher('/car1/cmd_vel', Twist, queue_size=5)
    rate = rospy.Rate(50) # default is 10
    
    pygame.display.set_caption("SOLabot Challenge!!")
    drawGrid()
    showStartScreen()
    showTutorial()
    countDown() 

    while True:
        print(keyPressed)

        for event in pygame.event.get() :
            if event.type == pygame.KEYDOWN :
                if event.key == pygame.K_w : keyPressed["w"] = True        
                elif event.key == pygame.K_s : keyPressed["s"] = True 
                elif event.key == pygame.K_x : keyPressed["x"] = True                 
                elif event.key == pygame.K_o : keyPressed["o"] = True 
                elif event.key == pygame.K_l : keyPressed["l"] = True 
                elif event.key == pygame.K_PERIOD : keyPressed["."] = True 

            elif event.type == pygame.KEYUP :
                if event.key == pygame.K_w : keyPressed["w"] = False        
                elif event.key == pygame.K_s : keyPressed["s"] = False 
                elif event.key == pygame.K_x : keyPressed["x"] = False                
                elif event.key == pygame.K_o : keyPressed["o"] = False
                elif event.key == pygame.K_l : keyPressed["l"] = False
                elif event.key == pygame.K_PERIOD : keyPressed["."] = False

            elif event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        carMotion()

        rate.sleep()


if __name__ == '__main__':
    
    try:
        main()

    except Exception as e:
        print(e)
