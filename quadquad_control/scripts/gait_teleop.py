#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import pygame

"""
This is a simple teleop program to control the quad
"""

pygame.init()
pygame.display.set_mode((100, 100))



def main():
  rospy.init_node('quad_teleop_controller')

  speed = 0.0
  direction = 0.0

  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  rate = rospy.Rate(10) # 10hz

  rospy.loginfo(rospy.get_caller_id() + " created gait teleop manager")


  while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
            key = event.key
            isdown = 1 if (event.type == pygame.KEYDOWN) else -1

            elif key == 274: #Down arrow
                speed-=1.0*isdown
            elif key == 273: #Up arrow
                speed+=1.0*isdown
            elif key == 275: #right arroww
                direction+=1.0*isdown
            elif key == 276: #left arrow
                direction-=1.0*isdown

            t = Twist()
            t.linear.x = speed
            t.angular.z = direction

            pub.publish(t)
            rate.sleep()




if __name__ == "__main__":
  main()