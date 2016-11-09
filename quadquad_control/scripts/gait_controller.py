#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from quadquad_hardware.msg import QuadServos
from geometry_msgs.msg import Twist

from threading import Thread

from gaits import CreepGait

speed = 0.0
direction = 0.0


def move_quad_thread():
  global speed
  global direction

  pub = rospy.Publisher('servo_controller', QuadServos, queue_size=10)
  rate = rospy.Rate(10) # 10hz

  rospy.loginfo(rospy.get_caller_id() + " created gait controller")

  gait_controller = CreepGait()

  while not rospy.is_shutdown():
    #for testing only
    msg = gait_controller.get_next_position(0, 1.0, 0.0)
    pub.publish(msg)
    rate.sleep()

def handle_msg(gaitMsg):
  rospy.loginfo(rospy.get_caller_id() + "new control command")


def create_listener_node():
  rospy.Subscriber('gait_controller', Twist, handle_msg)
  rospy.loginfo(rospy.get_caller_id() + " created listener")
  rospy.spin()
    

if __name__ == "__main__":
  rospy.init_node('quad_gait_controller')

  t = Thread(target=move_quad_thread)
  t.setDaemon = True
  t.start()

  create_listener_node()
