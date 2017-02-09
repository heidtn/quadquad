#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from quadquad_hardware.msg import QuadServos
from geometry_msgs.msg import Twist

from threading import Thread, Lock

from gaits import CreepGait


"""
This program creates a thread that continually listens for incoming Twist messages to steer the quad.
Meanwhile another thread actually writes messages to control the quads motion.
"""

speed = 0.0
direction = 0.0

lock = Lock()

def move_quad_thread():
  global speed
  global direction

  pub = rospy.Publisher('servo_controller', QuadServos, queue_size=10)
  rate = rospy.Rate(10) # 10hz

  rospy.loginfo(rospy.get_caller_id() + " created gait controller")

  gait_controller = CreepGait()

  while not rospy.is_shutdown():
    lock.acquire()
    spd = speed
    dr = direction
    lock.release()
    msg = gait_controller.get_next_position(0, spd, dr)
    pub.publish(msg)
    rate.sleep()

def handle_msg(gaitMsg):
  global speed
  global direction
  rospy.loginfo(rospy.get_caller_id() + "new control command")
  lock.acquire()
  speed = gaitMsg.linear.x
  direction = gaitMsg.angular.z
  print speed, direction

  lock.release()


def create_listener_node():
  rospy.Subscriber('cmd_vel', Twist, handle_msg)
  rospy.loginfo(rospy.get_caller_id() + " created listener")
  rospy.spin()
    

if __name__ == "__main__":
  rospy.init_node('quad_gait_controller')

  t = Thread(target=move_quad_thread)
  t.setDaemon = True
  t.start()

  create_listener_node()
