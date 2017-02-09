from quadquad_hardware.msg import QuadServos
import time
import numpy as np
import math

from scipy.interpolate import interp1d


class Gait(object):
  def __init__(self):
    self.timestep = 0

  def get_next_position(self, timestep, speed, direction):
    msg = QuadServos
    msg.BLHip = 90.0
    msg.BRHip = 90.0
    msg.FLHip = 90.0
    msg.FRHip = 90.0

    msg.BLLeg = 0.0
    msg.BRLeg = 0.0
    msg.FLLeg = 0.0
    msg.FRLeg = 0.0
    return msg


class CreepGait(Gait):
  def __init__(self):
    self.index = 0
    self.timestep = 0
    self.build_gaits()

  def build_gaits(self):
    self.steps = 1000

    qtime = np.array(range(self.steps))

    self.leg = np.sin(qtime*2*np.pi/self.steps*2)
    self.leg[(self.steps/4 - 1):] = 0


    hippts = np.array([(0, 0), (150, 0.9), (500, 0.6), (700, 0.2), (1000, 0)])
    hipfn = interp1d(hippts[:,0], hippts[:,1], kind='cubic')
    self.hip = hipfn(qtime)
    self.hip = np.clip(self.hip, 0.0, 1.0)



  def get_next_position(self, curtime, speed, direction):
    """
        @curtime is a time interval between movements
        @speed is a value from 0 to 1.0 corresponding to stride
        @direction gives a radial (degrees) direction, 0 being forward 180 being backwards
    """
    msg = QuadServos()

    msg.BLHip = 90.0
    msg.BRHip = 90.0
    msg.FLHip = 90.0
    msg.FRHip = 90.0

    msg.BLLeg = 0.0
    msg.BRLeg = 0.0
    msg.FLLeg = 0.0
    msg.FRLeg = 0.0

    if speed == 0.0 and direction == 0:
      return msg

    vel_left = (speed - direction)
    vel_right = (speed + direction)

    msg.FLHip = (vel_left*self.hip[self.index % len(self.hip)] * .5)*40. + 70
    msg.FLLeg = self.leg[self.index % len(self.leg)]*20. + 30

    msg.FRHip = (vel_right*self.hip[(self.index + 500) % len(self.hip)] * .5)*40. + 70
    msg.FRLeg = self.leg[(self.index + 500) % len(self.leg)]*20. + 20

    msg.BLHip = (vel_left*self.hip[(self.index + 500) % len(self.hip)] * .5)*40. + 70
    msg.BLLeg = self.leg[(self.index + 500) % len(self.leg)]*20. + 20

    msg.BRHip = (vel_right*self.hip[(self.index) % len(self.hip)] * .5)*40. + 70
    msg.BRLeg = self.leg[(self.index) % len(self.leg)]*20. + 35

    self.index += 20

    return msg


