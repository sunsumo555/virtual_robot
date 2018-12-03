from __future__ import division
from math import pi, cos, sin
from random import gauss, uniform
import numpy as np
import montecarlo as mc

class Particle:
    def __init__(self,x=84.0,y=30.0, theta=0, motion_sigma_distance=2,
                  motion_sigma_angle=1.0*pi/180.0, rotation_sigma_angle=2.0*pi/180.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.motion_sigma_distance = motion_sigma_distance
        self.motion_sigma_angle = motion_sigma_angle
        self.rotation_sigma_angle = rotation_sigma_angle

    def __str__(self):
        return "x = "+str(self.x)+", y = "+str(self.y)+", theta = "+str(self.theta*180.0/pi)

    def drive(self, distance):
        noisy_distance = distance + gauss(0,self.motion_sigma_distance)*(distance/40)
        delta_x = noisy_distance * cos(self.theta)
        delta_y = noisy_distance * sin(self.theta)
        self.x += delta_x
        self.y += delta_y
        self.theta += gauss(0,self.motion_sigma_angle)*(distance/40)

    def rotate(self, theta):
        error = gauss(0,self.rotation_sigma_angle)*(2.0*theta/pi)
        new_theta = self.theta + theta + error
        #print("theta = "+str(self.theta*180.0/pi)+" + theta("+str(theta*180.0/pi)+") + error("+str(error*180.0/pi)+") = "+str(new_theta*180.0/pi))
        self.theta = new_theta

    def renormalize_theta(self):
        while self.theta > pi:
            self.theta -= 2*pi
        while self.theta <= -1*pi:
            self.theta += 2*pi
