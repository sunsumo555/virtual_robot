from __future__ import division
from math import pi, cos, sin
from random import gauss, uniform
import numpy as np
import montecarlo as mc

def error(sigma):
    return gauss(0, sigma)

class Particle:
    def __init__(self,x=84.0,y=30.0, theta=0, motion_sigma_distance=0,
                  motion_sigma_angle=0, rotation_sigma_angle=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.motion_sigma_distance = motion_sigma_distance
        self.motion_sigma_angle = motion_sigma_angle
        self.rotation_sigma_angle = rotation_sigma_angle

    def __str__(self):
        return "x = "+str(self.x)+", y = "+str(self.y)+", theta = "+str(self.theta)

    def drive(self, distance):
        noisy_distance = distance + error(self.motion_sigma_distance)*distance
        delta_x = noisy_distance * cos(self.theta)
        delta_y = noisy_distance * sin(self.theta)
        #print(" x = "+str(self.x)+" + "+str(delta_x)+" = "+str(self.x+delta_x))
        self.x += delta_x
        self.y += delta_y
        self.theta += error(self.motion_sigma_angle)*distance

    def rotate(self, theta):
        new_theta = self.theta + theta + error(self.rotation_sigma_angle)*theta
        self.theta = new_theta

    def renormalize_theta(self):
        while self.theta > pi:
            self.theta -= 2*pi
        while self.theta <= -1*pi:
            self.theta += 2*pi
