from __future__ import division
import time
import math
import numpy as np
import cloud
import random
import montecarlo as mc
from math import pi

class Robot:

    def __init__(self,starting_x=0, starting_y=0, n_particles=100, motion_sigma_distance=2.0, 
        motion_sigma_angle=1.0*pi/180.0,rotation_sigma_angle=2.0*pi/180.0):
        # register starting position
        self.particle_cloud = cloud.Cloud(x = starting_x, y = starting_y,
                                            n_particles = n_particles,
                                            motion_sigma_distance=motion_sigma_distance,
                                            motion_sigma_angle=motion_sigma_angle,
                                            rotation_sigma_angle=rotation_sigma_angle)

        self.measurement_sigma = 3

    def __str__(self):
        return "robot at x = "+str(self.particle_cloud.avg_x)+", y = "+str(self.particle_cloud.avg_y)+" theta = "+str(self.particle_cloud.avg_theta)

    def rotate_sonar_by(self,theta):
        #-1 is used so that the positive angle is ccw
        self.particle_cloud.rotate_sonar(theta)

    def rotate_sonar_to(self,theta):
        delta_angle = theta-self.particle_cloud.sonar_angle
        self.rotate_sonar_by(delta_angle)

    def where_am_i(self):
        return self.particle_cloud.avg_x, self.particle_cloud.avg_y, self.particle_cloud.avg_theta

    def rotate(self, angle):
        #rotates ccw
        self.particle_cloud.rotate(angle)

    def update_particles(self):
        distance_observed = self.sonar_sensor.value()[0]
        self.particle_cloud.update_cloud_from_measurement(distance_observed, self.measurement_var)

    def measure(self):
        good_measure = random.random()
        # 20% to give crappy value
        if good_measure < 0.2:
            return 255
        else:
            wall, dist = mc.find_relevant_wall(self.where_am_i(),mc.walls)
            return dist + random.gauss(0,self.measurement_sigma)

    def drive(self, distance_cm):
        #drives the robot forward
        self.particle_cloud.drive(distance_cm)

    def localize(self):
        # measure
        measurement = self.measure()
        # update weights
        self.particle_cloud.update_weights(measurement)
        # resample
        self.particle_cloud.resample()