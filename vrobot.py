from __future__ import division
import time
import math
import numpy as np

from state import ParticleCloud

class Robot:

    def __init__(self,starting_x=0, starting_y=0, n_particles=100, motion_sigma_distance=0, motion_sigma_angle=0, rotation_sigma_angle=0):
        # register starting position
        self.particle_cloud = ParticleCloud(x = starting_x, y = starting_y,
                                            n_particles = n_particles,
                                            motion_sigma_distance=motion_sigma_distance,
                                            motion_sigma_angle=motion_sigma_angle,
                                            rotation_sigma_angle=rotation_sigma_angle)

        self.measurement_var = 3

    def rotate_sonar_by(self,theta):
        #-1 is used so that the positive angle is ccw
        self.particle_cloud.rotate_sonar(theta)

    def rotate_sonar_to(self,theta):
        delta_angle = theta-self.particle_cloud.sonar_angle
        self.rotate_sonar_by(delta_angle)

    def where_am_i(self):
        return self.particle_cloud.avg_x, self.particle_cloud.avg_y, self.particle_cloud.avg_theta

    def rotate_on_spot(self, angle):
        """
        Rotates the robot by `angle` radians on the spot anticlockwise.
        """
        self.particle_cloud.rotate(angle)

    def update_particles(self):
        distance_observed = self.sonar_sensor.value()[0]
        self.particle_cloud.update_cloud_from_measurement(distance_observed, self.measurement_var)

    def travel_forwards(self, distance_cm):
        """
        Causes the robot to move forwards for a given distance at a given speed.
        :param distance: The distance the robot will travel in cm.
        :param rate: The speed at which the robot will travel.
        """
        self.particle_cloud.drive(distance_cm)
