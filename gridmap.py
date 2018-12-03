import montecarlo as mc
import numpy as np
import robot
import state
from math import cos, sin, acos, sqrt, exp, pi
import time
import matplotlib.pyplot as plt

class Gridmap:
    def __init__(self,size = 250):
        self.r = robot.Robot(starting_x = 0,
                             starting_y = 0,
                             starting_theta = 0,
                             n_particles = 1,
                             motion_sigma_distance=0.0,
                             motion_sigma_angle=0.0,
                             rotation_sigma_angle=0.0)
        self.map = np.ones((size,size))*0.5
        self.increment_constant = 1
        self.decrement_constant = 0.01

    def set_robot_at(self,x,y,theta):
        self.r.particle_cloud.set_position(x,y,theta)

    def rotate_robot_to(self,theta):
        self.r.rotate(theta - self.r.particle_cloud.avg_theta)

    def measure_around(self,points = 120):
        original_bearing = self.r.particle_cloud.avg_theta
        measurements = []
        xs = []
        ys = []
        robot_thetas = []
        thetas = np.linspace(-1*pi,pi*(1.0-2.0/points),points)
        for theta in thetas:
            self.rotate_robot_to(theta+original_bearing)
            measurements.append(self.r.measure())
            xs.append(self.r.particle_cloud.avg_x)
            ys.append(self.r.particle_cloud.avg_y)
            robot_thetas.append(self.r.particle_cloud.avg_theta)
        self.rotate_robot_to(original_bearing)
        return xs, ys, robot_thetas, measurements

    def print_map(self):
        plt.figure(figsize = (7,5))
        plt.matshow(self.map)
        plt.colorbar()

    def update_map_batch(self,xs,ys,thetas,sonar_values):
        for x,y,theta,sonar_value in zip(xs,ys,thetas,sonar_values):
            if sonar_value == 255:
                continue
            self.update_map(x,y,theta,sonar_value)

    def update_map(self,x,y,theta,sonar_value):
        endpoint_x,endpoint_y = self.find_beam_endpoint(x,y,theta,sonar_value)
        self.increment_logodd(endpoint_x,endpoint_y)
        middle_points = self.find_intermediate_points(x,y,theta,sonar_value)
        for middle_point in middle_points:
            x,y = middle_point
            self.decrement_logodd(x,y)

    def increment_logodd(self,x,y):
        self.map[y,x] += self.increment_constant

    def decrement_logodd(self,x,y):
        self.map[y,x] -= self.decrement_constant

    def find_beam_endpoint(self,x,y,theta,sonar_value):
        endpoint_x = x + cos(theta)*sonar_value
        endpoint_y = y + sin(theta)*sonar_value
        return int(endpoint_x),int(endpoint_y)

    def find_intermediate_points(self,x,y,theta,sonar_value):
        points = []
        last_x = 0
        last_y = 0
        end_x,end_y = self.find_beam_endpoint(x,y,theta,sonar_value)
        for dist in range(int(sonar_value)):
            this_x, this_y = self.find_beam_endpoint(x,y,theta,dist)
            if this_x == end_x or this_y == end_y:
                break
            if(this_x != last_x and this_y != last_y):
                points.append((this_x,this_y))
        return points
