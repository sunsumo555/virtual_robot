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

    def navigate_to_waypoint(self, target_x, target_y):
        """
        Navigates to the given waypoint and stops.
        :param target_x: World frame x coordinate of target in cm
        :param target_y: World frame y coordinate of target in cm
        """
        # Convert coordinates into Robot frame
        x_diff = target_x - self.particle_cloud.avg_x
        y_diff = target_y - self.particle_cloud.avg_y
        tol = 10**-3
        if abs(x_diff) < tol and abs(y_diff) < tol:
            return None
        elif x_diff == 0:
            world_frame_angle = math.pi/2 * math.copysign(1, y_diff)
        elif y_diff==0 and x_diff < 0:
            world_frame_angle = math.pi
        elif x_diff < 0:
            world_frame_angle = math.pi - math.atan(y_diff/abs(x_diff))
        else:
            world_frame_angle = math.atan(y_diff/x_diff)

        robot_frame_angle = world_frame_angle - self.particle_cloud.avg_theta

        print("rotating " + str(robot_frame_angle) + " degrees")
        self.rotate_on_spot(robot_frame_angle)

        time.sleep(2)
        travel_distance = math.sqrt(x_diff**2 + y_diff**2)
        print("Moving forwards " + str(travel_distance) + " meters")

        step_distance_cm = 20.0
        if travel_distance < step_distance_cm:
            self.travel_forwards(travel_distance)
            self.update_particles()
        else:
            self.travel_forwards(step_distance_cm)
            time.sleep(2)
            self.update_particles()
            self.navigate_to_waypoint(target_x, target_y)
            
    def navigate_to_waypoint2(self, target_x, target_y):
        """
        Navigates to the given waypoint and stops.
        :param target_x: World frame x coordinate of target in cm
        :param target_y: World frame y coordinate of target in cm
        """
        # Convert coordinates into Robot frame
        x_diff = target_x - self.r.particle_cloud.avg_x
        y_diff = target_y - self.r.particle_cloud.avg_y
        tol = 10**-3
        
        # if the distance is not that great, forget it
        if abs(x_diff) < tol and abs(y_diff) < tol:
            return None

        world_frame_angle = math.atan2(y_diff,x_diff)
        robot_frame_angle = world_frame_angle - self.r.particle_cloud.avg_theta
        
        if robot_frame_angle < -self.pi:
            robot_frame_angle += 2*self.pi
            
        if robot_frame_angle > self.pi:
            robot_frame_angle -= 2*self.pi

        print("rotating " + str(robot_frame_angle * 180 / self.pi) + " degrees")
        self.rotate_on_spot(robot_frame_angle)
        print("rotated " + str(robot_frame_angle * 180 / self.pi) + " degrees")

        time.sleep(2)
        travel_distance = math.sqrt(x_diff**2 + y_diff**2)
        self.travel_forwards(travel_distance)

    def rotate_on_spot(self, angle):
        """
        Rotates the robot by `angle` radians on the spot anticlockwise.
        """
        self.particle_cloud.rotate(angle)

    def update_particles(self):
        distance_observed = self.sonar_sensor.value()[0]
        self.particle_cloud.update_cloud_from_measurement(distance_observed, self.measurement_var)

    def left_90_deg(self):
        """
        Rotates the robot 90 degrees on the spot anticlockwise.
        """
        self.rotate_on_spot(math.pi/4)

    def right_90_deg(self):
        """
        Rotates the robot 90 degrees on the spot clockwise.
        """
        self.rotate_on_spot(-1 * math.pi/4)

    def travel_forwards(self, distance_cm):
        """
        Causes the robot to move forwards for a given distance at a given speed.
        :param distance: The distance the robot will travel in cm.
        :param rate: The speed at which the robot will travel.
        """
        self.particle_cloud.drive(distance_cm)