from __future__ import division
from math import pi, cos, sin
from random import gauss, uniform
import numpy as np
import montecarlo as mc
import particle

class Cloud:
    def __init__(self, n_particles=100, x=0, y=0, theta=0,
                 motion_sigma_distance=0.1, motion_sigma_angle=0.005,
                 rotation_sigma_angle=0.05):
        self.n_particles = n_particles
        self.particles = [particle.Particle(x,
                                        y,
                                        theta,
                                        motion_sigma_distance,
                                        motion_sigma_angle,
                                        rotation_sigma_angle) for _ in range(self.n_particles)]
        self.weights = 1.0/self.n_particles * np.ones(self.n_particles)
        self.update_average_position()
        #sonar angle will be between -180 to 180 degrees
        self.sonar_angle = 0

    def __str__(self):
        for p in self.particles:
            print(p)
        return "====== done printing particles ======="

    def randomize_particles(self,x_lower_bound,x_upper_bound,y_lower_bound,y_upper_bound, x_0=10, y_0=10, t_0=0):
        i = 0
        self.particles[0].x = x_0
        self.particles[0].y = y_0
        self.particles[0].theta = t_0
        for p in self.particles[1:]:
            i+=1
            x = (x_upper_bound - x_lower_bound) * np.random.random_sample() + x_lower_bound
            y = (y_upper_bound - y_lower_bound) * np.random.random_sample() + y_lower_bound
            #x = i/self.n_particles * upper_bound
            theta = 2*3.14135 * np.random.random_sample() - 3.14135
            theta = 0
            p.x = x
            p.y = y
            p.theta = theta

    def reset_weight_matrix(self):
        self.weights = 1.0/self.n_particles * np.ones(self.n_particles)

    def draw_cloud(self,padding_x=0,padding_y=0):
        particles = [(particle.x+padding_x,
                      particle.y+padding_y,
                      particle.theta) for particle in self.particles]
        return particles

    def drive(self, distance):
        for particle in self.particles:
            particle.drive(distance)
        self.update_average_position()

    def rotate(self, theta):
        """
        rotate the particle cloud
        """
        for particle in self.particles:
            particle.rotate(theta)
            #print("theta after rotate, before normalizing = "+str(particle.theta))
        self.update_average_position()

    def update_weights(self, distance_observed, measurement_sigma):
        """
        Update the weight matrix based on the likelihood function using
        the measurement from the sonar.
        """
        new_weights = []
        for index, particle in enumerate(self.particles):
            new_weight = mc.calculate_likelihood(
                            [particle.x, particle.y, particle.theta + self.sonar_angle],
                            distance_observed,
                            measurement_sigma)

            new_weights.append(new_weight)

        weights_sum = np.sum(new_weights)
        self.weights = np.array(new_weights)/weights_sum

    def update_weights_batch(self, thetas, measurements, measurement_sigma):
        new_weights = []
        #print(thetas)
        #print(measurements)
        for index, particle in enumerate(self.particles):
            new_weight = 0
            calc_list = []
            i=0
            for theta, measurement in zip(thetas,measurements):
                if i==0:
                    #skip the first erratic measurement
                    calc_list.append(measurement)
                    i+=1
                    continue
                if measurement == 255:
                    calc_list.append(255)
                    continue
#                new_weight *= mc.calculate_likelihood(
#                                [particle.x, particle.y, particle.theta + theta],
#                                measurement,
#                                measurement_sigma)
                wall, min_distance_to_wall = mc.find_relevant_wall((particle.x,particle.y,particle.theta + theta), mc.walls)
                if wall == "particle out of bounds":
                    min_distance_to_wall = 9999

                calc_list.append(min_distance_to_wall)

            print("index = "+str(index) + "x = "+str(particle.x) + "y = "+ str(particle.y) + " z = "+ str(particle.theta))
            print(calc_list)
            print(measurements)
            new_weight = mc.calculate_likelihood_multivariate(calc_list, measurements, measurement_sigma)
            new_weights.append(new_weight)

        print("new weights:")
        for i,w in enumerate(new_weights):
            print("i = "+str(i)+" w = "+str(w))

        weights_sum = np.sum(new_weights)
        self.weights = np.array(new_weights)/weights_sum

        print("final weights after normalized by",weights_sum)
        for i,w in enumerate(self.weights):
            print("i = "+str(i)+" w = "+str(w))

    def update_cloud_from_measurement(self, distance_observed,
                                            measurement_var=3):
        """
        Given the measurement of the sonar, update the particle cloud.
        First update the weights using the likelihood function
        Then resample the particle set based on the new weights
        Update the average position of the particles given the new particle set
        """
        self.update_weights(distance_observed, measurement_var)
        self.resample()
        self.update_average_position()
        self.reset_weight_matrix()

    def update_cloud_from_measurement_batch(self, thetas, measurements, measurement_var=3, weights_only = False):
        '''update the particle cloud using a list of measurements'''
        self.update_weights_batch(thetas, measurements, measurement_var)
        if weights_only:
            return
        self.resample()
        self.update_average_position()
        self.reset_weight_matrix()

    def resample_np(self):
        self.particles = np.random.choice(self.particles, self.n_particles ,p=self.weights)

    def renormalize_avg_theta(self):
        while self.avg_theta > pi:
            self.avg_theta -= 2*pi

        while self.avg_theta <= -1*pi:
            self.avg_theta += 2*pi

    def renormalize_thetas(self):
        for p in self.particles:
            p.renormalize_theta()

    def update_average_position(self):
        """
        Update the average postion of the particle cloud
        """
        x_vector = np.array([particle.x for particle in self.particles])
        y_vector = np.array([particle.y for particle in self.particles])
        theta_vector = np.array([particle.theta for particle in self.particles])

        self.avg_x = self.weights.dot(x_vector)
        self.avg_y = self.weights.dot(y_vector)
        self.avg_theta = self.weights.dot(theta_vector)
        self.renormalize_avg_theta()

        if abs(self.avg_theta - pi) > pi/6.0:
            self.renormalize_thetas()

    def resample(self):
        #print("RESAMPLING")
        #print("STATE: particles before")
        #self.print_all_cloud()
        #print("=========================")

        new_particles = []
        acc_weights = []
        running_total = 0
        for weight in range(len(self.weights)):
            running_total += self.weights[weight]
            acc_weights.append(running_total)
        acc_weights[-1] = 1.0    # Enforces the list of weights to sum to 1. This avoids rounding errors.

        for i in range(len(self.weights)):
            rand_particle = np.random.random()
            if rand_particle < min(self.weights):
                rand_particle = max(self.weights) # prevents the random particle being less than any weight.
            for particle in range(len(self.particles)):
                if rand_particle < acc_weights[particle]:
                    break

            #create new particle instead of copying
            print("resampling: picked particle %s"%(particle-1))
            new_particle = WorldPosition(x = self.particles[particle - 1].x,
                                         y = self.particles[particle - 1].y,
                                         theta = self.particles[particle - 1].theta,
                                         motion_sigma_distance = self.particles[particle - 1].motion_sigma_distance,
                                         motion_sigma_angle = self.particles[particle - 1].motion_sigma_angle,
                                         rotation_sigma_angle = self.particles[particle - 1].rotation_sigma_angle)

            #new_particles.append(self.particles[particle - 1])
            new_particles.append(new_particle)
        self.particles = new_particles
        #print("STATE: particles after")
        #self.print_all_cloud()
        #print("=========================")

    def covariance(self):
        x_vector = np.array([particle.x for particle in self.particles])
        y_vector = np.array([particle.y for particle in self.particles])
        theta_vector = np.array([particle.theta for particle in self.particles])
        data = np.vstack([x_vector, y_vector, theta_vector]).T

        return np.cov(data, bias=True)

    def rotate_sonar(self,theta):
        #ccw is positive, cw is negative angle
        self.sonar_angle += theta
