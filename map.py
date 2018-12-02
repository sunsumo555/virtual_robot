import montecarlo as mc
import numpy as np
import robot
import state
import math
import time

class Map:
    def __init__(self,starting_x=0, starting_y=0, enlargement_factor = 1, x_padding = 0, y_padding = 0):
        self.r = robot.Robot(starting_x = starting_x,
                             starting_y = starting_y,
                             n_particles = 20,
                             motion_sigma_distance=0.01,
                             motion_sigma_angle=0.001,
                             rotation_sigma_angle=0.001)

        self.walls = mc.walls
        self.enlargement_factor = enlargement_factor
        self.x_padding = x_padding
        self.y_padding = y_padding

        self.print_weights = True
        self.sleep_time = 2

        #for debugging only, dont open it!
        self.r.particle_cloud.randomize_particles(70,90,20,40)

        self.measurement_var = 50

        self.draw_particles()
        self.pi = 3.14159

    def report_robot(self):
        print(">>> robot at x="+str(self.r.particle_cloud.avg_x) +" y=" +str(self.r.particle_cloud.avg_y)+" theta="+str(self.r.particle_cloud.avg_theta * 180 / self.pi))

    def draw_map(self):
        for wall in self.walls:
            line = (self.enlargement_factor*wall[0]+self.x_padding,
                    self.enlargement_factor*wall[1]+self.y_padding,
                    self.enlargement_factor*wall[2]+self.x_padding,
                    self.enlargement_factor*wall[3]+self.y_padding)
            print "drawLine:" + str(line)

    def draw_particles(self):
        if self.print_weights:
            print_vector = [(self.enlargement_factor*p.x + self.x_padding,
                             self.enlargement_factor*p.y + self.y_padding,
                             self.enlargement_factor*p.theta,w*100) for p,w in zip(self.r.particle_cloud.particles,self.r.particle_cloud.weights)]
            print "drawParticles:" + str(print_vector)

        else:
            print_vector = [(p.x + self.x_padding, p.y + self.y_padding, p.theta) for p in self.r.particle_cloud.particles]
            print "drawParticles:" + str(print_vector)

        crosshair_y = (self.enlargement_factor*self.r.particle_cloud.avg_x,
                       self.enlargement_factor*self.r.particle_cloud.avg_y-self.enlargement_factor*3,
                       self.enlargement_factor*self.r.particle_cloud.avg_x,
                       self.enlargement_factor*self.r.particle_cloud.avg_y+self.enlargement_factor*3)
        print "drawLine:" + str(crosshair_y)

        crosshair_x = (self.enlargement_factor*self.r.particle_cloud.avg_x-self.enlargement_factor*3,
                       self.enlargement_factor*self.r.particle_cloud.avg_y,
                       self.enlargement_factor*self.r.particle_cloud.avg_x+self.enlargement_factor*3,
                       self.enlargement_factor*self.r.particle_cloud.avg_y)
        print "drawLine:" + str(crosshair_x)

    def move_robot_forward(self,d,use_slam = False):
        if use_slam:
            initial_x = self.r.particle_cloud.avg_x
            initial_y = self.r.particle_cloud.avg_y
            initial_theta = self.r.particle_cloud.avg_theta

            target_x = initial_x + d*math.cos(initial_theta)
            target_y = initial_y + d*math.sin(initial_theta)

            for i in range(10):
                self.r.travel_forwards(d/10.0)
                self.draw_particles()
                self.update_particle_all()
                self.draw_particles()
                time.sleep(0.1)

            current_x = self.r.particle_cloud.avg_x
            current_y = self.r.particle_cloud.avg_y
            current_theta = self.r.particle_cloud.avg_theta

            distance_error = math.sqrt((current_x-target_x)**2 + (current_y-target_y)**2)
            self.draw_particles()

            if abs(distance_error) > 1.0:
                self.move_robot_forward(distance_error,use_slam=False)

        else:
            self.r.travel_forwards(d)
            self.draw_particles()

    def rotate_robot(self,radian,direction = 'ccw',use_slam = False):
        if direction == 'ccw':
            self.r.rotate_on_spot(radian)
            self.draw_particles()
        elif direction == 'cw':
            self.r.rotate_on_spot(-1*radian)
            self.draw_particles()
        else:
            print "invalid direction command on Map.rotate_robot: please input ccw or cw"

    def move_robot_to(self,target_x,target_y):
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
        self.rotate_robot(robot_frame_angle,direction='ccw',use_slam=True)
        print("rotated " + str(robot_frame_angle * 180 / self.pi) + " degrees")
        #self.report_robot()

        time.sleep(2)
        travel_distance = math.sqrt(x_diff**2 + y_diff**2)

        print("Moving forwards " + str(travel_distance) + " meters")
        self.move_robot_forward(travel_distance,use_slam=False)
        print("Moved forwards " + str(travel_distance) + " meters")
        #self.report_robot()

        time.sleep(2)

    def move_robot_to_slam(self,target_x,target_y):
        print("moving first half")
        x,y,theta = self.r.where_am_i()
        self.move_robot_to((target_x + x)/2.0,(target_y + y)/2.0)
        time.sleep(2)

        print("localizing robot")
        self.scan_sonar_and_update_batch()
        time.sleep(2)

        print("moving second half")
        self.move_robot_to(target_x,target_y)
        time.sleep(2)

        self.scan_sonar_and_update_batch()

    def move_robot_to2(self,target_x,target_y, use_slam = False):
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
        self.rotate_robot(robot_frame_angle,direction='ccw',use_slam=True)
        print("rotated " + str(robot_frame_angle * 180 / self.pi) + " degrees")
        #self.report_robot()

        time.sleep(self.sleep_time)
        travel_distance = math.sqrt(x_diff**2 + y_diff**2)

        if use_slam:
            print("Moving forwards " + str(travel_distance) + " meters")
            n_steps = 5.0
            for i in range(int(n_steps)):
                self.move_robot_forward(travel_distance/n_steps,use_slam = False)
                self.update_particle_all()
                self.draw_particles()
                time.sleep(1)

                current_x = self.r.particle_cloud.avg_x
                current_y = self.r.particle_cloud.avg_y
                current_theta = self.r.particle_cloud.avg_theta
                print(">>> World_frame_angle = "+str(world_frame_angle)+" current_angle = "+str(current_theta) + " error = " + str(world_frame_angle - current_theta))

                if abs(world_frame_angle - current_theta) > 0.016:
                    print(">>> Robot out of course, world_frame_angle = "+str(world_frame_angle)+" current_angle = "+str(current_theta))
                    #self.move_robot_to(target_x,target_y, use_slam = False)
                    self.rotate_robot(world_frame_angle - current_theta,direction='ccw',use_slam=False)
                    time.sleep(1)

            print("Moved forwards " + str(travel_distance) + " meters")

            self.update_particle_all()
            self.draw_particles()
            time.sleep(1)

            current_x = self.r.particle_cloud.avg_x
            current_y = self.r.particle_cloud.avg_y
            current_theta = self.r.particle_cloud.avg_theta

            distance_error = math.sqrt((target_x-current_x)**2 + (target_y - current_y)**2)
            print("Performing final correction of " + str(distance_error) + " meters")
            self.move_robot_forward(distance_error,use_slam=False)
            print("Destination reached")

        else:
            print("Moving forwards " + str(travel_distance) + " meters")
            self.move_robot_forward(travel_distance,use_slam=False)
            print("Moved forwards " + str(travel_distance) + " meters")
            #self.report_robot()

        time.sleep(self.sleep_time)

    def get_robot_measurement(self):
        return self.r.sonar_sensor.value()[0]

    def update_particle_weights(self):
        measurement_distance = self.get_robot_measurement()
        self.r.particle_cloud.update_weights(measurement_distance, self.measurement_var)
        self.draw_particles()

    def update_particle_all(self):
        measurement_distance = self.get_robot_measurement()
        self.r.particle_cloud.update_cloud_from_measurement(measurement_distance,self.measurement_var)
        self.draw_particles()

    def update_particles_at_angle(self,theta):
        self.r.rotate_sonar_to(theta)
        time.sleep(1)
        self.update_particle_all()

    def scan_sonar_and_update(self):
        for theta in np.linspace(-1*math.pi,math.pi,7):
            # rotate the sonar by 30 degree from -180 to 180 (7 steps)
            self.update_particles_at_angle(theta)

        self.r.rotate_sonar_to(0.0)

    def scan_sonar_and_update_batch(self):
        thetas = []
        measurements = []

        print(">>> taking measurements")

        self.r.rotate_sonar_to(-1*math.pi)
        time.sleep(2)

        for theta in np.linspace(-1*math.pi,5.0/6.0*math.pi,10):
            # rotate the sonar by 30 degree from -180 to 120 (6 steps)
            thetas.append(theta)

            self.r.rotate_sonar_to(theta)
            time.sleep(1.5)
            measurements.append(self.get_robot_measurement())

        print(measurements)

        self.r.rotate_sonar_to(0.0)
        print(">>> updating particles")
        #the first measurement is often errorneous
        self.r.particle_cloud.update_cloud_from_measurement_batch(thetas,measurements,self.measurement_var)
        print(">>> drawing particles")
        self.draw_particles()
        print(">>> done localizing")
