import importlib
import montecarlo as mc
import matplotlib.pyplot as plt
import cv2
import numpy as np
import robot
import math
from math import pi

class World:
    def __init__(self,starting_x=0, starting_y=0, enlargement_factor = 1, x_padding = 0, y_padding = 0):
        self.r = robot.Robot(starting_x = starting_x,
                             starting_y = starting_y,
                             n_particles = 100,
                             motion_sigma_distance=2.0, 
                             motion_sigma_angle=1.0*pi/180.0,
                             rotation_sigma_angle=2.0*pi/180.0)
        
        self.walls = mc.walls
        self.enlargement_factor = enlargement_factor
        self.x_padding = x_padding
        self.y_padding = y_padding
        
        self.actual_x = starting_x
        self.actual_y = starting_y
        self.actual_t = 0
        
        self.print_weights = True
        self.sleep_time = 2
        
        #for debugging only, dont open it!
        #self.r.particle_cloud.randomize_particles(0,200,0,200, x_0=self.actual_x, y_0=self.actual_y, t_0=self.actual_t)
        
        self.measurement_var = 50
               
        self.draw_particles()
        self.pi = 3.14159
        
        self.display = np.zeros((250*self.enlargement_factor,250*self.enlargement_factor))

    def move_robot_forward(self,d):
        self.r.travel_forwards(d)
        self.draw_particles()
        self.actual_x += d*math.cos(self.actual_t)
        self.actual_y += d*math.sin(self.actual_t)
        
    def rotate_robot(self,radian):
        self.r.rotate_on_spot(radian)
        self.draw_particles()
        self.actual_t += radian

    def get_robot_measurement(self):
        return self.r.measure()

    def localize_robot(self):
        self.r.localize()

#---------------------------------
    
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
        
    def display_img(self):
        self.draw_map()
        self.draw_particles()
        plt.imshow(self.display)
           
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
    
#=============================================================

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
    
    def scan_sonar_and_update(self):
        for theta in np.linspace(-1*math.pi,math.pi,7):
            # rotate the sonar by 30 degree from -180 to 180 (7 steps)
            self.update_particles_at_angle(theta)
            
        self.r.rotate_sonar_to(0.0)
        
    def scan_sonar_and_update_batch(self, weights_only = False):
        print("actual robot at x=",self.actual_x,"y=",self.actual_y,"t=",self.actual_t)
        thetas = []
        measurements = []
        
        print(">>> taking measurements")
        
        self.r.rotate_sonar_to(-1*math.pi)
        
        for theta in np.linspace(-1*math.pi,5.0/6.0*math.pi,4):
            # rotate the sonar by 30 degree from -180 to 120 (6 steps)
            thetas.append(theta)
            
            self.r.rotate_sonar_to(theta)
            measurements.append(self.get_robot_measurement())
                     
        print("===== measurement summary =====")
        print(np.array(thetas)*180/math.pi)
        print(measurements)
        print("===============================")
        
        self.r.rotate_sonar_to(0.0)
        print(">>> updating particles")
        #the first measurement is often errorneous
        self.r.particle_cloud.update_cloud_from_measurement_batch(thetas,measurements,self.measurement_var, weights_only)
        print(">>> drawing particles")
        self.draw_particles()
        print(">>> done localizing")

#============= book keeping ====================

    def __str__(self):
        return ">>> robot at x="+str(self.r.particle_cloud.avg_x) +" y=" +str(self.r.particle_cloud.avg_y)+" theta="+str(self.r.particle_cloud.avg_theta * 180 / self.pi)
        
    def draw_map(self):
        for wall in self.walls:
            line = (self.enlargement_factor*wall[0]+self.x_padding, 
                    self.enlargement_factor*wall[1]+self.y_padding, 
                    self.enlargement_factor*wall[2]+self.x_padding, 
                    self.enlargement_factor*wall[3]+self.y_padding)
            cv2.line(self.display,(self.enlargement_factor*wall[0]+self.x_padding,self.enlargement_factor*wall[1]+self.y_padding),(self.enlargement_factor*wall[2]+self.x_padding,self.enlargement_factor*wall[3]+self.y_padding),255,2)
            
    def draw_particles(self):
        self.display = np.zeros((250*self.enlargement_factor,250*self.enlargement_factor))
        self.draw_map()
        if self.print_weights:
            max_w = np.max(self.r.particle_cloud.weights)
            for p,w in zip(self.r.particle_cloud.particles,self.r.particle_cloud.weights):
                #print("displaying at x=",int(self.enlargement_factor*p.x + self.x_padding),"y =",int(self.enlargement_factor*p.y + self.y_padding))
                cv2.circle(self.display,(int(self.enlargement_factor*p.x + self.x_padding),int(self.enlargement_factor*p.y + self.y_padding)), 3*self.enlargement_factor, int(w/max_w*255), -1)

        else:
            print_vector = [(p.x + self.x_padding, p.y + self.y_padding, p.theta) for p in self.r.particle_cloud.particles]
            cv2.circle(self.display,(self.enlargement_factor*p.x + self.x_padding,self.enlargement_factor*p.y + self.y_padding), 255, -1)
