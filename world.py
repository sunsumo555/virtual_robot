import importlib
import montecarlo as mc
import matplotlib.pyplot as plt
import cv2
import numpy as np
import robot
import math
from math import pi, atan2

class World:
    def __init__(self,starting_x=0, starting_y=0, enlargement_factor = 1, x_padding = 0, y_padding = 0):
        self.r = robot.Robot(starting_x = starting_x,
                             starting_y = starting_y,
                             n_particles = 20,
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
        self.r.drive(d)
        self.actual_x += d*math.cos(self.actual_t)
        self.actual_y += d*math.sin(self.actual_t)
        plt.figure()
        plt.title("moved "+str(d))
        self.display_img()
        
    def rotate_robot(self,radian):
        self.r.rotate(radian)
        self.actual_t += radian
        plt.figure()
        plt.title("rotated "+str(radian*180.0/pi))
        self.display_img()

    def get_robot_measurement(self):
        return self.r.measure()

#-------------cw 5 ---------------
    def localize_robot(self):
        self.r.localize()

    def move_robot_to(self,target_x,target_y):
        x,y,theta = self.r.where_am_i()
        dx = target_x - x
        dy = target_y - y
        distance = (dx**2 + dy**2)**0.5

        target_theta = atan2(dy,dx)
        dtheta = target_theta - theta

        self.rotate_robot(dtheta)
        self.move_robot_forward(distance)

    def move_robot_to_slam(self,target_x,target_y):
        x,y,theta = self.r.where_am_i()
        dx = target_x - x
        dy = target_y - y
        distance = (dx**2 + dy**2)**0.5
        target_theta = atan2(dy,dx)
        dtheta = target_theta - theta

        while distance > 10:
            print("world: distance = "+str(distance))        

            self.rotate_robot(dtheta)
            self.move_robot_forward(10)
            self.localize_robot()
            
            x,y,theta = self.r.where_am_i()
            dx = target_x - x
            dy = target_y - y
            distance = (dx**2 + dy**2)**0.5
            target_theta = atan2(dy,dx)
            dtheta = target_theta - theta
            print("destination at "+str(target_x)+", "+str(target_y))
            print(self.r)    

        self.move_robot_forward(distance)

#---------------------------------
        



















    
#=============================================================
    
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

    def display_img(self):
        self.draw_map()
        self.draw_particles()
        plt.imshow(self.display)
