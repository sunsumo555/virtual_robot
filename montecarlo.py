from math import cos, sin, acos, sqrt, exp, pi
import numpy as np

walls = [(0,0,0,168),
         (0,168,84,168),
         (84,126,84,210),
         (84,210,168,210),
         (168,210,168,84),
         (168,84,210,84),
         (210,84,210,0),
         (210,0,0,0)]

def distance_to_wall(x,y,theta,wall):
    ax, ay, bx, by = wall
    num = (by-ay)*(ax-x)-(bx-ax)*(ay-y)
    denom = (by-ay)*cos(theta)-(bx-ax)*sin(theta)
    if abs(denom) < 10.0**-3:
        return -1
    return num/denom

def find_likelihood(measurement,x,y,theta,measurement_var = 9):
    actual_distance = find_actual_distance(x,y,theta)
    k = 0.1
    likelihood = k
    if actual_distance == -1:
        likelihood += 0
    else:
        likelihood += exp((-1*(measurement - actual_distance)**2)/(2*measurement_var))
    return likelihood

def find_actual_distance(x,y,theta):
    min_dist = 9999
    for wall in walls:
        dist = distance_to_wall(x,y,theta,wall)

        if dist < min_dist:
            if is_valid_distance(dist,wall,x,y,theta):
                min_dist = dist

    if(min_dist == 9999):
        print("montecarlo warning: no valid wall for the particle at x = "+str(x)+" y = "+str(y)+" theta = "+str(theta*180.0/pi))
        return -1
    return min_dist

def is_valid_distance(dist,wall,x,y,theta):
    if dist < 0:
        return False
    x_intersect = x+cos(theta)*dist
    y_intersect = x+sin(theta)*dist
    if not in_line(x_intersect,y_intersect,wall):
        return False
    return True

def in_line(x,y,wall):
    tol = 0.0001
    ax,ay,bx,by = wall
    if(ax>bx):
        ax,bx = bx,ax
    if(ay>by):
        ay,by = by,ay
    return x>=ax-tol and x<=bx+tol and y>=ay-tol and y<=by+tol



















def angle_to_wall_normal(position, wall):
    _, _, theta = position
    ax, ay, bx, by = wall
    num = cos(theta)(ay-by)+sin(theta)(bx-ax)
    denom = sqrt((ay-by)**2+(ax-bx)**2)
    return acos(num/denom)

def calculate_likelihood(position, measured_distance, sigma_squared):
    x, y, theta = position
    constant = 0.0001
    wall, min_distance_to_wall = find_relevant_wall(position, walls)
    #n_out_of_bounds=0

    if min_distance_to_wall == "particle out of bounds":
    #    n_out_of_bounds += 1
        likelihood = 0
    else:
        likelihood = exp(-1*(measured_distance-min_distance_to_wall)**2/(2*sigma_squared))
        likelihood += constant
        
    #print("likelihood: calculated = "+str(min_distance_to_wall) + " vs measured = " + str(measured_distance) + "likelihood = " + str(likelihood))
    return likelihood

def calculate_likelihood_multivariate(calculated_distance_list, measure_distance_list, measurement_var):
    if len(calculated_distance_list) != len(measure_distance_list):
        print("likelihood multivariate length not equal")
        return
    
    constant = 0.0001
    n_dimension = len(calculated_distance_list)
        
    #coeff = 1/(((2*pi)**(n_dimension/2))*(measurement_var**(n_dimension/2)))
    x_minus_mu = np.array(calculated_distance_list) - np.array(measure_distance_list)
    exponent = -0.5 * np.dot(x_minus_mu,x_minus_mu) / measurement_var

    coeff=100.0
    likelihood = coeff*exp(exponent)
    likelihood += constant
    
    #print("exponent = "+str(exponent)+" multivariate likelihood = "+str(likelihood))
    
#    likelihoods = []
#    for c,z in zip(calculated_distance_list,measure_distance_list):
#        likelihoods.append(np.maximum(0.1,exp(-1*(c-z)**2/(2*measurement_var))))
    
#    print(likelihoods)
#    likelihood = np.sum(likelihoods)+constant
    return likelihood