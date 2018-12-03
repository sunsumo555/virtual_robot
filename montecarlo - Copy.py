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

def distance_to_wall(position,wall):
    x, y, theta = position
    ax, ay, bx, by = wall
    num = (by-ay)*(ax-x)-(bx-ax)*(ay-y)
    denom = (by-ay)*cos(theta)-(bx-ax)*sin(theta)
    if abs(denom) < 10.0**-3:
        return -1
    return num/denom

def find_relevant_wall(position, walls):
    wall_distances = []
    walls_ahead = []
    x, y, theta = position
#    print(">>> Particle at x = "+str(x)+" y = "+str(y)+ " theta = "+str(theta * 180.0 / pi))
    i=0
    for wall in walls:
        distance_from_wall = distance_to_wall(position, wall)
        i+=1

        # Discount walls which are a negative distance away
        # i.e. behind the robot.
        x_intersect = x + (distance_from_wall * cos(theta))
        y_intersect = y + (distance_from_wall * sin(theta))

        #print(">>> Distance to wall "+str(i)+" = "+str(distance_from_wall) + ", intersect at = "+str(x_intersect)+","+str(y_intersect)+" inline = "+str(in_line(x_intersect, y_intersect, wall)))
        
        if distance_from_wall > 0 and in_line(x_intersect, y_intersect, wall):
            walls_ahead.append(distance_from_wall)
            wall_distances.append(distance_from_wall)
#        else:
#            print("wall ",str(i)," is irrelevant")

    #print("walls ahead")
    #print(walls_ahead)
    if walls_ahead == []:
        min_distance = "particle out of bounds"
        return min_distance, min_distance
    else:
        min_distance = min(walls_ahead)
        relevant_wall = wall_distances.index(min_distance)
    
#    print("relevant_wall :")
#    print(relevant_wall)
#    print("min distance = "+str(min_distance))
    return relevant_wall, min_distance

def in_line(x, y, wall):
    # checks if x,y is actually on the line
    tol = 0.0001
    
    ax, ay, bx, by = wall
    if(ax>bx):
        # make sure that ax is smaller than bx
        ax,bx = bx,ax
    if(ay>by):
        # make sure that ay is smaller than by
        ay,by = by,ay
    if(x>=ax-tol and x<=bx+tol and y>=ay-tol and y<=by+tol):
        #if the intersection point is not out of the boundary, it is on the line
        return True
    return False

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

if __name__ == "__main__":
    walls = [(0,0,0,168),
             (0,168,84,168),
             (84,126,84,210),
             (84,210,168,210),
             (168,210,168,84),
             (168,84,210,84),
             (210,84,210,0),
             (210,0,0,0)]
    
    x=84
    y=30
    theta = 0
    sonar_theta = 180 / 180.0 * pi
    wall,distance = find_relevant_wall((x,y,theta+sonar_theta),walls)
    print(wall)
    print(distance)
