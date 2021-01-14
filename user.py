import numpy as np
import pylab 
import random 
  

# Creates a user specified by its identity (one user one identity!), initial location, and initial speed
def create(initial_loc, initial_speed, identity):

	user = {}
	user["identity"] = identity
	user["phy_path_x"] = [] 
	user["phy_path_x"].append(initial_loc[0]) # Physical and virtual
	user["phy_path_y"] = []
	user["phy_path_y"].append(initial_loc[1])# Physical and virtual
	user["speed"] = initial_speed

	return user


# Creates a random virtual path 
def create_virtual_path_random(initial_loc, number_of_points, precision):

	# 50 ms precision, 20 points per second -> distance traversed for 1 m/s walk equals 0.05 m
	# Creating two arrays for containing x and y coordinates of size equals to the number of points and filled up with 0's 
	x = np.zeros(number_of_points) 
	x[0] = initial_loc[0]
	y = np.zeros(number_of_points) 
  	y[0] = initial_loc[1]

	# Filling the coordinates with random variables 
	for i in range(1, number_of_points): 
		
		val = random.randint(1, 4) # direction
		if val == 1: 
		    x[i] = x[i - 1] + 1 * precision
		    y[i] = y[i - 1] 
		elif val == 2: 
		    x[i] = x[i - 1] - 1 * precision
		    y[i] = y[i - 1] 
		elif val == 3: 
			x[i] = x[i - 1] 
			y[i] = y[i - 1] + 1 * precision
		else: 
			x[i] = x[i - 1] 
			y[i] = y[i - 1] - 1 * precision

	return x, y


# Creates a line-like virtual path 
def create_virtual_path_line(initial_loc, number_of_points, precision):

	# 50 ms precision, 20 points per second -> distance traversed for 1 m/s walk equals 0.05 m
	# Creating two arrays for containing x and y coordinates of size equals to the number of points and filled up with 0's 
	x = np.zeros(number_of_points) 
	x[0] = initial_loc[0]
	y = np.zeros(number_of_points) 
  	y[0] = initial_loc[1]

	# filling the coordinates with random variables 
	for i in range(1, number_of_points): 
		
		val = 1
		if val == 1: 
		    x[i] = x[i - 1] + 1 * precision
		    y[i] = y[i - 1] 
		elif val == 2: 
		    x[i] = x[i - 1] - 1 * precision
		    y[i] = y[i - 1] 
		elif val == 3: 
			x[i] = x[i - 1] 
			y[i] = y[i - 1] + 1 * precision
		else: 
			x[i] = x[i - 1] 
			y[i] = y[i - 1] - 1 * precision

	return x, y



