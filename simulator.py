#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Main script, should be modified according to the needs of an experiment. Example modifications can be found in
the 'examples' folder. The work is based on:

1) Bachmann, Eric R., et al. "Multi-user redirected walking and resetting using artificial potential fields." 
IEEE transactions on visualization and computer graphics 25.5 (2019): 2022-2031.

2) Hou, Xueshi, et al. "Head and body motion prediction to enable mobile VR experiences with low latency." 
2019 IEEE Global Communications Conference (GLOBECOM). IEEE, 2019.
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"


import environment
from user import User
import visualization
import algorithm
from algorithm import rad
import numpy as np
from collections import defaultdict
import pprint


# ---------- Simulation parameters ----------------------------
steps_per_second = 10 # Number of data points (both virtual and physical) per second (we're doing short-term predictions on a 100 ms scale)
duration = 100 # Duration of the experiment in seconds
gamma = 1.4 # Gamma causes the influence of users to fall off exponentially instead of linearly
radius = 7.5 # r is the radius of the arc on which a walking user is being redirected
movement_threshold = 0.1 # If linear velocity is below a minimum movement threshold moving_rate = 0
max_move_rate = 15 # Maximum rotational movement the user can tolerate without noticing

# --------------------------------------------------

# Define the shape and size of the environment.
env = environment.define_square(10.0)


rdw = algorithm.RedirectedWalker(duration = duration, steps_per_second = steps_per_second, gamma = 1.5,
								 base_rate=rad(1.5), max_move_rate=rad(15), max_head_rate=rad(30),
								 velocity_thresh = 0.1, ang_compress_scale=0.85, ang_amplify_scale=1.3,
								 scale_multiplier=2.5, radius=7.5, t_a_norm=15.0, env=env)

# Define the users by defining their virtual movement trajectory. 
usr1 = User([2.0, 2.0], 1.0, 1)
usr1.fill_virtual_path(rdw.steps, rdw.delta_t, None)
usr2 = User([2.0, 1.0], 1.0, 2)
usr2.fill_virtual_path(rdw.steps, rdw.delta_t, None)
usr3 = User([2.0, 0.0], 1.0, 3)
usr3.fill_virtual_path(rdw.steps, rdw.delta_t, None)


users = [usr1, usr2, usr3]

# --------------------------------------------------

# Iterate through all steps of the simulation (check simulation_time parameter, as it includes the resolution).
num_resets_per_users = defaultdict(int) # Definition of the performance metric entitled number of resets per user
distance_between_resets_per_user = defaultdict(int) # Storing all distances between resets per user

for user in users:
	#Let the first step be taken without redirection to kick-start stuff
	user.phy_locations.append(user.virt_locations[1])
	distance_between_resets_per_user[user.identity] = []
	distance_between_resets_per_user[user.identity].append(0.0) 

for time_iter in range(0, rdw.steps - 2):

	# At each step of the evaluation, calculate force_vectors and moving_rates. Force vectors are used to define the 
	# optimal physical movement direction for each user (i.e., to avoid hitting environmental obstacles and other users). 
	# Moving rates provide constraints on how much the user can be steered in the physical world without noticing it 
	# in the virtual one. Force vectors are (at the moment) calculated using the APF-RDW algorithm
	force_vectors, env_vectors, user_vectors = rdw.calculate_force_vectors(users)
	moving_rates = rdw.calculate_max_rotations(users)

	iter_temp = 0

	# Iterate through all users and calculate their next physical step based on the force vectors and moving rates. 
	for user in users:

		# x_step and y_step define the user's physical offset from the current location
		step = rdw.calculate_next_physical_step(user, moving_rates[iter_temp], force_vectors[iter_temp])

		# The following method checks if a collision is about to happen (threshold selected arbitrarily for now)
		reset_step 	= rdw.reset_if_needed(force_vectors[iter_temp], env_vectors[iter_temp], user_vectors[iter_temp], threshold = 50)

		reset_occurred = reset_step is not None
		if reset_occurred:
			step = reset_step
			# Create a new instance in the list representing distances passed without a reset
			distance_between_resets_per_user[user.identity].append(algorithm.norm(step))  
		else:
			# Add the step in latest instance of the list representing distances passed without a reset
			distance_between_resets_per_user[user.identity][-1] += algorithm.norm(step)  


		# This is just for storing the number of rotations per user
		num_resets_per_users[user.identity] += reset_occurred

		# Update the user's physical trajectory with the newest location
		user.phy_locations.append(user.get_phy_loc() + step)

		iter_temp += 1


# ---------- Make your decisions ----------------------------

# Visualization of the virtual and physical paths for all users 
visualization.visualize_paths(rdw.steps, users)

# Macro-scale performance metrics 
# print(num_resets_per_users) 
pprint.pprint(distance_between_resets_per_user) 

# Micro-scale performance metrics (!! Substantially longer simulation time !!) 
print(usr1.get_phy_path())



