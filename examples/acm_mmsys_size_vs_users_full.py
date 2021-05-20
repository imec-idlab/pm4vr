#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The goal of this experiment is to run the simulation for different sizes of a squared environment and varying number 
of users, and to capture both macro and micro-scale performance metrics. Moreover, an additional goal is to benchmark 
the execution time.

Macro-scale metrics: number of redirections per user, average distance between redirections
micro-scale metrics: mean absolute (MAE) and mean squared error (MSE) of 100 ms predictions

Note: this is a minimal working example and we acknowledge that both the macro and micro-scale performance metrics 
could be optimized. For example, the short-term prediction of future locations is not optimized and the initial locations 
of the users are selected randomly. 
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"


import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

import environment
from user import User
import visualization
import algorithm
import prediction
from algorithm import rad
import numpy as np
from collections import defaultdict
import time
import random

# ---------- Parameters ----------------------------
steps_per_second = 10 # Number of data points (both virtual and physical) per second (we're doing short-term predictions on a 100 ms scale)
duration = 60 * 60 # Duration of the experiment in seconds
gamma = 1.5 # Gamma causes the influence of users to fall off exponentially instead of linearly
radius = 7.5 # r is the radius of the arc on which a walking user is being redirected
max_move_rate = 15 # Maximum rotational movement the user can tolerate without noticing

# --------------------------------------------------

env_sizes = [5.0, 7.5, 10.0, 12.5]
num_users = [1, 2, 4, 6, 8]

for env_size in env_sizes:

	print("# Squared environment of sizes " + str(env_size) + 'x' + str(env_size) + 'm^2') 

	users = []

	for num_user in num_users:

		print("# Number of users equals " + str(num_user))

		time1 = time.time() # We want to benchmark the execution time of each experiment

		# Define the shape and size of the environment.
		env = environment.define_square(env_size)

		rdw = algorithm.RedirectedWalker(duration = duration, steps_per_second = steps_per_second, gamma = gamma,
										 base_rate=rad(1.5), max_move_rate=rad(15), max_head_rate=rad(30),
										 velocity_thresh = 0.1, ang_compress_scale=0.85, ang_amplify_scale=1.3,
										 scale_multiplier=2.5, radius=7.5, t_a_norm=15.0, env=env)

		# Define the users by defining their virtual movement trajectory. 
		usr1 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 1)
		usr1.fill_virtual_path(rdw.steps, rdw.delta_t, 1)

		users = [usr1]
		
		if num_user > 1:

			usr2 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 2)
			usr2.fill_virtual_path(rdw.steps, rdw.delta_t, 2)

			users = [usr1, usr2]

		if num_user > 2:

			usr3 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 3)
			usr3.fill_virtual_path(rdw.steps, rdw.delta_t, 3)
			usr4 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 4)
			usr4.fill_virtual_path(rdw.steps, rdw.delta_t, 4)

			users = [usr1, usr2, usr3, usr4]

		if num_user > 4:

			usr5 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 5)
			usr5.fill_virtual_path(rdw.steps, rdw.delta_t, 5)
			usr6 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 6)
			usr6.fill_virtual_path(rdw.steps, rdw.delta_t, 6)

			users = [usr1, usr2, usr3, usr4, usr5, usr6]

		if num_user > 6:

			usr7 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 7)
			usr7.fill_virtual_path(rdw.steps, rdw.delta_t, 7)
			usr8 = User([random.uniform(-env_size / 2, env_size / 2), random.uniform(-env_size / 2, env_size / 2)], 1.0, 8)
			usr8.fill_virtual_path(rdw.steps, rdw.delta_t, 8)

			users = [usr1, usr2, usr3, usr4, usr5, usr6, usr7, usr8]


		# Iterate through all steps of the simulation (!! check simulation_time parameter, as it includes the resolution !!).

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

				# (Jakob) Redirection was implemented with a 180 degree rotation. The paper however proposes to rotate towards
				# the force vector. This moves the user away from all obstacles (walls and other users) optimally meaning
				# there's no reason to check for users and walls separately.
				# The following method checks if a collision is about to happen (threshold selected arbitrarily for now)
				reset_step 	= rdw.reset_if_needed(force_vectors[iter_temp], env_vectors[iter_temp], user_vectors[iter_temp], threshold = 100)

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

		time2 = time.time() # We want to benchmark the execution time of each experiment


		# Visualization of the virtual and physical paths for all users
		# visualization.visualize_paths(rdw.steps, users)

		print("# Execution time of this experiment [sec]")
		print("time_" + str(num_user) + " = " + str(time2 - time1))

		print("# Number of resets per each user")
		print("resets_" + str(num_user) + " = " + str(num_resets_per_users.values())) 

		# Performance metrics
		for user in users:
			print("# User " + str(user.identity))
			print("dist_usr_" + str(user.identity) + ' = ' + str(distance_between_resets_per_user[user.identity])) 

		# Micro-scale performance metrics (!! Substantially longer simulation time !!) 
		for user in users:

			mse = prediction.make_and_evaluate_predictions(user.get_phy_path(), 9, 1, 2, 0.8)
			print("# User " + str(user.identity))
			print("mse_" + str(user.identity) + ' = ' + str(mse))

		time3 = time.time() # We want to benchmark the execution time of each experiment

		print("# Execution time of the experiment with predictions [sec]")
		print("time_full_" + str(num_user) + " = " + str(time3 - time1))

		print("# ------------------------------------------------")
		print()

	print("# ++++++++++++++++++++++++++++++++++++++++++")
	print()

