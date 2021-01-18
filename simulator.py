import environment
from user import User
import visualization
import algorithm
from algorithm import rad
import numpy as np
from collections import defaultdict

# ---------- Parameters ----------------------------
steps_per_second = 20
duration = 70
gamma = 1.4 # gamma causes the influence of users to fall off exponentially instead of linearly
radius = 7.5 # r is the radius of the arc on which a walking user is being redirected
movement_threshold = 0.1 # If linear velocity is below a minimum movement threshold moving_rate = 0
max_move_rate = 15 # Maximum rotational movement the user can tolerate without noticing

# --------------------------------------------------

# Define the shape and size of the environment.
env = environment.define_square(20.0)


rdw = algorithm.RedirectedWalker(duration = duration, steps_per_second = steps_per_second, gamma = 1.5,
								 base_rate=rad(1.5), max_move_rate=rad(15), max_head_rate=rad(30),
								 velocity_thresh = 0.1, ang_compress_scale=0.85, ang_amplify_scale=1.3,
								 scale_multiplier=2.5, radius=7.5, t_a_norm=15.0, env=env)

# (Jakob) Only one user for now! Wall stuff should work first (but doesn't yet)
# Define the users by defining their virtual movement trajectory. 
usr1 = User([-9.0, 0.0], 1.0, 1)
usr1.fill_virtual_path(rdw.steps, rdw.delta_t, 1)
usr2 = User([9.0, 0.0], 1.0, 2)
usr2.fill_virtual_path(rdw.steps, rdw.delta_t, 2)



users = [usr1, usr2]


# Iterate through all steps of the simulation (!! check simulation_time parameter, as it includes the resolution !!).

num_rotations_per_users = defaultdict(int)

for user in users:
	#Let the first step be taken without redirection to kickstart stuff
	user.phy_locations.append(user.virt_locations[1])

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

		# This is just for storing the number of rotations per user
		num_rotations_per_users[user.identity] += reset_occurred

		# Update the user's physical trajectory with the newest location
		user.phy_locations.append(user.get_phy_loc() + step)

		iter_temp += 1

# Visualization of the virtual and physical paths for all users
visualization.visualize_paths(rdw.steps, users)

# Performance metrics


