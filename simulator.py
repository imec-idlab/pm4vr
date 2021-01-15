import environment
from user import User
import visualization
import algorithm

# ---------- Parameters ----------------------------

steps_per_second = 20
simulation_time = 20 * steps_per_second # 10 sec with the resolution of 20 points per second (50 ms)
resolution = 1 / steps_per_second # Time steps, time discrimination
gamma = 1.4 # gamma causes the influence of users to fall off exponentially instead of linearly
radius = 7.5 # r is the radius of the arc on which a walking user is being redirected
movement_threshold = 0.1 # If linear velocity is below a minimum movement threshold moving_rate = 0
max_move_rate = 15 # Maximum rotational movement the user can tolerate without noticing

# --------------------------------------------------

# simulation_time = 9
# Define the shape and size of the environment.
env = environment.define_square(20.0)

# (Jakob) Only one user for now! Wall stuff should work first (but doesn't yet)
# Define the users by defining their virtual movement trajectory. 
usr1 = User([9, 0.0], 1.0, 1)
usr1.fill_virtual_path(simulation_time, resolution, 1)
# usr2 = User([5, 0.0], 0.0, 2)
# usr2.fill_virtual_path(simulation_time, resolution, 2)

# usr3 = user.create([-4.0, 4.0], 0.0, 3)
# usr3["virt_path_x"], usr3["virt_path_y"] = user.create_virtual_path_random([4.0, -4.0],  simulation_time, resolution)
#
# usr4 = user.create([4.0, -4.0], 0.0, 4)
# usr4["virt_path_x"], usr4["virt_path_y"] = user.create_virtual_path_random([-4.0, -4.0],  simulation_time, resolution)

users = [usr1]


# Iterate through all steps of the simulation (!! check simulation_time parameter, as it includes the resolution !!).

num_rotations_per_users = {}

for user in users:
	#Let the first step be taken without redirection to kickstart stuff
	user.phy_locations.append(user.virt_locations[1])

for time_iter in range(0, simulation_time - 2):

	# At each step of the evaluation, calculate force_vectors and moving_rates. Force vectors are used to define the 
	# optimal physical movement direction for each user (i.e., to avoid hitting environmental obstacles and other users). 
	# Moving rates provide constraints on how much the user can be steered in the physical world without noticing it 
	# in the virtual one. Force vectors are (at the moment) calculated using the APF-RDW algorithm
	force_vectors = algorithm.apf_rdw(users, env, gamma)
	moving_rates = algorithm.max_rotation(users, movement_threshold, radius, resolution)

	iter_temp = 0

	# Iterate through all users and calculate their next physical step based on the force vectors and moving rates. 
	for user in users:

		# x_step and y_step define the user's physical offset from the current location
		step = algorithm.calculate_next_physical_step(user, moving_rates[iter_temp], force_vectors[iter_temp], max_move_rate, resolution)

		# If the calculated x_step and y_step would result in hitting another user or an environmental obstacle (i.e., if the
		# distance between the user and another entity (user, obstacle) is lower than a certain threshold), invoke the APF-R 
		# algorithm (in essence, rotating the user by 360 degrees in the virtual words, while in the physical world the user 
		# rotates for 180 degrees only - away from the nearby entity). rotation_flag = 1 indicates that a rotation has happened.
		step, rotation_flag_1 	= algorithm.apf_r_env(step, user, env, threshold = 0.5)
		step, rotation_flag_2 = algorithm.apf_r_usr(step, user, users, threshold = 0.5)
		# (Jakob) The output steps above were stored in temp variables but not actually used. They now overwrite step.

		# Capturing the number of rotations (i.e., invocations of the APF-R algorithm) per each user. Rotations can 
		# happen when moving away from an environmental obstacle (rotation_flag_1) or another user (rotation_flag_1).
		# If both of them happen simultaneously, it is counted as one rotation.
		if rotation_flag_1 == 0 and rotation_flag_2 == 0:
			# A rotation didn't occur if both flags are 0.
			rotation_flag = 0

		else:
			# Otherwise a rotation occurred. 
			rotation_flag = 1

		# This is just for storing the number of rotations per user
		try:
			num_rotations_per_users[user.identity] += rotation_flag
		
		except:
			num_rotations_per_users[user.identity]  = rotation_flag


		# Update the user's physical trajectory with the newest location
		user.phy_locations.append(user.get_phy_loc() + step)

		iter_temp += 1

# Visualization of the virtual and physical paths for all users
visualization.visualize_paths(simulation_time, users)

# Performance metrics


