import environment
import user
import visualization
import algorithm

# ---------- Parameters ----------------------------

simulation_time = 10 * 200 # 10 sec with the resolution of 20 points per second (50 ms)
resolution = 0.05 # Time steps, time discrimination
gamma = 1.4 # gamma causes the influence of users to fall off exponentially instead of linearly
radius = 7.5 # r is the radius of the arc on which a walking user is being redirected
movement_threshold = 0.1 # If linear velocity is below a minimum movement threshold moving_rate = 0
max_move_rate = 15 # Maximum rotational movement the user can tolerate without noticing

# --------------------------------------------------

# Define the shape and size of the environment.
env = environment.define_square(20.0)

# Define the users by defining their virtual movement trajectory. 
usr1 = user.create([-4.0, -4.0], 0.0, 1)
usr1["virt_path_x"], usr1["virt_path_y"] = user.create_virtual_path_random([4.0, 4.0], simulation_time, resolution)

usr2 = user.create([4.0, 4.0], 0.0, 2)
usr2["virt_path_x"], usr2["virt_path_y"] = user.create_virtual_path_random([-4.0, 4.0], simulation_time, resolution)

usr3 = user.create([-4.0, 4.0], 0.0, 3)
usr3["virt_path_x"], usr3["virt_path_y"] = user.create_virtual_path_random([4.0, -4.0],  simulation_time, resolution)

usr4 = user.create([4.0, -4.0], 0.0, 4)
usr4["virt_path_x"], usr4["virt_path_y"] = user.create_virtual_path_random([-4.0, -4.0],  simulation_time, resolution)

users = [usr1, usr2, usr3, usr4]


# Iterate through all steps of the simulation (!! check simulation_time parameter, as it includes the resolution !!).

num_rotations_per_users = {}

for time_iter in range(0, simulation_time - 1):

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
		x_step, y_step = algorithm.calculate_next_physical_step(user, moving_rates[iter_temp], force_vectors[iter_temp], max_move_rate, resolution)
		
		# If the calculated x_step and y_step would result in hitting another user or an environmental obstacle (i.e., if the 
		# distance between the user and another entity (user, obstacle) is lower than a certain threshold), invoke the APF-R 
		# algorithm (in essence, rotating the user by 360 degrees in the virtual words, while in the physical world the user 
		# rotates for 180 degrees only - away from the nearby entity). rotation_flag = 1 indicates that a rotation has happened.
		final_x_step, final_y_step, rotation_flag = algorithm.apf_r(x_step, y_step, user, users, env, threshold = 0.5)

		# Capturing the number of ratations (i.e., invocations of the APF-R algrothm) per each user
		try:
			num_rotations_per_users[user["identity"]] += 1
		
		except:
			num_rotations_per_users[user["identity"]]  = 1


		# Update the user's physical trajectory with the newest location
		user['phy_path_x'].append(user['phy_path_x'][time_iter - 1] + x_step)
		user['phy_path_y'].append(user['phy_path_y'][time_iter - 1] + y_step)

		iter_temp += 1

# Visualization of the virtual and physical paths for all users
visualization.visualize_paths(simulation_time, users)

# Performance metrics
print num_rotations_per_users 


