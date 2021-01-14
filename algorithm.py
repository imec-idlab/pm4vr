import numpy as np
np.seterr(divide='ignore')
import math


# Calculation of the force vectors for the APH-RDW algorithm. For details, please check:
# Bachmann, Eric R., et al. "Multi-user redirected walking and resetting using artificial potential 
# fields." IEEE transactions on visualization and computer graphics 25.5 (2019): 2022-2031.
def apf_rdw(users, env, gamma):


	force_vectors = []

	# The idea here is to calculate the optimal movement vector for each user, so that the collisions among the 
	# users, as well as between the users and the environmental obstacles are avoided without the users realizing  
	# they are being steered in the physical world.  
	for user in users:

		# Equation 5 from Bachmann et al.
		sum_distance = calculate_sum_distance(user, users, env)

		# Equation 6 from Bachmann et al.
		dist_x_env, dist_y_env = calculate_env_vectors(user, env, sum_distance)

		# Equation 7 from Bachmann et al.
		dist_x_usr, dist_y_usr = calculate_other_users_vector(user, users, sum_distance, gamma) 

		# Equation 1 from Bachmann et al.
		force_vectors.append((dist_x_env + dist_x_usr, dist_y_env + dist_y_usr))

	return force_vectors


# This is actually my attempt at (still not correctly) implementing Equation 5 from Bachmann et al. 
def calculate_sum_distance(user, users, env):

	dist = 0.0

	# Calculating the force vector for the environmental segments (Equation 2 from Bachmann et al.)
	for env_segment in env:

		p1 = env_segment[0]
		p2 = env_segment[1]
		p3 = np.array([user["phy_path_x"][-1], user["phy_path_y"][-1]])

		dist += np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)

	# Calculating the force vector for other users 
	for other_user in users:
		
		# Since all users are stored int he users variable, the idea here is not calculate the distance of a user from itself
		if user["identity"] != other_user["identity"]:

			# Check Equation 3 in Bachmann et al.
			kapa = calculate_kapa(user, other_user)
			# Check Equation 4 from Bachmann et al.
			dist += kapa * math.sqrt(pow(other_user["phy_path_x"][-1] - user["phy_path_x"][-1], 2) + pow(other_user["phy_path_y"][-1] - user["phy_path_y"][-1], 2))

	return dist


# Implements the APF-R algorithm from Bachmann et al.
def apf_r(x_step, y_step, user, users, env, threshold):

	# If too close to any environmental segment just rotate the user physically by 180 degrees
	for env_segment in env:

		p1 = env_segment[0]
		p2 = env_segment[1]
		p3 = np.array([user["phy_path_x"][-1] + x_step, user["phy_path_y"][-1] + y_step])

		if np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1) < threshold and len(user["phy_path_x"]) > 1:

			# The idea here is to make the step in the previous direction (i.e., 180 degree rotation)
			return -user["phy_path_x"][-1] + user["phy_path_x"][-2], -user["phy_path_y"][-1] + user["phy_path_y"][-2]


	# If too close to any other user just rotate the user physically by 180 degrees
	for other_user in users: 
		
		if user["identity"] != other_user["identity"]:

			dist = math.sqrt(pow(other_user["phy_path_x"][-1] - user["phy_path_x"][-1] - x_step, 2) + pow(other_user["phy_path_y"][-1] - user["phy_path_y"][-1] - y_step, 2))

			if dist < threshold:

				# The idea here is to make the step in the previous direction (i.e., 180 degree rotation)
				return -user["phy_path_x"][-2] + user["phy_path_x"][-1], -user["phy_path_y"][-2] + user["phy_path_y"][-1]

	# if not too close to any environmental obstacle and/or users, don't modify the calculated steps
	return x_step, y_step



# The force vectors above indicate how much the user should move in a certain physical direction in order to optimally mitigate collisions 
# with other users and environmental obstacles. However, the goal is to make the steering movements imperceptible for the user in the virtual
# world. Hence, this function calculates the maximum steering rate that can be applied for that purpose.  
def max_rotation(users, movement_threshold, radius, resolution):

	moving_rates = []

	# The idea is to iterate through all users and calculate their maximum moving rates. See Equation 8 from Bachmann et al. for details.
	for user in users:

		if len(user["phy_path_x"]) > 1:

			linear_velocity = math.sqrt(pow(user["phy_path_x"][-1] - user["phy_path_x"][-2], 2) + pow(user["phy_path_y"][-1] - user["phy_path_y"][-2], 2)) / resolution
		
		else:
		
			linear_velocity = user["speed"]

		moving_rate = 0.0

		if linear_velocity > movement_threshold:

			moving_rate = 360 * linear_velocity / (2 * math.pi * radius)
			
		if moving_rate < 0.261:
			
			moving_rates.append(max(min(moving_rate, movement_threshold), 0))
		
		else:
			
			moving_rates.append(0.261)

	return moving_rates



# See Equation 3 from Bachmann et al. for details. 
def calculate_kapa(user, other_user):

	try:
		ref_angle = np.arctan2((other_user["phy_path_y"][-1] - user["phy_path_y"][-1]), (other_user["phy_path_x"][-1] - user["phy_path_x"][-1]))
		user_angle = np.arctan2((user["virt_path_y"][len(user["phy_path_x"])] - user["virt_path_y"][len(user["phy_path_x"])-1]), (user["virt_path_x"][len(user["phy_path_x"])] - user["virt_path_x"][len(user["phy_path_x"])-1]))
		other_user_angle = np.arctan2((other_user["virt_path_y"][len(other_user["phy_path_x"])] - other_user["virt_path_y"][len(other_user["phy_path_x"])-1]), (other_user["virt_path_x"][len(other_user["phy_path_x"])] - other_user["virt_path_x"][len(other_user["phy_path_x"])-1]))
		
		return max(min(math.cos(ref_angle - user_angle) + math.cos(ref_angle - other_user_angle) / 2, 1), 0)

	except ZeroDivisionError:

		return 1.0
	

# See Equation 6 from Bachmann et al. for details.
def calculate_env_vectors(user, env, sum_distance):

	dist_x = 0.0
	dist_y = 0.0

	for env_segment in env:

		p1 = env_segment[0]
		p2 = env_segment[1]
		p3 = np.array([user["phy_path_x"][-1], user["phy_path_y"][-1]])

		# If vertical line segment (it can be either vertical of horizontal)
		if p1[0] == p2[0]:

			dist_x_temp = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)

			# Taking care of the direction of the resulting vector
			if p1[0] > user["phy_path_x"][-1]:

				dist_x -= sum_distance / dist_x_temp 

			else:

				dist_x += sum_distance / dist_x_temp

		# If horizontal line segment
		else:

			dist_y_temp = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)

			if p1[1] > user["phy_path_y"][-1]:

				dist_y += sum_distance / dist_y_temp 

			else:

				dist_y -= sum_distance / dist_y_temp 

	return dist_x, dist_y



# See Equation 7 from Bachmann et al. for details.
def calculate_other_users_vector(user, users, sum_distance, gamma):

	dist_x = 0.0
	dist_y = 0.0

	for other_user in users:

		# To avoid calculating the distance with itself
		if user["identity"] != other_user["identity"]:

			kapa = calculate_kapa(user, other_user)

			if kapa > 0:

				abs_dist = kapa * math.sqrt(pow(other_user["phy_path_x"][-1] - user["phy_path_x"][-1], 2) + pow(other_user["phy_path_y"][-1] - user["phy_path_y"][-1], 2))

				if other_user["phy_path_x"][-1] > user["phy_path_x"][-1]:

					dist_x -= kapa * abs(other_user["phy_path_x"][-1] - user["phy_path_x"][-1]) * sum_distance / pow(abs_dist, 1 + gamma) 

				elif other_user["phy_path_x"][-1] < user["phy_path_x"][-1]:

					dist_x += kapa * abs(other_user["phy_path_x"][-1] - user["phy_path_x"][-1]) * sum_distance / pow(abs_dist, 1 + gamma)


				if other_user["phy_path_y"][-1] > user["phy_path_y"][-1]:

					dist_y -= kapa * abs(other_user["phy_path_y"][-1] - user["phy_path_y"][-1]) * sum_distance / pow(abs_dist, 1 + gamma)

				elif other_user["phy_path_y"][-1] < user["phy_path_y"][-1]:

					dist_y += kapa * abs(other_user["phy_path_y"][-1] - user["phy_path_y"][-1]) * sum_distance / pow(abs_dist, 1 + gamma)


	return dist_x, dist_y



# The point here is not to calculate next step, but to steer the user in the physical world as much as possible 
# (i.e., maintaining imperceptibility) in the direction suggested by the force vectors. Since the goal of the simulator is to
# provide mapping between virtual and physical world, this steering can be modeled solely by the current and next locations of 
# the users.   
def calculate_next_physical_step(user, moving_rate, force_vector, max_move_rate, precision):

	user_angle = np.arctan2((user["virt_path_y"][len(user["phy_path_x"])] - user["virt_path_y"][len(user["phy_path_x"])-1]), (user["virt_path_x"][len(user["phy_path_x"])] - user["virt_path_x"][len(user["phy_path_x"])-1]))
	desired_angle = np.arctan2(force_vector[0], force_vector[1])

	if user_angle < 0.0:

		user_angle += 2 * math.pi

	if desired_angle < 0.0:

		desired_angle += 2 * math.pi


	# If the desired moving rotation is smaller that the threshold, everything seems easy
	if abs(user_angle - desired_angle) < max_move_rate * math.pi / 180:

		x_step_temp = force_vector[0] / math.sqrt(pow(force_vector[0],2) + pow(force_vector[1],2))
		y_step_temp = force_vector[1] / math.sqrt(pow(force_vector[0],2) + pow(force_vector[1],2))

	else:

		user_plus = user_angle * 180 / math.pi + max_move_rate
		user_minus = user_angle * 180 / math.pi - max_move_rate
		
		if user_minus < 0.0:
			user_minus += 360.0

		x_step_temp = force_vector[0] / math.sqrt(pow(force_vector[0],2) + pow(force_vector[1],2))

		# Use plus, as it is closer to the desired angle	
		if abs(desired_angle  * 180 / math.pi - user_plus) < abs(desired_angle  * 180 / math.pi - user_minus):

			y_step_temp = math.tan(user_plus * math.pi / 180) * x_step_temp	
		
		# Use minus, as it is closer to the desired angle	
		else:

			y_step_temp = math.tan(user_minus * math.pi / 180) * x_step_temp	


	# Scale to precision
	if y_step_temp > 0.0:
		
		y_step = math.sqrt(pow(precision, 2) / (1 + pow(x_step_temp, 2) / pow(y_step_temp, 2)))
	
	else:
		y_step = -math.sqrt(pow(precision, 2) / (1 + pow(x_step_temp, 2) / pow(y_step_temp, 2)))
	
	x_step = x_step_temp * y_step / y_step_temp

	return x_step, y_step



