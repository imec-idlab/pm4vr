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
		w_i_sum = calculate_env_vectors(user, env, sum_distance)

		# Equation 7 from Bachmann et al.
		u_j_sum = calculate_other_users_vector(user, users, sum_distance, gamma)

		# Equation 1 from Bachmann et al.
		# (Jakob) Turning the delta vectors into force vectors requires a negation, I think
		force_vec = - (w_i_sum + u_j_sum)
		force_vectors.append(force_vec)

	return force_vectors


# This is actually my attempt at (still not correctly) implementing Equation 5 from Bachmann et al. 
def calculate_sum_distance(user, users, env):

	dist = 0.0

	# Calculating the force vector for the environmental segments (Equation 2 from Bachmann et al.)
	for env_segment in env:

		start = env_segment[0]
		stop = env_segment[1]
		point = user.get_phy_loc()

		d_i = get_vector_to_segment(point, start, stop)
		dist += np.linalg.norm(d_i)

		# (Jakob) Checks that it gives the same result as what was here before, sanity check
		assert abs(abs(np.cross(stop - start, point - start) / np.linalg.norm(stop - start)) -  np.linalg.norm(get_vector_to_segment(point, start, stop))) < 10e-5

	# Calculating the force vector for other users 
	for other_user in users:
		
		# Since all users are stored int he users variable, the idea here is not calculate the distance of a user from itself
		if user.identity != other_user.identity:

			# Check Equation 3 in Bachmann et al.
			# (Jakob) Using kappa here makes no sense
			#kappa = calculate_kappa(user, other_user)
			# Check Equation 4 from Bachmann et al.
			h_j = (user.get_phy_loc() - other_user.get_phy_loc())
			dist += np.linalg.norm(h_j)

	return dist


# Implements the APF-R algorithm from Bachmann et al.
def apf_r_env(step, user, env, threshold):

	# The idea here is to capture if a rotation has happended, as it is not desired. 
	number_of_rotations = 0

	# If too close to any environmental segment just rotate the user physically by 180 degrees
	for env_segment in env:

		start = env_segment[0]
		end = env_segment[1]
		point = user.get_phy_loc() + step

		if np.linalg.norm(get_vector_to_segment(point, start, end)) < threshold and len(user.phy_locations) > 1:

			# The idea here is to make the step in the previous direction (i.e., 180 degree rotation)
			number_of_rotations += 1 
			return -step, number_of_rotations

	# if not too close to any environmental obstacle and/or users, don't modify the calculated steps
	return step, number_of_rotations



# Implements the APF-R algorithm from Bachmann et al.
def apf_r_usr(step, user, users, threshold):

	# The idea here is to capture if a rotation has happended, as it is not desired. 
	number_of_rotations = 0

	# If too close to any other user just rotate the user physically by 180 degrees
	for other_user in users: 
		
		if user.identity != other_user.identity:
			proposed_loc = user.get_phy_loc() + step
			dist = np.linalg.norm(other_user.get_phy_loc() - proposed_loc)
			
			if dist < threshold:

				# The idea here is to make the step in the previous direction (i.e., 180 degree rotation)
				number_of_rotations += 1 
				return -step, number_of_rotations

	# if not too close to any environmental obstacle and/or users, don't modify the calculated steps
	return step, number_of_rotations


# The force vectors above indicate how much the user should move in a certain physical direction in order to optimally mitigate collisions 
# with other users and environmental obstacles. However, the goal is to make the steering movements imperceptible for the user in the virtual
# world. Hence, this function calculates the maximum steering rate that can be applied for that purpose.  
def max_rotation(users, movement_threshold, radius, resolution):

	moving_rates = []

	# The idea is to iterate through all users and calculate their maximum moving rates. See Equation 8 from Bachmann et al. for details.
	for user in users:

		# (Jakob) Setting this to a fixed 1 for now. Below, the baseRate was missing (I think).
		# Tried to fix some stuff but unfinished
		### DEBUG MODE
		moving_rates.append(1)
		continue
		### END

		if len(user.phy_locations) > 1:
			# (Jakob) This way previously _multiplied_ by the resolution but it should be divided!
			linear_velocity = np.linalg.norm(user.get_phy_loc() - user.get_phy_loc(-1)) / resolution
		
		else:
		
			linear_velocity = user.speed

		moving_rate_base = 1.5 * resolution

		if linear_velocity > movement_threshold:

			moving_rate = (360 * linear_velocity / (2 * math.pi * radius) % 360)
			
		if moving_rate < 0.261 * math.pi / 180:
			
			moving_rates.append(max(min(moving_rate, movement_threshold), 0))
		
		else:
			
			moving_rates.append(0.261)

	return moving_rates





# See Equation 3 from Bachmann et al. for details. 
def calculate_kappa(user, other_user):

	if len(user.phy_locations) == 1:
		return 0.0

	# (Jakob) get vector from this to other user AND the other way around, both are needed!
	vector_to_other = other_user.get_phy_loc() - user.get_phy_loc()
	vector_from_other = user.get_phy_loc() - other_user.get_phy_loc()
	user_dir = user.get_phy_loc() - user.get_phy_loc(-1)
	other_user_dir = other_user.get_phy_loc() - other_user.get_phy_loc(-1)

	# (Jakob) Angle between two 2d vectors is arccos(a dot b)
	theta_user = math.acos(np.dot(vector_to_other, user_dir))
	theta_other = math.acos(np.dot(vector_from_other, other_user_dir))
	return np.clip(math.cos(theta_user) + math.cos(theta_other) / 2, 0, 1)

	

# See Equation 6 from Bachmann et al. for details.
def calculate_env_vectors(user, env, sum_distance):

	w_i_sum = 0

	for env_segment in env:

		start = env_segment[0]
		stop = env_segment[1]
		point = user.get_phy_loc()

		# (Jakob) I removed all the special cases here, numpy can do everything properly
		# d_i here used to be a length but it has to be a vector to the nearest point!
		# This should work for hor/vert/diag
		d_i = get_vector_to_segment(point, start, stop)
		w_i = (d_i / np.linalg.norm(d_i)) * (sum_distance / np.linalg.norm(d_i))

		w_i_sum += w_i
	return w_i_sum



# See Equation 7 from Bachmann et al. for details.
def calculate_other_users_vector(user, users, sum_distance, gamma):
	u_j_sum = 0

	for other_user in users:
		if other_user.identity == user.identity:
			continue
		kappa = calculate_kappa(user, other_user)
		# Check Equation 4 from Bachmann et al.

		# (Jakob) Same cleanup as for the env vectors
		h_j = (user.get_phy_loc() - other_user.get_phy_loc())
		u_j = kappa * (h_j / np.linalg.norm(h_j)) * (sum_distance / (np.linalg.norm(h_j) * gamma))
		u_j_sum += u_j
	return u_j_sum



# The point here is not to calculate next step, but to steer the user in the physical world as much as possible 
# (i.e., maintaining imperceptibility) in the direction suggested by the force vectors. Since the goal of the simulator is to
# provide mapping between virtual and physical world, this steering can be modeled solely by the current and next locations of 
# the users.   
def calculate_next_physical_step(user, moving_rate, force_vector, max_move_rate, resolution):
	user_angle = np.arctan2( user.get_phy_loc()[1] - user.get_phy_loc(-1)[1], user.get_phy_loc()[0] - user.get_phy_loc(-1)[0])
	desired_angle = np.arctan2(force_vector[1], force_vector[0])


	if user_angle < 0.0:

		user_angle += 2 * math.pi

	if desired_angle < 0.0:

		desired_angle += 2 * math.pi

	# If the desired moving rotation is smaller that the threshold, everything seems easy
	if abs(moving_rate) < max_move_rate * math.pi / 180:
		step_temp = force_vector / np.linalg.norm(force_vector)

	else:
		step_temp = np.empty(2)
		user_plus = user_angle * 180 / math.pi + max_move_rate
		user_minus = user_angle * 180 / math.pi - max_move_rate

		if user_minus < 0.0:
			user_minus += 360.0
		desired_angle *= 180 / math.pi

		# (Jakob) Move with highest allowed move rate for now, scaling by force vector still todo
		final_dir = user_plus if abs(user_plus - desired_angle) <= abs(user_minus - desired_angle) else user_minus

		# (Jakob) https://math.stackexchange.com/a/3534251 For step size 1, scales below
		step_temp[0] = math.cos(final_dir * math.pi / 180)
		step_temp[1] = math.sin(final_dir * math.pi / 180)

		# step_temp[0] = force_vector[0] / np.linalg.norm(force_vector)
		#
		# # Use plus, as it is closer to the desired angle
		# if abs(desired_angle  * 180 / math.pi - user_plus) < abs(desired_angle  * 180 / math.pi - user_minus):
		#
		# 	step_temp[1] = math.tan(user_plus * math.pi / 180) * step_temp[0]
		#
		# # Use minus, as it is closer to the desired angle
		# else:
		#
		# 	step_temp[1] = math.tan(user_minus * math.pi / 180) * step_temp[0]

	# Scale to precision
	# (Jakob) This now assumes a speed of 1 unit per second, more advanced approach todo
	step = resolution * step_temp / (np.linalg.norm(step_temp))

	# if step_temp[1] > 0.0:
	#
	# 	step[1] = math.sqrt(pow(resolution, 2) / (1 + pow(step_temp[0], 2) / pow(step_temp[1], 2)))
	#
	# else:
	# 	step[1] = -math.sqrt(pow(resolution, 2) / (1 + pow(step_temp[0], 2) / pow(step_temp[1], 2)))
	# step[0] = step_temp[0] * step[1] / step_temp[1]

	return step

def find_nearest_point_on_segment(point, s1, s2):
	# https://math.stackexchange.com/a/330329
	#The segment [s1,s2] is part of the line s1 + t(s2-s1), and each point in that line is on the segment iff t in [0,1]

	t_closest = np.dot(point - s1, s2 - s1) / (np.linalg.norm(s2-s1) ** 2)
	t_on_segment = np.clip(t_closest, 0, 1)
	return s1 + t_on_segment * (s2 -s1)


def get_vector_to_segment(point, s1, s2):
	return find_nearest_point_on_segment(point, s1, s2) - point
