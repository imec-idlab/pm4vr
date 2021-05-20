#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Library implementing the multi-user redirected walking (APH-RDW) and resetting (APH-R) using artificial potential fields.
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"

import numpy as np
np.seterr(divide='ignore')
import math


# All rates in radians!
class Rates:
	def __init__(self):
		self.base_rate = 0  # Default redirection, always possible
		self.moving_rate = 0 # Redirection during walking
		self.head_rate_actual = 0 # The actual head rotation the user is performing now
		self.head_rate_compress = 0 # The farthest the above rotation could be compressed
		self.head_rotate_amplify = 0 # The farthest the above rotation could be amplified

	def get_best_rate(self, desired):
		# TODO head rotation amplification/compression
		return max(self.base_rate, self.moving_rate)

	def scale(self, scale):
		self.moving_rate *= scale
		# TODO head rotation amplification/compression

class RedirectedWalker:
	def __init__(self, *, duration, steps_per_second, gamma, base_rate, max_move_rate, max_head_rate, velocity_thresh,
				 ang_compress_scale, ang_amplify_scale, scale_multiplier, radius, t_a_norm, env):

		self.duration = duration # seconds
		self.steps_per_second = steps_per_second
		self.gamma = gamma
		self.base_rate = base_rate # radians
		self.max_move_rate = max_move_rate # radians
		self.max_head_rate = max_head_rate # radians
		self.velocity_thresh = velocity_thresh # m/s
		self.ang_compress_scale = ang_compress_scale
		self.ang_amplify_scale = ang_amplify_scale
		self.scale_multiplier = scale_multiplier
		self.radius = radius # m
		self.t_a_norm = t_a_norm
		self.env = env

		self.delta_t = 1 / self.steps_per_second
		self.steps = self.duration * self.steps_per_second

	# Calculation of the force vectors for the APH-RDW algorithm. For details, please check:
	# Bachmann, Eric R., et al. "Multi-user redirected walking and resetting using artificial potential
	# fields." IEEE transactions on visualization and computer graphics 25.5 (2019): 2022-2031.
	def calculate_force_vectors(self, users):

		force_vectors = []
		individual_env_vectors = []
		individual_user_vectors = []

		# The idea here is to calculate the optimal movement vector for each user, so that the collisions among the
		# users, as well as between the users and the environmental obstacles are avoided without the users realizing
		# they are being steered in the physical world.
		for user in users:

			# Equation 5 from Bachmann et al.
			sum_distance = self.calculate_sum_distance(user, users)

			# Equation 6 from Bachmann et al.
			env_vectors = self.calculate_env_vectors(user, sum_distance)

			# Equation 7 from Bachmann et al.
			user_vectors = self.calculate_other_users_vector(user, users, sum_distance)

			# Equation 1 from Bachmann et al.
			force_vec = sum(env_vectors) + sum(user_vectors)

			force_vectors.append(force_vec)
			individual_env_vectors.append(env_vectors)
			individual_user_vectors.append(user_vectors)

		return force_vectors, individual_env_vectors, individual_user_vectors


	def calculate_sum_distance(self, user, users):

		dist = 0.0

		# Calculating the force vector for the environmental segments (Equation 2 from Bachmann et al.)
		for env_segment in self.env:

			start = env_segment[0]
			stop = env_segment[1]
			point = user.get_phy_loc()

			d_i = get_vector_from_segment(point, start, stop)
			dist += norm(d_i)

			# (Jakob) Checks that it gives the same result as what was here before, sanity check
			# assert abs(abs(np.cross(stop - start, point - start) / norm(stop - start)) -  norm(get_vector_from_segment(point, start, stop))) < 10e-5

		# Calculating the force vector for other users
		for other_user in users:

			# Since all users are stored int he users variable, the idea here is not calculate the distance of a user from itself
			if user.identity != other_user.identity:

				# Check Equation 3 in Bachmann et al.
				# (Jakob) Using kappa here makes no sense
				#kappa = calculate_kappa(user, other_user)
				# Check Equation 4 from Bachmann et al.
				h_j = (user.get_phy_loc() - other_user.get_phy_loc())
				dist += norm(h_j)

		return dist

	# See Equation 3 from Bachmann et al. for details.
	def calculate_kappa(self, user, other_user):

		if len(user.phy_locations) == 1:
			return 0.0

		# Get vector from this to other user AND the other way around, both are needed!
		vector_to_other = other_user.get_phy_loc() - user.get_phy_loc()
		vector_from_other = user.get_phy_loc() - other_user.get_phy_loc()
		user_dir = user.get_phy_loc() - user.get_phy_loc(-1)
		other_user_dir = other_user.get_phy_loc() - other_user.get_phy_loc(-1)

		theta_user = get_angle_between(vector_to_other, user_dir)
		theta_other = get_angle_between(vector_from_other, other_user_dir)

		return np.clip(math.cos(theta_user) + math.cos(theta_other) / 2, 0, 1)


	# See Equation 6 from Bachmann et al. for details.
	def calculate_env_vectors(self, user, sum_distance):

		env_vectors = []

		for env_segment in self.env:

			start = env_segment[0]
			stop = env_segment[1]
			point = user.get_phy_loc()

			# (Jakob) I removed all the special cases here, numpy can do everything properly
			# d_i here used to be a length but it has to be a vector to the nearest point!
			# This should work for hor/vert/diag
			d_i = get_vector_from_segment(point, start, stop)
			w_i = (d_i / norm(d_i)) * (sum_distance / norm(d_i))

			env_vectors.append(w_i)
		return env_vectors



	# See Equation 7 from Bachmann et al. for details.
	def calculate_other_users_vector(self, user, users, sum_distance):
		user_vectors = []

		for other_user in users:
			if other_user.identity == user.identity:
				continue
			kappa = self.calculate_kappa(user, other_user)
			# Check Equation 4 from Bachmann et al.

			# (Jakob) Same cleanup as for the env vectors
			h_j = (user.get_phy_loc() - other_user.get_phy_loc())
			u_j = kappa * (h_j / norm(h_j)) * (sum_distance / (norm(h_j) * self.gamma))

			user_vectors.append(u_j)
		return user_vectors


	# The force vectors above indicate how much the user should move in a certain physical direction in order to optimally mitigate collisions
	# with other users and environmental obstacles. However, the goal is to make the steering movements imperceptible for the user in the virtual
	# world. Hence, this function calculates the maximum steering rate that can be applied for that purpose.
	def calculate_max_rotations(self, users):

		user_rates = []
		# The idea is to iterate through all users and calculate their maximum moving rates. See Equation 8 from Bachmann et al. for details.
		for user in users:
			user_rates.append(Rates())
			rates = user_rates[-1]

			if len(user.phy_locations) > 1:
				# (Jakob) This way previously _multiplied_ by the resolution but it should be divided!
				linear_velocity = norm(user.get_phy_loc() - user.get_phy_loc(-1)) / self.delta_t
			else:
				linear_velocity = user.speed

			rates.base_rate = self.base_rate * self.delta_t

			# (Jakob) This part was still missing I think
			# Note that the sign of the angle matters!
			if len(user.phy_locations) > 1:
				rates.head_rate_actual = get_angle_between(user.get_phy_loc(), user.get_phy_loc(-1)) / self.delta_t
			else:
				rates.head_rate_actual = 0


			if linear_velocity >= self.velocity_thresh:
				rates.moving_rate = linear_velocity / self.radius # Paper provides this in degrees
			else:
				rates.head_rotate_amplify = rates.head_rate_actual * self.ang_amplify_scale
				rates.head_rate_compress = rates.head_rate_actual * self.ang_compress_scale

		return user_rates

	# The point here is not to calculate next step, but to steer the user in the physical world as much as possible
	# (i.e., maintaining imperceptibility) in the direction suggested by the force vectors. Since the goal of the simulator is to
	# provide mapping between virtual and physical world, this steering can be modeled solely by the current and next locations of
	# the users.
	def calculate_next_physical_step(self, user, rates, force_vector):
		# (Jakob) The user is walking towards user_dr and this should be pushed towards desired_dir as much as possible
		user_dir = get_angle(user.get_phy_loc() - user.get_phy_loc(-1))
		desired_dir = get_angle(force_vector)

		user_dir %= 2 * math.pi
		desired_dir %= 2 * math.pi

		# (Jakob) negative value is rightward rotation!
		desired_rotation = desired_dir - user_dir
		# (Jakob) Smallest rotation is in [-pi,pi]
		desired_rotation = (desired_rotation + math.pi) % (2 * math.pi) - math.pi

		# Get best rate, Eq. 10
		rates.scale(norm(force_vector) * self.scale_multiplier / self.t_a_norm)
		#Eq. 11
		rates.moving_rate = min(rates.moving_rate, self.max_move_rate)
		#Eq. 12
		best_rate = rates.get_best_rate(desired_dir) * self.delta_t

		# (Jakob) This wasn't implemented properly I think. The max_move_rate is a ceiling to the calculated move_rate,
		# linear to the velocity. In the previous implementation, one being smaller than the other meant the user
		# was rotated instantly towards the force vector.

		if best_rate >= abs(desired_rotation):
			# (Jakob) Can immediately achieve desired rotation: walk along force vector
			step = force_vector

		else:
			# (Jakob) Can't rotate far enough right away, so close the gap as much as possible
			possible_rotation = user_dir - best_rate if desired_rotation < 0 else user_dir + best_rate

			# (Jakob) https://math.stackexchange.com/a/3534251 For step size 1, scales below
			step = np.empty(2)

			step[0] = math.cos(possible_rotation)
			step[1] = math.sin(possible_rotation)

		# Scale to precision
		# (Jakob) This now assumes a speed of 1m/s, more advanced approach todo
		step = self.delta_t * step / norm(step)

		return step


	# Implements the APF-R algorithm from Bachmann et al.
	def reset_if_needed(self, force_vector, env_vectors, user_vectors, threshold):

		# If too close to any obstacle, turn towards force vector
		vectors = []
		vectors.extend(env_vectors)
		vectors.extend(user_vectors)
		for vec in vectors:
			if norm(vec) > threshold:
				return self.delta_t * force_vector / norm(force_vector)  # Move towards force vector
		return None

def find_nearest_point_on_segment(point, s1, s2):
	# https://math.stackexchange.com/a/330329
	#The segment [s1,s2] is part of the line s1 + t(s2-s1), and each point in that line is on the segment iff t in [0,1]

	t_closest = np.dot(point - s1, s2 - s1) / (norm(s2-s1) ** 2)
	t_on_segment = np.clip(t_closest, 0, 1)
	return s1 + t_on_segment * (s2 -s1)

# Helpers
def get_vector_from_segment( point, s1, s2):
	return point - find_nearest_point_on_segment(point, s1, s2)

def get_angle_between(vec1, vec2):
	# (Jakob) Angle between two 2d vectors is arccos(a dot b / ||a||||b||) https://www.omnicalculator.com/math/angle-between-two-vectors
	try:
		return math.acos(np.dot(vec1, vec2) / (norm(vec1) * norm(vec2)))
	except:
		return 0

def get_angle(vec):
	return np.arctan2(vec[1], vec[0])

def deg(angle):
	return angle * 180 / math.pi

def rad(angle):
	return angle * math.pi / 180

def norm(vec):
	return np.linalg.norm(vec)