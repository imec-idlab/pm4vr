import numpy as np
import pylab 
import random 

class User:
	def __init__(self, initial_loc, initial_speed, identity):
		if type(initial_loc) == list:
			initial_loc = np.array(initial_loc)
		self.identity = identity
		self.initial_loc = initial_loc
		self.speed = initial_speed
		self.phy_locations = [initial_loc]
		self.virt_locations = []

	def get_phy_loc(self, offset=0):
		return self.phy_locations[-1 + offset]


	def get_virt_loc(self, offset=0):
		return self.virt_locations[-1 + offset]

	def fill_virtual_path(self, number_of_points, step, fixed_dir=None):
		print(step, "precision", number_of_points)
		self.virt_locations.append(self.initial_loc)

		# Filling the coordinates with random variables
		for i in range(1, number_of_points):

			val = random.randint(1, 4) if fixed_dir is None else fixed_dir # direction
			newloc = np.array(self.virt_locations[-1])
			if val == 1: #right
				newloc[0] += step
			elif val == 2: #left
				newloc[0] -= step

			elif val == 3: #up
				newloc[1] += step

			else:  #down
				newloc[1] -= step
			self.virt_locations.append(newloc)
