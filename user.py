#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Library for defining a user. Each user should be defined with its identity, initial location and 
speed, and virtual movement trajectory. 
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"


import numpy as np
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

	# The goal is to return a 1-dimensional list to be reshaped later on for short-term predictions
	def get_phy_path(self):

		dataset = {}
		dataset['phy_x'] = []
		dataset['phy_y'] = []

		for i in self.phy_locations:
			
			dataset['phy_x'].append(i[0])
			dataset['phy_y'].append(i[1])

		return dataset


