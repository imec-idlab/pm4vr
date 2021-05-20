#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Library for visualizing the physical and virtual movement trajectories. 
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"


import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
# Use for visualizing the virtual and physical paths of the users defined in the users variable.   

def visualize_paths(number_of_points, users):

	# Change the color-code if you require more than 8 users.
	color_codes = {1: "#f7fbff", 2: "#deebf7", 3: "#c6dbef", 4: "#9ecae1", 5: "#6baed6", 6: "#4292c6", 7: "#2171b5", 8: "#084594"}

	iter_temp = 8 
	flag_temp = 1

	for user in users:
					
		plt.figure(1) 
		plt.grid(True)
		plt.title("Virtual paths: random walk ($n = " + str(number_of_points) + "$ steps)")
		virt_path = np.array(user.virt_locations)
		plt.plot(virt_path[:,0], virt_path[:,1], '.', color=color_codes[iter_temp], label="User" + str(flag_temp))
		
		if flag_temp <= len(users):
			plt.plot(virt_path[0,0], virt_path[0,1], 'x', color="black")
		plt.legend()
		
		plt.figure(2)
		plt.grid(True)
		plt.title("Physical paths")
		phy_path = np.array(user.phy_locations)

		# Fade in the color  https://stackoverflow.com/a/61758419
		cmap = colors.LinearSegmentedColormap.from_list(
			'incr_alpha', [(0, (*colors.to_rgb(color_codes[iter_temp]), 0)), (1, color_codes[iter_temp])])
		plt.scatter(phy_path[:,0], phy_path[:,1], c=range(phy_path.shape[0]), cmap=cmap, label="User" + str(flag_temp), ec=None)

		if flag_temp <= len(users):
			plt.plot(phy_path[0,0], phy_path[0,1], 'x', color="black")
		plt.xlim(-11,11)
		plt.ylim(-11,11)
		plt.gca().set_aspect('equal', adjustable='box')
		plt.legend()

		iter_temp -= 1 
		flag_temp += 1
		
	plt.show() 