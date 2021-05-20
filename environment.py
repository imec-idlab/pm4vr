#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Library for defining the shape and size of an environment. Each environment has to be a bounded shape 
and is defined by the end points of the shape.
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"


import numpy as np

# Creates a square-like environment of a given size, with the center of the environment being at (0,0)
def define_square(size):

	p1 = [np.array([ size / 2,  size / 2]), np.array([ size / 2, -size / 2])]
	p2 = [np.array([ size / 2, -size / 2]), np.array([-size / 2, -size / 2])]
	p3 = [np.array([-size / 2, -size / 2]), np.array([-size / 2,  size / 2])]
	p4 = [np.array([-size / 2,  size / 2]), np.array([ size / 2,  size / 2])]

	# Environment is defined with a set of straight walls, where each wall is defined by its end-points
	env = [p1, p2, p3, p4]

	return env



# Creates a square-like environment of a given sizes, with the center of the environment being at (0,0)
def define_rectangle(x_size, y_size):

	p1 = [np.array([ x_size / 2,  y_size / 2]), np.array([ x_size / 2, -y_size / 2])]
	p2 = [np.array([ x_size / 2, -y_size / 2]), np.array([-x_size / 2, -y_size / 2])]
	p3 = [np.array([-x_size / 2, -y_size / 2]), np.array([-x_size / 2,  y_size / 2])]
	p4 = [np.array([-x_size / 2,  y_size / 2]), np.array([ x_size / 2,  y_size / 2])]

	# Environment is defined with a set of straight walls, where each wall is defined by its end-points
	env = [p1, p2, p3, p4]

	return env


# The ones below are used for relatively specialized problems discussed in [].

# Creates a relatively unstructured environment  
def define_weirdness_1(x_size, y_size):

	p1 = [np.array([ x_size / 2,  y_size / 2]), np.array([ x_size / 2, -y_size / 2])]
	p2 = [np.array([ x_size / 2, -y_size / 2]), np.array([-x_size / 2, -y_size / 2])]
	p3 = [np.array([-x_size / 2, -y_size / 2]), np.array([-x_size / 2,  y_size / 2])]
	p4 = [np.array([-x_size / 2,  y_size / 2]), np.array([ x_size / 2,  y_size / 2])]

	# Environment is defined with a set of straight walls, where each wall is defined by its end-points
	env = [p1, p2, p3, p4]

	return env


# Creates a relatively unstructured environment  
def define_weirdness_2(x_size, y_size):

	p1 = [np.array([ x_size / 2,  y_size / 2]), np.array([ x_size / 2, -y_size / 2])]
	p2 = [np.array([ x_size / 2, -y_size / 2]), np.array([-x_size / 2, -y_size / 2])]
	p3 = [np.array([-x_size / 2, -y_size / 2]), np.array([-x_size / 2,  y_size / 2])]
	p4 = [np.array([-x_size / 2,  y_size / 2]), np.array([ x_size / 2,  y_size / 2])]

	# Environment is defined with a set of straight walls, where each wall is defined by its end-points
	env = [p1, p2, p3, p4]

	return env


# Creates a relatively unstructured environment  
def define_weirdness_3(x_size, y_size):

	p1 = [np.array([ x_size / 2,  y_size / 2]), np.array([ x_size / 2, -y_size / 2])]
	p2 = [np.array([ x_size / 2, -y_size / 2]), np.array([-x_size / 2, -y_size / 2])]
	p3 = [np.array([-x_size / 2, -y_size / 2]), np.array([-x_size / 2,  y_size / 2])]
	p4 = [np.array([-x_size / 2,  y_size / 2]), np.array([ x_size / 2,  y_size / 2])]

	# Environment is defined with a set of straight walls, where each wall is defined by its end-points
	env = [p1, p2, p3, p4]

	return env