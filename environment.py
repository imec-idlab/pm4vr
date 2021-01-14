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