import matplotlib.pyplot as plt

# Use for visualizing the virtual and physical paths of the users defined in the users variable.   
def visualize_paths(number_of_points, users):

	# Change the colorcode if you require more than 8 users.
	color_codes = {1: "#f7fbff", 2: "#deebf7", 3: "#c6dbef", 4: "#9ecae1", 5: "#6baed6", 6: "#4292c6", 7: "#2171b5", 8: "#084594"}

	iter_temp = 8 
	flag_temp = 1

	for user in users:
					
		plt.figure(1) 
		plt.grid(True)
		plt.title("Virtual paths: random walk ($n = " + str(number_of_points) + "$ steps)") 
		plt.plot(user["virt_path_x"], user["virt_path_y"], '.', color=color_codes[iter_temp], label="User" + str(flag_temp)) 
		
		if flag_temp <= len(users):
			plt.plot(user["virt_path_x"][0], user["virt_path_y"][0], 'x', color="black") 
		plt.legend()
		
		plt.figure(2) 
		plt.grid(True)
		plt.title("Physical paths") 
		plt.plot(user["phy_path_x"], user["phy_path_y"], '.', color=color_codes[iter_temp], label="User" + str(flag_temp))
		
		if flag_temp <= len(users):
			plt.plot(user["phy_path_x"][0], user["phy_path_y"][0], 'x', color="black") 
		plt.legend()

		iter_temp -= 1 
		flag_temp += 1
		

	plt.show() 