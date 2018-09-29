# Discrete-Planner
discrete motion planners for a holonomic robot

	1. Coding files (C++)
	a) random_planner	directory
		header.h	Defined all the header files, classes, functions used in the program
		Planner.cpp	Includes the main algorithm implemented for the random discrete planner
		main.cpp	Defined all inputs and outputs and called the search function
		Makefile	make file to compile and create an executable file
	
	b) optimal_planner	directory
		header.h	Defined all the header files, classes, functions used in the program
		Planner.cpp	Includes the main algorithm implemented for the optimal discrete planner
		main.cpp	Defined all inputs and outputs and called the search function
		Makefile	make file to compile and create an executable file

	output.txt 		Output file generated

	2. How to compile and run the code on Linux using g++

	Using  Makefile by going inside the follwing directory on linux terminal
	a) random_planner 	-----directory
	$ make
	$ ./random		-----output.txt is generated
	
	b) optimal_planner	-----directory
	$ make
	$ ./optimal		-----output.txt is generated

	You can also use the following way
	$ g++ -std=c++11 header.h Planner.cpp main.cpp
	$ ./a.out
	After the a.out is executed, you can find the path in "output.txt"

	3. Default value of inputs inside the main.cpp file
	robot_pose = {2,0}
	goal_pose  = {5,5}
	world_state = {{0, 0, 1, 0, 0, 0},
                       {0, 0, 1, 0, 0, 0},
                       {0, 0, 0, 0, 1, 0},
                       {0, 0, 0, 0, 1, 0},
                       {0, 0, 1, 1, 1, 0},
                       {0, 0, 0, 0, 0, 0}};

	max_step_number = 25 (this input is only for the random planner)
	
	you can change these values for different test cases
	here the planners are designed for 2D square world_state
