# Discrete-Planner
discrete motion planners for a holonomic robot
Default value of inputs inside the main.cpp file
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
