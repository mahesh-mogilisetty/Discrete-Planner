#ifndef HEADER_H
#define HEADER_H

#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <time.h>
#include <math.h>
#include <fstream>
#include <string>

using namespace std;

namespace Planner
{
    ///represent a coordinate in the system with coordinates x and y of the grid
	class coordinate
	{
		public:	int x,y;
				coordinate() : x(0), y(0) {}
				///constructor which stores the pair of coordinates when an object of this class is created
				coordinate(int x1, int y1) : x(x1), y(y1) {}

	};

    ///stores a set of coordinates, mainly used for determining the next move/neighbors of a node
    using coordinate_list = std::vector <coordinate>;

    ///stores the information of a node, contains its own location (x and y coordinates)
    ///boolean 'obs' is used to determine if the current node is an obstacle or not
    class node : public coordinate
    {
        public:	bool obs = false;
                ///default constructor
                node() : coordinate() {}
                ///constructor which stores the information of its own location
                node(int x1, int y1) : coordinate(x1, y1) {}
    };

    ///stores a list of nodes
    using node_list = std::vector <node*>;

    /// a list of nodes visited in the last sqrt(max_step_number)
    using coordinate_deque = std::deque <node*>;

    ///function used for taking the input world_state and the grid is populated with the nodes as objects
    void make_grid(vector<vector<int> > &world_state, unsigned int &grid_size);

    ///function to check whether the coordinates x and y are inside the grid and free of obstacle
    bool check(int x, int y, unsigned int &grid_size);

    ///function to randomly find the next possible move
    ///it considers all moves in all four directions (orthogonal moves)
	  node* next_move(node*, unsigned int &grid_size, coordinate_deque &path);

    ///function to check whether the present node is in the last visited list which contains (sqrt(max_step_number)) nodes/steps
    bool last_visited(node*, coordinate_deque &path);

    ///to check if the goal has been reached from the source or any other node, it can also be used to find if two nodes have the same coordinates
    bool goal_reached(node* A, node* B);

    ///Function for finding a path to the goal by randomly moving in the environment using this random planner function
    void random_planner(coordinate &robot_pose, coordinate &goal_pose, unsigned int &grid_size,coordinate_list &final_path);
};

///The function search shall [] return a path between robot position and goal position in a predefined world.
///The predefined world will be of type world_state indicating size and position of obstacles.
Planner::coordinate_list search(vector<vector<int> > &world_state,int [],int[]);

///Set maximum number of steps, the planner has to find an acceptable solution in less than the max_step_number.
extern unsigned int max_step_number;

///the planner will not visit a cell that was visited in the last sqrt(max_step_number)
extern unsigned int sqrt_max_step_number;

#endif
