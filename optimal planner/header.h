#ifndef HEADER_H
#define HEADER_H

#include <iostream>
#include <vector>
#include <map>
#include <time.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>

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
    ///pointer 'previous' is used to store the node so that it can used to backtrack the original path
    ///f(n) = g(n) + h(n) , where f(n) = estimated total cost of path through n to goal
    ///where g(n) = cost so far to reach n, h(n) = estimated cost from n to goal
    /// cost of the path and an estimate of the cost required to extend the path all the way to the goal
    class node  : public coordinate
    {
        public:	int f,g,h;
                node* previous = nullptr;
                bool obs = false;
                std::vector <node*> neighbors;
                ///default constructor
                node() : coordinate(), f(0), g(0), h(0) {}
                node(int x1, int y1) : coordinate(x1, y1), f(0), g(0), h(0) {}
    };

    ///function used for taking the input world_state and the grid is populated with the nodes as objects
    void make_grid(vector<vector<int> > &world_state, unsigned int &grid_size);

    ///Calculate the  Manhattan Distance between two nodes
    int heuristic(node* a, node* b);

    ///find neighbors of a particular node
    void find_neighbors(node* current, unsigned int &grid_size);

    ///List for storing the vectors
    using node_list = std::vector <node*>;

    ///function to check whether the coordinates x and y are inside the grid and free of obstacle
    bool check(int x, int y, unsigned int &grid_size);

    ///to check if the goal has been reached from the source or any other node, it can also be used to find if two nodes have the same coordinates
	bool goal_reached(node* A, node* B);

    ///Optimal Planner to implement the A* search algorithm for finding the path from source to destination
    void optimal_method(coordinate &robot_pose, coordinate &goal_pose, unsigned int &grid_size,coordinate_list &final_path);

};

///The function search shall [] return a path between robot position and goal position in a predefined world.
///The predefined world will be of type world_state indicating size and position of obstacles.
Planner::coordinate_list search(vector<vector<int> > &world_state,int [],int[]);

#endif
