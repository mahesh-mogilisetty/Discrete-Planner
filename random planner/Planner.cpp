/************  Motion Planning: Random Discrete planner  ************

Inputs:
world_state is a 2D-grid representation of the environment where the value 0 indicates a navigable
space and the value 1 indicates an occupied/obstacle space.
robot_pose is a 2 element int array consisting the two indices (x, y) which represent the current pose of the robot in
world_state .
goal_pose is a is a 2 element int array consisting the two indices (x, y) which represent the goal in world_state coordinate system.


Output:
final_path is a list of coordinates x and y representing a path from the robot_pose to the goal_pose in
world_state or None if no path has been found.

Algorithm used for the Optimal Planner is A* search algorithm

*/

#include "header.h"

/// 2D array of objects for class node is declared, used to store the information of the grid
Planner::node **Node;

///Node - a 2D array of node objects are declared and memory is allocated accordingly
///iterates over the world_state array and sets the obs (obstacle) of each node
/// when value of an element in world_state is 0, Node[i][j].obs = false
/// when value of an element in world_state is 1, Node[i][j].obs = true
void Planner::make_grid(vector<vector<int> > &world_state, unsigned int &grid_size)
{

    Node = new node*[grid_size];
    for (unsigned int i = 0; i < grid_size; ++i)
        Node[i] = new  node[grid_size];

    for (unsigned int i = 0; i < grid_size; ++i)
        for (unsigned int j = 0; j < grid_size; ++j)
        {
            Node[i][j] = node(i,j);
            if(world_state[i][j]==1)
                Node[i][j].obs = true;
        }
}

///Check if we reached our goal or destination, takes in two node pointers,
/// determines if the both nodes match or not
bool Planner::goal_reached(node* A, node* B)
{
    if((A->x==B->x)&&(A->y==B->y))
        return true;
    else
        return false;

}

///Checks whether the x and y coordinates are inside the world_state or the grid
///also returns a value 'false' if there is any obstacle at (x,y)
bool Planner::check(int x, int y, unsigned int &grid_size)
{
    if( (unsigned(x)>(grid_size-1)) || (unsigned(y)>(grid_size-1)) || (x<0) || (y<0) || ((Node[x][y]).obs) )
        return false;
    return true;
}

///checks whether the node was visited in last sqrt(max_step_number) steps
bool Planner::last_visited(node* temp, coordinate_deque &path)
{
    unsigned int i = 0;
    for (auto it = path.rbegin(); it != path.rend() && i< sqrt_max_step_number; ++it,i++)
    {
        node* current = *it;
        if( (temp->x == current->x) && (temp->y == current->y) )
                return true;

    }
    return false;
}

/// Finds all the possible moves from the current node and returns a single move
/// There are possibly four orthogonal moves if the location of this is in the middle of the grid
/// if the current node has x and y coordinates then the possible moves are
/// (x-1,y) (x+1,y) (x,y-1) (x,y+1)
/// a set of coordinates  { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } to find the possible moves as stated above

Planner::node* Planner::next_move(node* current, unsigned int &grid_size, coordinate_deque &path)
{
    node_list a;
    coordinate_list moves = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
            int i = current->x;
            int j = current->y;
            for(unsigned k = 0; k <moves.size(); ++k)
            {
                int x1 = i+moves[k].x;
                int y1 = j+moves[k].y;
                if(check(x1,y1,grid_size))
                {
                    a.push_back(&Node[x1][y1]);
                }
            }
    /// all the possible moves(nodes) are stored using the node_list 'a'
    /// another node_list 'b' is used to create a copy of node_list 'a'
    /// all the moves/nodes are removed from the node_list 'b'
    /// if they were last visited by using the memory of the random planner
    /// Accordingly, a random move/node is picked from the elements left in the node_list 'b'
    /// if all the nodes present in node_list 'b' were visited previously,
    /// then  a random move/node is picked from all the elements present in node_list 'a'
    if(a.size()>1)
    {
        node_list b = a;
        node_list::iterator it_b = b.begin();

        for(;it_b!=b.end();)
        {
            if(last_visited(*it_b,path))
            {
                it_b = b.erase(it_b);
            }
            else
                ++it_b;
        }
        if(b.size()>0)
        {
            it_b = b.begin();
            int k_b = rand()% b.size();
            std::advance( it_b, k_b );
            current = *it_b;
            return current;
        }
    }


    node_list::iterator it = a.begin();


    int k = rand()% a.size();
    std::advance( it, k );
    current = *it;
    return current;
}

///Function for finding a path to the goal by randomly moving in the environment using this random planner function
void Planner::random_planner(coordinate &robot_pose, coordinate &goal_pose, unsigned int &grid_size, coordinate_list &final_path)
{
    /// list to save the final path consisting coordinates of the path
    coordinate_deque path;

    node *Start = &Node[robot_pose.x][robot_pose.y];
    node *Goal = &Node[goal_pose.x][goal_pose.y];
    path.push_back(Start);
    unsigned int i;
    node* temp = Start;
	///Start of the random planner algorithm
    for(i=0;!(goal_reached(temp,Goal)) && i < max_step_number;i++)
    {
        ///find the next set of node in the path
		temp = Planner::next_move(temp,grid_size,path);
		///here the random planner has a short memory i.e. sqrt_max_step_number
        if(path.size()>sqrt_max_step_number)
        {
            node* current = path[0];
            final_path.push_back(coordinate(current->x,current->y));
            path.pop_front();
            path.push_back(temp);
        }
        else
        {
            path.push_back(temp);
        }

    }
	///to check if the robot has reached the goal or not
    if(goal_reached(temp,Goal))
    {

        for(auto it : path)
        {
            node* current = it;
            final_path.push_back(coordinate(current->x,current->y));
        }
     }
	 ///The robot failed to reach within max_step_number
     else
     {
         final_path.clear();         
     }
}

///The function search shall [] return a path between robot position and goal position using the random planner
///in a predefined world. The predefined world will be of type world_state indicating the position of obstacles.
Planner::coordinate_list search(vector<vector<int> > &world_state,int robot_pos[],int goal_pos[])
{
    ///variables to store the coordinates of the robot position and the goal position
    Planner::coordinate robot_pose,goal_pose;
    unsigned int grid_size;
	 ///find the size of world_state (grid)
    grid_size = world_state[0].size();
	///a list to save the final path taken by the planner
    Planner::coordinate_list final_path;
    robot_pose = Planner::coordinate(robot_pos[0],robot_pos[1]);
    goal_pose = Planner::coordinate(goal_pos[0],goal_pos[1]);
	
	///Check if the robot_pose and goal_pose are inside the scope of world_state grid
    ///Also check if the robot_pose and goal_pose are free from obstacle
    int x = robot_pose.x;
    int y = robot_pose.y;
    if((unsigned(x)>(grid_size-1))||(unsigned(y)>(grid_size-1))||(x<0)||(y<0))
    {
        cout<<"robot position out of scope of the world state environment";
        return final_path;
    }
    if(world_state[x][y]==1)
    {
        cout<<"robot position is invalid as there is already an obstacle on the same position";
        return final_path;
    }
    x = goal_pose.x;
    y = goal_pose.y;
    if((unsigned(x)>(grid_size-1))||(unsigned(y)>(grid_size-1))||(x<0)||(y<0))
    {
        cout<<"goal position out of scope of the world state environment";
        return final_path;
    }
    if(world_state[x][y]==1)
    {
        cout<<"goal position is invalid as there is already an obstacle on the same position";
        return final_path;
    }
	///to check if the robot position and goal position are the same
    if(robot_pose.x == goal_pose.x && robot_pose.y == goal_pose.y)
    {
        cout<<"The robot is already at the goal position";
        final_path.push_back(robot_pose);
        return final_path;
    }
	
	///function call to make the grid as per the world_state environment
    Planner::make_grid(world_state, grid_size);
	///function call to find the shortest path using the optimal planner method
    Planner::random_planner(robot_pose,goal_pose,grid_size,final_path);
	
	///Delete memory allocated for the objects of class node
	for (unsigned int i = 0; i < grid_size; ++i)
        delete[] Node[i];
    delete[] Node;
    return final_path;
}



