/************  Motion Planning: Optimal Discrete planner  ************

The optimal planner tries to find the shortest (non-colliding) path to the goal with orthogonal moves.

Inputs:
world_state is a 2D-grid representation of the environment where the value 0 indicates a navigable
space and the value 1 indicates an occupied/obstacle space.
robot_pose is a 2 element int array consisting the two indices (x, y) which represent the current pose of the robot in world_state .
goal_pose is a is a 2 element int array consisting the two indices (x, y) which represent the goal in world_state coordinate system.

Output:
final_path is a list of coordinates x and y representing a path from the robot_pose to the goal_pose in
world_state or None if no path has been found.

A* is a search algorithm which is the process of finding a path between multiple points called nodes

Reference - "https://en.wikipedia.org/wiki/A*_search_algorithm"

*/


#include "header.h"

Planner::node **Node;

///Node - a 2D array of node objects are declared and memory is allocated accordingly
///iterates over the world_state array and sets the obs (obstacle) of each node
/// when value of an element in world_state is 0, Node[i][j].obs = false
/// when value of an element in world_state is 1, Node[i][j].obs = true
void Planner::make_grid(vector<vector<int> > &world_state, unsigned int &grid_size)
{
    Node = new node*[grid_size];
    for (unsigned int i = 0; i < grid_size; ++i)
        Node[i] = new node[grid_size];

    for (unsigned int i = 0; i < grid_size; ++i)
        for (unsigned int j = 0; j < grid_size; ++j)
        {
            Node[i][j] = node(i,j);
            if(world_state[i][j]==1)
                Node[i][j].obs = true;
        }
}

/// determines if the coordinates of two nodes match or not
///used mainly to check whether the robot has reached the goal or not,
bool Planner::goal_reached(node* A, node* B)
{
    if((A->x==B->x)&&(A->y==B->y))
        return true;
    else
        return false;

}

///find neighbors of a particular node and store them in neighbors vector of the node
void Planner::find_neighbors(node* current, unsigned int &grid_size)
{

    Planner::coordinate_list moves = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
            int i = current->x;
            int j = current->y;
            for(unsigned k = 0; k <moves.size(); ++k)
            {
                int x1 = i+moves[k].x;
                int y1 = j+moves[k].y;
                if(check(x1,y1,grid_size))
                {
                    current->neighbors.push_back(&Node[x1][y1]);
                }
            }
}

///Calculate the  Manhattan Distance between two nodes 'a' and 'b'
///the sum of absolute values of differences in the x and y coordinates
/// of nodes 'a' and 'b' respectively
int Planner::heuristic(node* a, node* b)
{
    return abs(a->x - b->x) + abs(a->y - b->y);
}

///Checks whether the x and y coordinates are inside the world_state or the grid
///also returns a value 'false' if there is any obstacle at (x,y)
bool Planner::check(int x, int y, unsigned int &grid_size)
{
    if((unsigned(x)>(grid_size-1))||(unsigned(y)>(grid_size-1))||(x<0)||(y<0))
        return false;
    if((Node[x][y]).obs)
        return false;
    return true;
}

/// A* search algorithm selects the path that minimizes
/// f(n) = g(n) + h(n), where f(n) = estimated total cost of path through n to goal
/// n is the last node on the path, g(n) is the cost of the path from the start node to n
/// h(n) is a heuristic function that estimates the cost of the cheapest path from n to the goal
void Planner::optimal_method(coordinate &robot_pose, coordinate &goal_pose, unsigned int &grid_size,coordinate_list &final_path)
{
    node *Start = &Node[robot_pose.x][robot_pose.y];
    node *Goal = &Node[goal_pose.x][goal_pose.y];

    ///open_set - The set of newly discovered nodes
    ///closed_set - The set of nodes already evaluated
    Planner::node_list open_set, closed_set, path;

    ///Initially only Start node is known
    open_set.push_back(Start);

    while(open_set.size()>0)
    {
            int best = 0;
            ///find the node by which the robot can most efficiently be
            ///reached from, having the lowest value of 'f'
            for(unsigned i=0; i<open_set.size();i++)
            {
                if(open_set[i]->f < open_set[best]->f)
                    best = i;
            }
            node *current = open_set[best];
            /// Check if the goal has been reached or not
            if(goal_reached(current,Goal))
            {
                //cout<<"goal reached"<<endl;
                node *temp = current;
                while(temp->previous)
                {
                    path.push_back(temp);
                    temp = temp->previous;
                }
                path.push_back(temp);

                for (int i = path.size()-1; i >= 0 ; i--)
                {
                    final_path.push_back(coordinate(path[i]->x,path[i]->y));
                }
                return;
            }

            ///remove current from open_set
            open_set.erase(std::remove(open_set.begin(), open_set.end(), current), open_set.end());

            closed_set.push_back(current);
            ///find neighbors of current node if not evaluated
            if(current->neighbors.size()==0)
            {
                ///finding neighbors
                find_neighbors(current, grid_size);
            }

            for(auto it: current->neighbors)
            {
                node* neighbor = it;
                /// Ignore the neighbor which is already evaluated.
                if(!(std::find(closed_set.begin(), closed_set.end(), neighbor) != closed_set.end()))
                {
                    /// The distance from current node to a neighbor
                    int gTemp = current->g + 1;
                    bool flag = false;
                    /// If it is in the open list or not, check to see if this path is better,
                    /// using 'f' cost as the measure.
                    if((std::find(open_set.begin(), open_set.end(), neighbor) != open_set.end()))
                    {
                        if(gTemp<(neighbor->g))
                        {
                            neighbor->g = gTemp;
                            flag = true;
                        }
                    }
                    else
                    {
                            neighbor->g = gTemp;
                            open_set.push_back(neighbor);
                            flag = true;
                    }
                    if(flag)
                    {
                        ///This path is the best until now
                        neighbor->h = heuristic(neighbor,Goal);
                        neighbor->f = neighbor->g + neighbor->h;
                        neighbor->previous = current;
                    }
                }
            }
    }
    ///No solution - Could not find a path from source position to destination position
    final_path.clear();

}

///The function search shall return a path between robot position and goal position
///using A * algorithm in a predefined world. The predefined world will be of type
///world_state indicating the position of obstacles.
Planner::coordinate_list search(vector<vector<int> > &world_state,int robot_pos[],int goal_pos[])
{
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
    Planner::optimal_method(robot_pose,goal_pose,grid_size,final_path);

    ///Delete memory allocated for the objects of class node
    for (unsigned int i = 0; i < grid_size; ++i)
        delete[] Node[i];
    delete[] Node;
    return final_path;
}
