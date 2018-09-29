#include "header.h"

int main(int argc, char* argv[])
{

    ///INPUTS given by the user
    ///Robot Position and Goal Position
    ///robot_pose is a 2 element int array consisting the two indices (x, y) which represent the current pose of the robot in world_state .
    ///goal_pose is a is a 2 element int array consisting the two indices (x, y) which represent the goal in world_state coordinate system.
    int robot_pose[2] = {2,0};											//-----------USER CAN CHANGE VALUE OF ROBOT POSITION HERE
    int goal_pose[2]  = {5,5};											//-----------USER CAN CHANGE VALUE OF GOAL POSITION HERE

    ///world_state is a 2D-grid representation of the environment where the value 0 indicates a navigable
    ///space and the value 1 indicates an occupied/obstacle space.
    vector<vector<int> > world_state = {{0, 0, 1, 0, 0, 0},				//-----------USER CAN CHANGE THE 2D WORLD STATE HERE
                                        {0, 0, 1, 0, 0, 0},
                                        {0, 0, 0, 0, 1, 0},
                                        {0, 0, 0, 0, 1, 0},
                                        {0, 0, 1, 1, 1, 0},
                                        {0, 0, 0, 0, 0, 0}};

    ///call for search function
    Planner::coordinate_list path = search(world_state,robot_pose,goal_pose);


    ///The path from Robot to Goal is saved in the output.txt file
    ///There will no be no path if the Robot is not able to reach the Goal
    std::ofstream file;													//-----------USER CAN COMMENT THIS PART IF NOT REQUIRED till line no 48
    std::string file_output;
    file_output.assign("output.txt");
    file.open(file_output.c_str());

    if(path.size()==0)
    {
    	file<<"With Optimal Planner, Robot failed to reach the goal";
    }
    else
    {
    	file<<"goal reached in "<<path.size()-1<<" step"<<endl;
    	file<<"[";
    	for(auto it : path)
        {
            Planner::coordinate point = it;
            file<<" ("<<point.x<<", "<<point.y<<")"<<",";
        }
        file<<"]";
    }
    file.close();														//-----------USER CAN COMMENT THIS PART IF NOT REQUIRED

    return 0;
}
