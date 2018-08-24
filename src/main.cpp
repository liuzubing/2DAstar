#include"ros/ros.h"
#include"astar/astar_planner.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"astar_node");
    AstarPlanner ap;
    //printf("mainmainmain");
    ap.plan();
    ros::spin();//goto the recall cycle function, until Ctrl + C is pressed.
    return 0;
}