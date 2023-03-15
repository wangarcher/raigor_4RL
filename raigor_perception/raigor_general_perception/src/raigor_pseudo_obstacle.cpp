#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raigor_pseudo_obstacle");
 
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<geometry_msgs::PoseArray>("/move_base/TebLocalPlannerROS/ee_obstacles", 1);

    geometry_msgs::PoseArray ee_obstacles;
    ee_obstacles.header.frame_id = "map"; 
    geometry_msgs::Pose ee_point_obstacle;
    for (int i = 0; i < 6; i++)
    {
        ee_point_obstacle.position.x = 3.75;
        ee_point_obstacle.position.y = 3 + 0.5 * i;
        ee_point_obstacle.position.z = 1.0;
        ee_obstacles.poses.push_back(ee_point_obstacle);
    }
    for (int j = 0; j < 11; j++)
    {
        ee_point_obstacle.position.x = 3.75 + 0.5 * j;
        ee_point_obstacle.position.y = 5.5;
        ee_point_obstacle.position.z = 1.0;
        ee_obstacles.poses.push_back(ee_point_obstacle);
    }
    ros::Rate rate(50.0);
    while (node.ok())
    {

        pub.publish(ee_obstacles);
        rate.sleep();
    }
    return 0;
};