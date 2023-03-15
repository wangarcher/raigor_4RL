#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "sensor_msgs/JointState.h"

using namespace std;

ofstream f1, f2;

double duration=0; //time duration

double start_time = 0;


void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // interval time
  duration = ros::Time::now().toSec() - start_time;

  f1 << duration << "\t" << msg->position[0] << "\t"<< msg->position[1] << "\t"<< msg->position[2] << "\t"<< msg->position[3] << "\t"<< msg->position[4] << "\t"<< msg->position[5] << "\n" ;  //saving joint positions to file
  f2 << duration << "\t" << msg->velocity[0] << "\t"<< msg->velocity[1] << "\t"<< msg->velocity[2] << "\t"<< msg->velocity[3] << "\t"<< msg->velocity[4] << "\t"<< msg->velocity[5] << "\n" ;  //saving joint velocities to file


  ROS_INFO("I heard: [%f] ",duration);
}
 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  
  // open the file in the file system

  f1.open("/home/archer/work/catkin_ws/src/mm_perceive/measurement/joint_position.txt", std::ios_base::trunc);
  f2.open("/home/archer/work/catkin_ws/src/mm_perceive/measurement/joint_velocity.txt", std::ios_base::trunc);
 
  ros::NodeHandle n;
  
  // start time for the interval time

  start_time = ros::Time::now().toSec();

  // publish frequency

  ros::Subscriber sub = n.subscribe("/joint_states", 1000,  jointstatesCallback);

  ros::spin();
 
  return 0;
}