// a pick ang place demo, initialized in 211124

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>


#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>


typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

class Tracker
{
public:
    Tracker();
    void ee_callback(const geometry_msgs::PoseStamped& ee_real_global_pose);
    void wrist_odom_callback(const nav_msgs::Odometry& wrist_odom);
    void base_odom_callback(const nav_msgs::Odometry& base_odom);

    void run();

public:
    ros::NodeHandle n_; 
    ros::Subscriber sub_ee_real_global_pose_;
    ros::Subscriber sub_wrist_real_global_pose_;
    ros::Subscriber sub_base_real_global_pose_;

    // tf::StampedTransform ee_transform;
    // tf::StampedTransform map_arm_transform;
    
    Vector7f ee_real_global_pose_;
    Vector7f wrist_real_global_pose_;
    Vector7f base_real_global_pose_;

    Vector6f wrist_real_global_twist_;
    Vector6f base_real_global_twist_;
};
