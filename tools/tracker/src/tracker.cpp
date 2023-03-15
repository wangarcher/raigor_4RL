// a tracker msgs receive and save project
// initializd in 2202217, by archer

#include <iostream>
#include <math.h>
#include <ros/ros.h>



#include "tracker.h"


Tracker::Tracker()
{
    sub_wrist_real_global_pose_ = n_.subscribe("/wrist_pose_ground_truth", 1, &Tracker::wrist_odom_callback, this);
    sub_base_real_global_pose_ = n_.subscribe("/base_pose_ground_truth", 1, &Tracker::base_odom_callback, this);

}

// void Tracker::ee_callback(const geometry_msgs::PoseStamped& ee_real_global_pose)
// {
//     ee_real_global_pose_ << ee_real_global_pose.pose.position.x,
//                             ee_real_global_pose.pose.position.y,
//                             ee_real_global_pose.pose.position.z,
//                             ee_real_global_pose.pose.orientation.x,
//                             ee_real_global_pose.pose.orientation.y,
//                             ee_real_global_pose.pose.orientation.z,
//                             ee_real_global_pose.pose.orientation.w;
// }

void Tracker::wrist_odom_callback(const nav_msgs::Odometry& wrist_odom)
{
    wrist_real_global_pose_ << wrist_odom.pose.pose.position.x,
                               wrist_odom.pose.pose.position.y,
                               wrist_odom.pose.pose.position.z,
                               wrist_odom.pose.pose.orientation.x,
                               wrist_odom.pose.pose.orientation.y,
                               wrist_odom.pose.pose.orientation.z,
                               wrist_odom.pose.pose.orientation.w;
    wrist_real_global_twist_ << wrist_odom.twist.twist.linear.x,
                                wrist_odom.twist.twist.linear.y,
                                wrist_odom.twist.twist.linear.z,
                                wrist_odom.twist.twist.angular.x,
                                wrist_odom.twist.twist.angular.y,
                                wrist_odom.twist.twist.angular.z;

}

void Tracker::base_odom_callback(const nav_msgs::Odometry& base_odom)
{
    base_real_global_pose_ << base_odom.pose.pose.position.x,
                              base_odom.pose.pose.position.y,
                              base_odom.pose.pose.position.z,
                              base_odom.pose.pose.orientation.x,
                              base_odom.pose.pose.orientation.y,
                              base_odom.pose.pose.orientation.z,
                              base_odom.pose.pose.orientation.w;
    base_real_global_twist_ << base_odom.twist.twist.linear.x,
                               base_odom.twist.twist.linear.y,
                               base_odom.twist.twist.linear.z,
                               base_odom.twist.twist.angular.x,
                               base_odom.twist.twist.angular.y,
                               base_odom.twist.twist.angular.z;
}

void Tracker::run()
{
    struct timespec time = {0, 0}; // time initialization
    std::ofstream something_great_in;
    something_great_in.open("v.txt", std::ios::trunc);
    float  wrist_real_global_pose_x;
    std::cout << "now instilling data to the file" << std::endl;
    while (n_.ok())
    {
        if(wrist_real_global_pose_(0) != wrist_real_global_pose_x )
        {
            clock_gettime(CLOCK_REALTIME, &time);
            // something_great_in << time.tv_sec << "\t" << time.tv_nsec << "\t" << wrist_real_global_twist_.transpose() 
            //                                                           << "\t" << base_real_global_twist_.transpose() << "\n";
            something_great_in << time.tv_sec << "\t" << time.tv_nsec << "\t" << wrist_real_global_pose_.transpose() 
                                                                      << "\t" << base_real_global_pose_.transpose() << "\n";
            wrist_real_global_pose_x = wrist_real_global_pose_(0);

        }
        ros::Duration(0.05).sleep();

    }
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "tracker");
    Tracker tracker;
    ros::AsyncSpinner spinner(4);
	spinner.start();

    // ros::WallDuration(1.0).sleep();
    tracker.run();

    ros::waitForShutdown();

    return 0;
}