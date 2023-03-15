// example for ee_link 2 base_link tf info, adopt this code to your own program
// first edition in 210813
// offer this code to Shixing, for further mm system integration
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <cmath>
using namespace std;
double x,y,z,ww,zz,hh,ii,Aww,Azz,Ahh,Aii;
double theta;
int main(int argc, char** argv){
    ros::init(argc, argv, "raigor_ee_tf");
 
    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (node.ok())
    {
        tf::StampedTransform transform;
        try
        {
            // changing both tip link frames helps you to get different tf infos
            listener.waitForTransform("base_link", "ee_link", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("base_link", "ee_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        // calculation of the pose
        x=transform.getOrigin().x();
        y=transform.getOrigin().y();
        z=transform.getOrigin().z();
        ww=transform.getRotation()[0];
        zz=transform.getRotation()[1];
        hh=transform.getRotation()[2];
        ii=transform.getRotation()[3];
        cout<<"x: "<<x<<endl;
        cout<<"y: "<<y<<endl;
        cout<<"z: "<<z<<endl;
        cout<<"ww: "<<ww<<endl;
        cout<<"zz: "<<zz<<endl;
        cout<<"hh: "<<hh<<endl;
        cout<<"ii: "<<ii<<endl;
        cout<<endl;
 
        rate.sleep();
    }
    return 0;
};