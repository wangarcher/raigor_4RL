// work 
#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define PI 3.1415926535897932384626433
// param 
static std::string odom_topic = "/omron_ros_wheel/odom";
static std::string odom_output_topic = "";
static std::string parent_frame = "/odom";
static std::string child_frame = "/base_link";
static std::string lidar_frame = "";
static std::string path_topic = "/path_gt";
static std::string path_dist_topic = "";
static std::string path_points_topic = "/path_points_gt";
static bool path_enable = false;
static float path_dist_threshold = 0.1;          // Unit:m, 每隔path_dist_threshold发布一个point
static float path_angle_threshold = 5;          // Unit:agree, 每隔path_angle_threshold发布一个point

// path 
ros::Publisher  pub_path, pub_path_dist, pub_path_points, pub_odom_output;
nav_msgs::Path path;
sensor_msgs::PointCloud2 path_points;
pcl::PointCloud<pcl::PointXYZI>::Ptr path_points_pcl(new pcl::PointCloud<pcl::PointXYZI>());

Eigen::Isometry3d nav_msgsPose2Eigen(const geometry_msgs::Pose* _pose){
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond result_r(_pose->orientation.w,_pose->orientation.x,
                                _pose->orientation.y,_pose->orientation.z);
  Eigen::Vector3d result_t(_pose->position.x,_pose->position.y,_pose->position.z);
  result.rotate(result_r);
  result.pretranslate(result_t);
  return result;
}

void odom_callback(const nav_msgs::OdometryConstPtr& odom){
  static uint count = 1;
  static double traveling_dist = 0;
  static std_msgs::Float32MultiArray traveling_dist_msg;
  traveling_dist_msg.data.resize(2);
  // 发布tf
  static tf::TransformBroadcaster br;
  static tf::TransformListener tfListener;
  tf::Transform tf;
  geometry_msgs::Pose odom_pose = odom->pose.pose;

//  tf.setOrigin(tf::Vector3(odom_pose.position.x, odom_pose.position.y, odom_pose.position.z));
//  tf::Quaternion quat;
//  tf::quaternionMsgToTF(odom_pose.orientation, quat);
//  tf.setRotation(quat);
  tf::poseMsgToTF(odom_pose, tf);

  if( !lidar_frame.empty() ) {
    tf::StampedTransform Base2Lidarlink;
    try
    {
      // 等待3s
      tfListener.waitForTransform(lidar_frame, child_frame, ros::Time(0), ros::Duration(3.0));
      // baselink系到lidar系的变换
      tfListener.lookupTransform(lidar_frame, child_frame, ros::Time(0), Base2Lidarlink);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      tf::Transform tf;
      geometry_msgs::Pose pose_tmp;
      pose_tmp.position.x = 0;
      pose_tmp.position.y = 0;
      pose_tmp.position.z = 0;
      pose_tmp.orientation.w = 1;
      tf::poseMsgToTF(pose_tmp, tf);  
      Base2Lidarlink = tf::StampedTransform(tf, ros::Time(0), child_frame, lidar_frame); // 单位化
    }
    tf = Base2Lidarlink.inverse() * tf * Base2Lidarlink;
  }

  tf::StampedTransform stamped_tf(tf, odom->header.stamp, parent_frame, child_frame);

  if( !odom_output_topic.empty() ) {
    nav_msgs::Odometry odom_output_msg;
    odom_output_msg.header = odom->header;
    odom_output_msg.header.frame_id = parent_frame;
    odom_output_msg.child_frame_id = child_frame;
    tf::poseTFToMsg(tf, odom_output_msg.pose.pose);
    pub_odom_output.publish(odom_output_msg);
  }

  br.sendTransform(stamped_tf);
  if(++count % 100 == 0){ // 保证path的发布频率
    pub_path.publish(path);
    pcl::toROSMsg(*path_points_pcl, path_points);
    path_points.header = path.header;
    pub_path_points.publish( path_points );
    if( pub_path_dist.getNumSubscribers() > 0 ) {
      traveling_dist_msg.data[0] = ros::Time::now().toSec();
      traveling_dist_msg.data[1] = traveling_dist;
      pub_path_dist.publish(traveling_dist_msg);
    }

    if( path.poses.size() % 1000 == 0 && path.poses.size() != 0){
      ROS_WARN("SIZE in the path:%d", path.poses.size() );  // 实验测试size大于16000时，rviz会崩
    }
    count = 1;
  }


  // update path
  static bool is_first = true;
  static geometry_msgs::PoseStamped pre_pose;
  if(is_first) {
    pre_pose.header = odom->header;
    pre_pose.pose = odom->pose.pose;
    path.header.frame_id = parent_frame;
    path.header.stamp = odom->header.stamp;
    path.poses.push_back(pre_pose);
    pcl::PointXYZI tmp_way_point;
    tmp_way_point.x = pre_pose.pose.position.x;
    tmp_way_point.y = pre_pose.pose.position.y;
    tmp_way_point.z = pre_pose.pose.position.z;
    tmp_way_point.intensity = path.poses.size();
    path_points_pcl->push_back(tmp_way_point);
    is_first = false;
    return;
  }
  Eigen::Isometry3d pre_pose_eigen = nav_msgsPose2Eigen(&pre_pose.pose);
  Eigen::Isometry3d curr_pose_eigen = nav_msgsPose2Eigen(&odom->pose.pose);
  Eigen::Isometry3d delta_pose_eigin = curr_pose_eigen.inverse() * pre_pose_eigen;
  Eigen::Vector3d delta_euler = delta_pose_eigin.rotation().eulerAngles(2,1,0); 
  Eigen::Vector3d delta_trans = delta_pose_eigin.translation();
  // std::cout << delta_pose_eigin.matrix() << std::endl;
  // std::cout << delta_euler << std::endl;
  // std::cout << delta_trans << std::endl;


  if( std::min( abs( delta_euler[0] ), PI - abs( delta_euler[0] ) ) < path_angle_threshold && 
      std::min( abs( delta_euler[1] ), PI - abs( delta_euler[1] ) )  < path_angle_threshold && 
      std::min( abs( delta_euler[2] ), PI - abs( delta_euler[2] ) )  < path_angle_threshold && 
     delta_trans.norm() < path_dist_threshold )
  {
    return; 
  }
  else {
    traveling_dist += delta_trans.norm();
    path.header.stamp = odom->header.stamp;
    path.poses.push_back(pre_pose);
    pcl::PointXYZI tmp_way_point;
    tmp_way_point.x = pre_pose.pose.position.x;
    tmp_way_point.y = pre_pose.pose.position.y;
    tmp_way_point.z = pre_pose.pose.position.z;
    tmp_way_point.intensity = path.poses.size();
    path_points_pcl->push_back(tmp_way_point);
    pre_pose.pose = odom->pose.pose;
    pre_pose.header = odom->header;
  }
}

bool clear_path_handler(std_srvs::Empty::Request &rqt,
                        std_srvs::Empty::Response &res)
{
  ROS_INFO("The size of current Path is %d.", path.poses.size());
  ROS_INFO(" Path clear finished!");
  path.poses.clear();
  path_points_pcl->clear();
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "odom2tf");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("odom_topic", odom_topic);
  private_nh.getParam("odom_output_topic", odom_output_topic);
  private_nh.getParam("parent_frame", parent_frame);
  private_nh.getParam("child_frame", child_frame);
  private_nh.getParam("lidar_frame", lidar_frame);
  private_nh.getParam("path_topic", path_topic);
  private_nh.getParam("path_dist_topic", path_dist_topic);
  private_nh.getParam("path_points_topic", path_points_topic);
  private_nh.getParam("path_enable", path_enable);
  private_nh.getParam("path_dist_threshold", path_dist_threshold);
  private_nh.getParam("path_angle_threshold", path_angle_threshold);

  std::cout << "Odom topic: " << odom_topic << std::endl;
  if( !odom_output_topic.empty() ) {
    std::cout << "Odom output topic: " << odom_output_topic << std::endl;
  }
  std::cout << "Parent frame: " << parent_frame << std::endl;
  std::cout << "Child frame: " << child_frame << std::endl;
  if( !lidar_frame.empty() ) {
    std::cout << "Lidar frame: " << lidar_frame << std::endl;
  }
  std::cout << "Path topic: " << path_topic << std::endl;
  std::cout << "Path topic: " << path_dist_topic << std::endl;
  std::cout << "Path points topic: " << path_points_topic << std::endl;
  std::cout << "path_enable: " << path_enable << std::endl;
  std::cout << "path_dist_threshold: " << path_dist_threshold << std::endl;
  std::cout << "path_angle_threshold: " << path_angle_threshold << std::endl;

  pub_path = nh.advertise<nav_msgs::Path>(path_topic, 10,true);
  pub_path_dist = nh.advertise<std_msgs::Float32MultiArray>(path_dist_topic, 1,true);
  pub_path_points = nh.advertise<sensor_msgs::PointCloud2>(path_points_topic, 10,true);
  pub_odom_output = nh.advertise<nav_msgs::Odometry>(odom_output_topic, 10);
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odom_callback);
  ros::ServiceServer clear_path_service = nh.advertiseService("clear_path", clear_path_handler);
  ros::spin();

  return 0;
}