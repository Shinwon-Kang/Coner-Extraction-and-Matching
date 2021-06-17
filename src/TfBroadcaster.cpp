#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>


float position_x, position_y, position_z;
float orientation_x, orientation_y, orientation_z, orientation_w;

// init pose
void initposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  // msg->pose->pose->position.(x, y, z)
  // msg->pose->pose->orientation.(x, y, z, w)
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  
  orientation_x = msg->pose.pose.orientation.x;
  orientation_y = msg->pose.pose.orientation.y;
  orientation_z = msg->pose.pose.orientation.z;
  orientation_w = msg->pose.pose.orientation.w;
}

// estimation pose
void estimationposeCallback(const geometry_msgs::Pose::ConstPtr& msg){
  ROS_INFO("@@@@@@@@@@ Estimation Pose Update @@@@@@@@@@");
  position_x = msg->position.x;
  position_y = msg->position.y;
  position_z = msg->position.z;

  orientation_x = msg->orientation.x;
  orientation_y = msg->orientation.y;
  orientation_z = msg->orientation.z;
  orientation_w = msg->orientation.w;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "TfBroadcaster");
    
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/initialpose", 10, &initposeCallback);
  ros::Subscriber sub2 = node.subscribe("/odom_test", 10, &estimationposeCallback);
  
  position_x = 0.0;
  position_y = 0.0;
  position_z = 0.0;
  
  orientation_x = 0.0;
  orientation_y = 0.0;
  orientation_z = 0.0;
  orientation_w = 1.0;
  
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  while(ros::ok()) {
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/map";
    transformStamped.child_frame_id = "/odom_test";

    transformStamped.transform.translation.x = position_x;
    transformStamped.transform.translation.y = position_y;
    transformStamped.transform.translation.z = position_z;
    
    transformStamped.transform.rotation.x = orientation_x;
    transformStamped.transform.rotation.y = orientation_y;
    transformStamped.transform.rotation.z = orientation_z;
    transformStamped.transform.rotation.w = orientation_w;

    br.sendTransform(transformStamped);  
    ros::spinOnce();
  }
  
  return 0;
}
