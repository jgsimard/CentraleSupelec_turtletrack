#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include "tl_turtle_track/pose.h"
#include <angles/angles.h>
#include <cmath>
#include <geometry_msgs/Twist.h>

#define BASE_TF "turtle1"
#define WORLD_TF "world"

bool transformPose2D(const geometry_msgs::Pose2D& pose_src,
		     std::string source_frame,
		     geometry_msgs::Pose2D& pose_dst,
		     std::string target_frame) {
  
  ros::Time now = ros::Time(0);

  // Build up a geometry_msgs::PoseStamped from a geometry_msgs::Pose2D
  geometry_msgs::PoseStamped pose_src_stamped;
  pose_src_stamped.header.seq = 0;
  pose_src_stamped.header.stamp = now;
  pose_src_stamped.header.frame_id = source_frame;
  pose_src_stamped.pose.position.x = pose_src.x;
  pose_src_stamped.pose.position.y = pose_src.y;
  pose_src_stamped.pose.position.z = 0;
  pose_src_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose_src.theta);

  tf::Stamped<tf::Pose> tf_pose_src, tf_pose_dst;
  
  // Build up a tf::Stamped<Pose> from a geometry_msgs::PoseStamped
  tf::poseStampedMsgToTF(pose_src_stamped, tf_pose_src);
  
  tf::TransformListener tf_listener;
  try {
    // Let us wait for the frame transformation to be avaiable
    tf_listener.waitForTransform(source_frame, target_frame, now,  ros::Duration(1.0));
    
    // And compute the transformation
    tf_listener.transformPose(target_frame, tf_pose_src, tf_pose_dst);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  double useless_pitch, useless_roll, yaw;
  tf_pose_dst.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
  // move the yaw in [-pi, pi]
  yaw = angles::normalize_angle(yaw);


  // Transform the tf::Pose back into the Pose2D
  pose_dst.x = tf_pose_dst.getOrigin().x();
  pose_dst.y = tf_pose_dst.getOrigin().y();
  pose_dst.theta = yaw;

  return true;
}


bool transform_callback(tl_turtle_track::pose::Request& request, tl_turtle_track::pose::Response& response) {

  geometry_msgs::Pose2D goal = request.pose;
  geometry_msgs::Pose2D target_pose;

  // Let us transform the current goal from the world to the 
  // base frame
  transformPose2D(goal, WORLD_TF, response.pose, BASE_TF);

  return true;
}

#define MAX_DIST 0.1
#define MAX_ANGL 0.1

bool moveTo(const geometry_msgs::Pose2D& target_pose, geometry_msgs::Twist& cmd_vel)
{
  if(std::abs(target_pose.x) > MAX_DIST){ // align with destination
    cmd_vel.linear.x = 0.0;
    cmd_vel.angluar.z = target_pose.x;
  }
  
  else if (target_pose.y > MAX_DIST){ //go to destination
    cmd_vel.linear.x = target_pose.y;
    cmd_vel.angluar.z = 0.0;
  }
  
  else if (std::abs(target.theta) > MAX_ANGL){
      cmd_vel.linear.x = 0.0;
      cmd_vel.angluar.z = target_pose.theta;
    }
  else{
     cmd_vel.linear.x = 0.0;
     cmd_vel.angluar.z = 0.0;
     return true;
  }

  return false;
}
int main(int argc, char* argv[]) {

  ros::init(argc, argv, "transform_pose2D");  
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("transform_pose2D", transform_callback);

  ros::NodeHandle node1
  ros::Publisher chatter_pub = node1.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ROS_INFO("Ready to transform poses from %s into %s", WORLD_TF, BASE_TF);
  
  ros::spin();

  return 0;
}
