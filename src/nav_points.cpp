#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include "tl_turtle_track/pose.h"
#include <angles/angles.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "tl_turtle_track/go.h"
#include "tl_turtle_track/abort.h"

#define BASE_TF "turtle1" //"base_footprint"
#define WORLD_TF "world"  // "odom"

//#define BASE_TF "base_footprint"
//#define WORLD_TF "odom"

#define MAX_DIST 0.05
#define MAX_ANGL 0.05

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

bool moveTo(const geometry_msgs::Pose2D& target_pose_world, geometry_msgs::Twist& cmd_vel)
{

  geometry_msgs::Pose2D target_pose_base;
  transformPose2D(target_pose_world, WORLD_TF, target_pose_base, BASE_TF);

  double distance = std::sqrt(std::pow(target_pose_base.x,2) + std::pow(target_pose_base.y,2));
  double angle = std::atan2(target_pose_base.y,target_pose_base.x);
  
  if(distance > MAX_DIST) {

    if (std::abs(angle) > MAX_ANGL) {
      // ROS_INFO("1 ; dist = %f ; angle = %f ; y = %f", cmd_vel.linear.x, angle, target_pose_base.y);
      //ROS_INFO("1");
      cmd_vel.linear.x = 0.1*cmd_vel.linear.x;
      cmd_vel.angular.z = 1*angle;
    }
    
    else { //go to destination
      // ROS_INFO("2 ; dist = %f ; x = %f", distance, target_pose_base.x);
      //ROS_INFO("2");
      cmd_vel.linear.x = 0.4*target_pose_base.x;
      cmd_vel.angular.z = 0.0;
    }
  }
  
  else if (std::abs(target_pose_base.theta) > MAX_ANGL){
    // ROS_INFO("3 ; dist = %f ; theta = %f", distance, target_pose_base.theta);
    //ROS_INFO("3");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 1*target_pose_base.theta;
  }

  else{
    //ROS_INFO("4");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }
  return false;
  
}
  

bool go_callback(tl_turtle_track::go::Request &req, tl_turtle_track::go::Response &res, bool &to_process) {
  ROS_INFO("GO TURTLE!!!");
  to_process = true;
}

bool abort_callback(tl_turtle_track::abort::Request &req, tl_turtle_track::abort::Response &res,  bool &to_process) {
  ROS_INFO("ABORT TURTLE!!!");
  to_process = false;
}

int main(int argc, char* argv[]) {

  bool to_process = false;
  ros::init(argc, argv, "NAV_POSE");

  // auto go_callback = [&to_process](tl_turtle_track::go::Request &req, tl_turtle_track::go::Response &res) {to_process = true; res.ret = true;};
  // auto abort_callback = [&to_process](tl_turtle_track::abort::Request &req, tl_turtle_track::abort::Response &res) {to_process = false; res.cool = true;};

  ros::NodeHandle node;
  
  ros::Publisher pub1 = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher pub2 = node.advertise<geometry_msgs::PoseArray>("poses", 1000);
  ros::Publisher pub3 = node.advertise<geometry_msgs::Pose>("curr_pose", 1000);

  ros::ServiceServer go = node.advertiseService<tl_turtle_track::go::Request, tl_turtle_track::go::Response>
    ("go", std::bind(go_callback,
		     std::placeholders::_1,
		     std::placeholders::_2,
		     std::ref(to_process)));
  
  ros::ServiceServer abort = node.advertiseService<tl_turtle_track::abort::Request, tl_turtle_track::abort::Response>
    ("abort", std::bind(abort_callback,
			std::placeholders::_1,
			std::placeholders::_2,
			std::ref(to_process)));

  ROS_INFO("Ready to transform poses from %s into %s", WORLD_TF, BASE_TF);
  ROS_INFO("ARGC  %i", argc); 
  ROS_INFO("command  %s", argv[0]);
  ROS_INFO("file  %s", argv[1]);
  
  if (argc > 0) {
    std::vector<geometry_msgs::Pose> poses;
    std::string x, y, theta;

    geometry_msgs::PoseArray pose_array;  
    geometry_msgs::Twist cmd_vel;

    std::string line;
    std::ifstream file (argv[1]);

    if(file.is_open()) {
      
      ROS_INFO("file  %s", argv[1]);
    
      for (int i = 0; std::getline(file, line); i++) {
	ROS_INFO("line  %s", line.c_str());
	
	std::istringstream ss(line);
	std::getline(ss, x, ',');
	std::getline(ss, y, ',');
	std::getline(ss, theta, ',');
	
	//ROS_INFO("x = %s ; y = %s ; theta = %s", x.c_str(), y.c_str(), theta.c_str());
      
	geometry_msgs::Pose target_pose;
    
	target_pose.position.x = atof(x.c_str());
	target_pose.position.y = atof(y.c_str());
	target_pose.position.z = 0;
	target_pose.orientation = tf::createQuaternionMsgFromYaw(atof(theta.c_str()));
	
	poses.push_back(target_pose);
      }

      pose_array.poses = poses;
      
      // while (pub2.getNumSubscribers() < 1); // wait for all subscribed nodes to be "connected"
      
      pub2.publish(pose_array);

      ROS_INFO("Waiting to start");
      while(!to_process){
	ros::spinOnce();
      }
      ROS_INFO("Start");
      for(geometry_msgs::Pose & target_pose : pose_array.poses) {
	geometry_msgs::Pose2D target_pose_world;
	
	target_pose_world.x = target_pose.position.x;
	target_pose_world.y = target_pose.position.y;  
	target_pose_world.theta = angles::normalize_angle(tf::getYaw(target_pose.orientation));
	
	ROS_INFO("x = %f ; y = %f ; theta = %f", target_pose_world.x, target_pose_world.y, target_pose_world.theta);

        //while (pub3.getNumSubscribers() < 1);
	pub3.publish(target_pose);
	
	while(!moveTo(target_pose_world,cmd_vel)) {
	  //ROS_INFO("while");
	  pub1.publish(cmd_vel);
	  ros::spinOnce();

	  if(!to_process)
	    break;
	}
    
	pub1.publish(cmd_vel);
	
	ros::spinOnce();
	if(!to_process)
	  break;
      }
    }
  }  
  return 0;
}
