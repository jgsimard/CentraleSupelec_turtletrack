#include <ros/ros.h>
#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"
#include "pantiltzoom.hpp"
//#include "tl_turtle_track/track_mode.h"
//#include "axis_camera/Axis.h" //"tl_turtle_track/Axis.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <string>
#include <limits>
#include <utility>

#define MODE_SCAN   "SCAN"
#define MODE_SEARCH "SEARCH"
#define MODE_TRACK  "TRACK"


void change_pos(ros::Publisher& pub, float hpos, float vpos) {
  tl_turtle_track::Axis pos;
  pos.pan = hpos;
  pos.tilt = vpos;
  pos.zoom = 0;
  pos.focus = 0;
  pos.iris = 0 ;
  pos.brightness = 0;
  pos.autofocus = true;

  pub.publish(pos);

  ros::spinOnce();
  ros::Duration(4).sleep();
}
 
void change_zoom(ros::Publisher& pub, float zoom) {
  std::cout << "Setting zoom to " << zoom << std::endl;
  tl_turtle_track::Axis pos;
  pos.pan = 0;
  pos.tilt = 0;
  pos.zoom = zoom;
  pos.focus = 0;
  pos.iris = 0 ;
  pos.brightness = 0;
  pos.autofocus = true;

  pub.publish(pos);

  ros::spinOnce();
  ros::Duration(4).sleep();
}

void scan(tl_turtle_track::Axis pose_local, std::vector<std::pair<double,double>> &states, int &state, ros::Publisher &pub) {
    pose_local.pan  = states[state].first;
    pose_local.tilt = states[state].second;
    pub.publish(pose_local);
    state = (state + 1)%5;
}

void track (const tl_turtle_track::PanTilts::ConstPtr &msg,
	    ros::Publisher &pub,
	    tl_turtle_track::Axis &pose_local)
{
  double dist_min = std::numeric_limits<double>::max(), pan = pose_local.pan, tilt=pose_local.tilt, dist;
    
    for (auto& pantilt : msg->PanTilt_Array){
      dist = std::sqrt(std::pow(pose_local.pan - pantilt.pan,2) + std::pow(pose_local.tilt - pantilt.tilt,2));
      
      if (dist < dist_min){
	dist_min = dist;
        pan  = pantilt.pan;
	tilt = pantilt.tilt;
      }
    }

    pose_local.tilt = tilt;
    pose_local.pan  = pan;
    pub.publish(pose_local);
}

void pantilts_callback(const tl_turtle_track::PanTilts::ConstPtr &msg,
		       std::string &mode, ros::Publisher &pub, tl_turtle_track::Axis &pose_in,
		       std::vector<std::pair<double,double>> &states, int &state)
{
  tl_turtle_track::Axis pose_local = pose_in;
  if (mode == MODE_SCAN){
    scan(pose_local, states, state, pub);
  }
  else if( mode == MODE_SEARCH){
    if(msg->PanTilt_Array.empty()) 
      scan(pose_local, states, state, pub);
    else 
      track(msg, pub, pose_local);
  }
  else{ //MODE_TRACK by default
    if (!msg->PanTilt_Array.empty()) 
      track(msg, pub, pose_local);
  }

}

void pose_callback(const tl_turtle_track::Axis::ConstPtr &msg, tl_turtle_track::Axis &pose)
{
  pose = *msg;
}
      

int main(int argc, char * argv[]) {
  
  std::string mode = MODE_SEARCH;
  tl_turtle_track::Axis pose;
  std::vector<std::pair<double,double>> states = {
    std::make_pair(0.0,-20.0),
    std::make_pair(50.0,-20.0),
    std::make_pair(25.0,-35.0),
    std::make_pair(0.0,-50.0),
    std::make_pair(50.0,-50.0)
  };
  
  int state = 2;
  
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh;
  ros::Publisher pub_track = nh.advertise<tl_turtle_track::Axis>("/cmd", 1);

  ros::Subscriber sub_pose = nh.subscribe<tl_turtle_track::Axis>
    ("pose_in",1,std::bind(pose_callback, std::placeholders::_1,
			      std::ref(pose)));
    
  ros::Subscriber sub_track = nh.subscribe<tl_turtle_track::PanTilts>
     ("pantilts_in",1,std::bind(pantilts_callback, std::placeholders::_1,
				std::ref(mode),
				std::ref(pub_track),
				std::ref(pose),
				std::ref(states),
				std::ref(state)
				));
 
  ros::Rate loop_rate(((double) 1)/4);
  int count = 0;
  while (ros::ok()) {
    ROS_INFO("%i", count);
    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  /*
  double pan, tilt, zoom;
  ros::NodeHandle nh;
  ros::Publisher pub_pose_bot = nh.advertise<tl_turtle_track::PanTilts>("/pose_bot", 1);

  ros::Subscriber sub_pos = nh.subscribe<tl_turtle_track::Axis>
    ("pose_in", 1000, std::bind(pose_callback, std::placeholders::_1, std::ref(pan), std::ref(tilt),std::ref(zoom)));
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe
    ("image_in", 1, std::bind(img_callback, std::placeholders::_1, std::ref(pub_pose_bot), std::ref(pan), std::ref(tilt),std::ref(zoom)));
  */
  //ros::spin();
}  

