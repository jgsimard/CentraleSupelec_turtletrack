#include <ros/ros.h>
#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"
#include "pantiltzoom.hpp"
//#include "axis_camera/Axis.h" //"tl_turtle_track/Axis.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

#define NB_COLORS 2

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

#define TOLERANCE 5

void img_callback(const sensor_msgs::ImageConstPtr& msg,
		  ros::Publisher &pub,
		  double &pan, double &tilt, double &zoom)
{
  cv_bridge::CvImageConstPtr bridge_input;
  try {
    bridge_input = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
  }
  catch (cv::Exception& e) {
    std::ostringstream errstr;
    errstr << "cv_bridge exception caught: " << e.what();
    return;
  }
  
  const cv::Mat& input  = bridge_input->image;
  cv::Mat        rgb   (input.rows, input.cols, CV_8UC3);

  unsigned int size           = input.rows * input.cols * 3;
  unsigned char* begin_input  = (unsigned char*)(input.data);
  unsigned char* end_input    = (unsigned char*)(input.data) + size;
  unsigned char* out          = (unsigned char*)(rgb.data);
  unsigned char* in           = begin_input;

  // Efficient way to process each channel in each pixel,
  // while(in != end_input) *(out++) = 255 - *(in++);
  while(in != end_input) *(out++) = *(in++);

  //pub.publish(cv_bridge::CvImage(msg->header, "rgb8", output).toImageMsg());
  
   std::array<cv::Scalar,NB_COLORS> colors = {
     //cv::Scalar(10,0,0),
     cv::Scalar(165,0,0)
  };//ORANGE, ROSE
   
  cv::Mat hsv, tresh, detection, cleaned;

  
  
  cv::Mat open_elem  = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7));
  cv::Mat close_elem = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,9));

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i>               hierarchy;

  tl_turtle_track::PanTilts pantilts;
  std::vector<tl_turtle_track::PanTilt> vec_pantilt;

  cv::cvtColor(rgb, hsv, CV_BGR2HSV);       // convert it to HSV
  
  for(auto& color : colors){

    cv::Scalar hsv_min(color[0] - TOLERANCE, 50, 50);
    cv::Scalar hsv_max(color[0] + TOLERANCE, 255, 255);

    cv::inRange(hsv,hsv_min,hsv_max, tresh); // select in-range pixels

    //nettoyer img
    cv::erode (tresh,detection,open_elem);  // opening...
    cv::dilate(detection,detection,open_elem);  // ...
    cv::dilate(detection,detection,close_elem); // closing...
    cv::erode (detection,detection,close_elem); // ...

    cv::findContours(detection,contours, hierarchy, 
		     CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0, 0));
    
    std::vector<cv::Moments> mu(contours.size());
    std::vector<cv::Point2f> mc(contours.size());

    double v0 = input.rows/2, u0 = input.cols/2;

    for( int j = 0; j < contours.size(); j++ ){
      mu[j] = moments( contours[j], false );                             // Get the moments
      mc[j] = cv::Point2f( mu[j].m10/mu[j].m00 , mu[j].m01/mu[j].m00 );  // Get the mass centers
      auto res = pantiltzoom(mc[j].x, mc[j].y, u0, v0, pan, tilt, zoom); // &&corriger cette ligne&&
      tl_turtle_track::PanTilt pantilt;
      pantilt.pan  = res.first;
      pantilt.tilt = res.second;
      vec_pantilt.push_back(pantilt);
      circle(rgb, mc[j], 20, cv::Scalar(255,0,0));
    }
    cv::imshow( "hsv", hsv );  // Show our image inside it.
    cv::imshow( "rgb", rgb );  // Show our image inside it.
    cv::imshow( "detection", detection );  // Show our image inside it.
    cv::imshow( "tresh", tresh );  // Show our image inside it.
    cv::waitKey(1);
  }
  pantilts.PanTilt_Array = vec_pantilt;
  pub.publish(pantilts);
}
//::ConstPtr& msg
void pose_callback(const tl_turtle_track::Axis::ConstPtr &msg,
		   double &pan, double &tilt, double &zoom)
{
  
  pan  = msg->pan;
  tilt = msg->tilt;
  zoom = msg->zoom;
}

int main(int argc, char * argv[]) {
  
  ros::init(argc, argv, "axis_move");
  double pan, tilt, zoom;
  ros::NodeHandle nh;
  ros::Publisher pub_pose_bot = nh.advertise<tl_turtle_track::PanTilts>("/pose_bot", 1);

  ros::Subscriber sub_pos = nh.subscribe<tl_turtle_track::Axis>
    ("pose_in", 1, std::bind(pose_callback, std::placeholders::_1, std::ref(pan), std::ref(tilt),std::ref(zoom)));
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe
    ("image_in", 1,
     std::bind(img_callback,
	       std::placeholders::_1,
	       std::ref(pub_pose_bot),
	       std::ref(pan),
	       std::ref(tilt),
	       std::ref(zoom)));

  // ros::Rate loop_rate(((double) 1)/4);
  // int count = 0;
  // while (ros::ok()) {
  //   ROS_INFO("%i", count);
  //   count++;
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
   ros::spin();
}
