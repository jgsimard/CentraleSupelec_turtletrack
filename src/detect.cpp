#include <ros/ros.h>
#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"
#include "pantiltzoom.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

#define NB_COLORS 4

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

#define TOLERANCE 10

void callback()
{
  std::array<cv::Scalar,NB_COLORS> colors = {{cv::Scalar(255,0,0),
					      {0,255,0},
					      {0,0,255},
					      {255,255,0}}};
  cv::Mat hsv;
  cv::Mat detection; 
  cv::Mat cleaned; 
  cv::Mat frame;
  
  cv::Mat open_elem  = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
  cv::Mat close_elem = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,9));

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i>               hierarchy;

  PanTilts pantilts;

  cv::cvtColor(frame, hsv, CV_BGR2HSV);       // convert it to HSV

  for(int i = 0; i < NB_COLORS; ++i){

    cv::Scalar hsv_min(colors[i][0] - TOLERANCE - 1, colors[i][1] - TOLERANCE, 0);
    cv::Scalar hsv_max(colors[i][0] + TOLERANCE - 1, colors[i][1] + TOLERANCE, 255);

    cv::inRange(hsv,hsv_min,hsv_max,detection); // select in-range pixels

    cv::erode (detection,detection,open_elem);  // opening...
    cv::dilate(detection,detection,open_elem);  // ...
    cv::dilate(detection,detection,close_elem); // closing...
    cv::erode (detection,detection,close_elem); // ...

    cv::findContours(detection, 
		     contours, hierarchy, 
		     CV_RETR_TREE, 
		     CV_CHAIN_APPROX_SIMPLE, 
		     cv::Point(0, 0));


    /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  for( int i = 0; i < contours.size(); i++ )
    pantilts.push_back(pantiltlzoom());
  }
  
}

int main(int argc, char * argv[]) {
  
  ros::init(argc, argv, "axis_move");
  ros::NodeHandle n1, n2, n3;
  ros::Publisher pub_pose_bot = n1.advertise<tl_turtle_track::PanTilts>("/pose_bot", 1);

  double u, v, pan, tilt, zoom;

  auto callback = [&u, &v, &pan, &tilt, &zoom]
  
  ros::Subscriber sub_img     = n2.subscibe("img_cam",1000,
					    
  ros::Subscriber sub_pos_cam = n3.subscibe("pos_cam",1000; imgCallBack);

  /*
  ros::Rate poll_rate(100);
  while(pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  std::cout << "I got one connection" << std::endl;

  std::list<std::pair<double, double> > positions = { {0,0}, 
						      {180, 0},
						      {225, 0},
						      {135, 0},
						      {180, 0},
						      {180, -90},
						      {180, 10},
						      {180, 0}};

  change_zoom(pub, 0);
  
  for(auto& pt : positions) 
    change_pos(pub, pt.first, pt.second);
  
  change_zoom(pub, 100);
  change_zoom(pub, 1000);
  change_zoom(pub, 10000);
  */

}
