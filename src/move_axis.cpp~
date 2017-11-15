#include <ros/ros.h>
#include <axis_camera/Axis.h>

void change_pos(ros::Publisher& pub, float hpos, float vpos) {
  axis_camera::Axis pos;
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
  axis_camera::Axis pos;
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

int main(int argc, char * argv[]) {
  
  ros::init(argc, argv, "axis_move");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<axis_camera::Axis>("/cmd", 1);

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

}
