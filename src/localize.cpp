#include <ros/ros.h>
#include <string>
#include <utility> 
#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"
#include "tl_turtle_track/ArenaPosition.h"
#include "tl_turtle_track/ArenaPositions.h"

#include <gaml-libsvm.hpp>

//cochonerie
typedef std::pair<double,double> XY;
typedef double                   Z;
typedef std::pair<XY,Z> Data;

// x y are stored with 3 nodes...
int nb_nodes_of(const XY& xy) {
  return 3;
}
// ... as follows.
void fill_nodes(const XY& xy,struct svm_node* nodes) {
  nodes[0].index = 1;
  nodes[0].value = xy.first;  // x 
  nodes[1].index = 2;
  nodes[1].value = xy.second; // y
  nodes[2].index = -1;        // end
}
const XY& input_of (const Data& data) {return data.first;}
double    output_of(const Data& data) {return data.second;} 

bool pantilts_callback(const tl_turtle_track::PanTilts::ConstPtr &msg,
		       gaml::libsvm::Predictor<XY,Z> &svm_x,
		       gaml::libsvm::Predictor<XY,Z> &svm_y,
		       ros::Publisher &pub)
{
  if (msg->PanTilt_Array.size() != 0) {
    tl_turtle_track::ArenaPosition ap;
    XY xy(msg->PanTilt_Array[0].pan, msg->PanTilt_Array[0].tilt);
    ap.x = svm_x(xy);
    ap.y = svm_y(xy);
       
    pub.publish(ap);
  }
  return true;
}

int main(int argc, char * argv[]) {
  
  ros::init(argc, argv, "localize");
  
  ros::NodeHandle nh;

  ROS_INFO("CHECK -1");
		       
  std::string path_svm_x, path_svm_y;

  ROS_INFO("CHECK 0");

  try {
  
  if (argc > 1)
    {
      path_svm_x = std::string(argv[1]) + "/svm_x.pred";
      path_svm_y = std::string(argv[1]) + "/svm_y.pred";

      ROS_INFO("CHECK 1");
      
      gaml::libsvm::Predictor<XY,Z> svm_x(nb_nodes_of, fill_nodes);
      svm_x.load_model(path_svm_x);
      
      ROS_INFO("CHECK 2");

      gaml::libsvm::Predictor<XY,Z> svm_y(nb_nodes_of, fill_nodes);
      svm_y.load_model(path_svm_y);

      XY xy(33,-22);
      double x = svm_y(xy);
      
      ROS_INFO("%f", x);

      ROS_INFO("CHECK 3");

      ros::Publisher pub_loc = nh.advertise<tl_turtle_track::ArenaPosition>("/localize_out", 1);
      ros::Subscriber sub_pantilts = nh.subscribe<tl_turtle_track::PanTilts>
	("pantilts_in",1,std::bind(pantilts_callback,
				   std::placeholders::_1,
				   std::ref(svm_x),
				   std::ref(svm_y),
				   std::ref(pub_loc)
				   ));
      
      ROS_INFO("CHECK 4");

      ros::spin();

      ROS_INFO("CHECK 5");
    }
  } catch(ros::Exception & e) {
    ROS_INFO("%s", e.what());
  }

  ROS_INFO("CHECK 6");

  return 0;
}
