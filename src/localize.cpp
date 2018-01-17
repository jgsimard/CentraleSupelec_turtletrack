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
		       gaml::libsvm::Predictor<XY,Z> &svm_pan,
		       gaml::libsvm::Predictor<XY,Z> &svm_tilt,
		       ros::Publisher &pub)
{
  tl_turtle_track::ArenaPosition ap;
  XY xy = std::make_pair(msg.pan, msg.tilt);
  ap.x = svm_pan(xy);
  ap.y = svm_tilt(xy);
  pub.publish(ar);
}

int main(int argc, char * argv[]) {

  std::string path_svm_pan, path_svm_tilt;  
  if (argc > 1)
    {
      path_svm_pan  = std::string(argv[1]) + "/svm_pan.pred";
      path_svm_tilt = std::string(argv[2]) + "/svm_tilt.pred";

      gaml::libsvm::Predictor<XY,Z> svm_pan(nb_nodes_of, fill_nodes);
      svm_pan.load_model(path_svm_pan);

      gaml::libsvm::Predictor<XY,Z> svm_tilt(nb_nodes_of, fill_nodes);
      svm_tilt.load_model(path_svm_tilt);

      ros::init(argc, argv, "localize");
      ros::NodeHandle nh;

      ros::Publisher pub_loc = nh.advertise<tl_turtle_track::ArenaPosition>("/localize_out", 1);
      ros::Subscriber sub_pantilts = nh.subscribe<tl_turtle_track::PanTilts>
	("pantilts_in",1,std::bind(pantilts_callback,
				   std::placeholders::_1,
				   std::ref(svm_pan),
				   std::ref(svm_tilt),
				   std::ref(pub_loc)
				   ));      

      ros::spin();
      }
    }
 
}
