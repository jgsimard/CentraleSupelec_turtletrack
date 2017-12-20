#include <ros/ros.h>
#inculde <string>
#include <fsteam>
#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"

bool pantilts_callback(const tl_turtle_track::PanTilts::ConstPtr &msg, std::vector<PanTilt> &pt){
  pt = msg->PanTilt_Array;
}

bool record_callback(tl_turtle_track::record::Request &req, tl_turtle_track::record::Response &res, std::vector<PanTilt> &pt, std::ofstream &myfile) {
  myfile << pt;
}

int main(int argc, char * argv[]) {

  std::string path;
  std::vector<PanTilt> pt;
  
  if (argc > 1)
    {
      path = std::string arg1(argv[1]);

      ofstream myfile;
      myfile.open ( ~/path/"record.txt");
   

      ros::init(argc, argv, "sampler");
      ros::NodeHandle nh;

      ros::Subscriber sub_track = nh.subscribe<tl_turtle_track::PanTilts>
	("pantilts_in",1,std::bind(pantilts_callback, std::placeholders::_1,
				   std::ref(pt)
				   ));
 
      ros::ServiceServer service = nh.advertiseService<tl_turtle_track::record::Request, tl_turtle_track::record::Response>
	("record", std::bind(record_callback, std::placeholders::_1,
			     std::placeholders::_2,
			     std::ref(pt),
			     std::ref(myfile)
			     ));

      ros::spin();
      myfile.close();
 
    }
 
}
