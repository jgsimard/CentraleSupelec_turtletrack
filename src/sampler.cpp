#include <ros/ros.h>
#include <string>
#include <fstream>
#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"
#include "tl_turtle_track/record.h"

bool pantilts_callback(const tl_turtle_track::PanTilts::ConstPtr &msg, std::vector<tl_turtle_track::PanTilt> &pt)
{
  pt = msg->PanTilt_Array;
}

bool record_callback(tl_turtle_track::record::Request &req, tl_turtle_track::record::Response &res, std::vector<tl_turtle_track::PanTilt> &pt, std::ofstream &output_file, float& x, float& y)
{
  output_file << x << ',' << y << ',' << pt.front().pan << ',' << pt.front().tilt << "\n";
}

int main(int argc, char * argv[]) {

  std::string path;
  std::vector<tl_turtle_track::PanTilt> pt;
  float x = 0.0, y = 0.0;
  
  if (argc > 1)
    {
      path = std::string(argv[1]);
      std::ifstream input_file;
      input_file.open (path + "/position.txt");

      std::ofstream output_file;
      output_file.open(path + "/record.txt");

      ros::init(argc, argv, "sampler");
      ros::NodeHandle nh;

      ros::Subscriber sub_track = nh.subscribe<tl_turtle_track::PanTilts>
	("pantilts_in",1,std::bind(pantilts_callback,
				   std::placeholders::_1,
				   std::ref(pt)
				   ));
 
      ros::ServiceServer service = nh.advertiseService<tl_turtle_track::record::Request, tl_turtle_track::record::Response>
	("record", std::bind(record_callback, std::placeholders::_1,
			     std::placeholders::_2,
			     std::ref(pt),
			     std::ref(output_file),
			     std::ref(x),
			     std::ref(y)
			     ));

      ros::Rate r(10);
      if(!input_file.eof()){
	ros::spinOnce();
	r.sleep();
      }
      input_file.close();
      output_file.close();
    }
 
}
