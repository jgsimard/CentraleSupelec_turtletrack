#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <iterator>

#include <boost/asio.hpp>

#include <ros/ros.h>
#include "tl_turtle_track/ArenaPosition.h"
#include "tl_turtle_track/ArenaPositions.h"

struct Point {
  double x,y;
};

int main(int argc,char* argv[]) {

  if(argc!=3) {
    std::cerr << "Usage : " << argv[0] 
	      << " <hostname> <port> " << std::endl;
    return 1;
  }
  
  ros::init(argc, argv, "share");
  ros::NodeHandle nh;
  unsigned int nb;
  std::vector<tl_turtle_track::ArenaPosition> aps_vec;
  tl_turtle_track::ArenaPosition ap;
  tl_turtle_track::ArenaPositions aps;
   
  try{
    // SOCKET CREATION
    std::string hostname = argv[1];
    std::string port     = argv[2];
    boost::asio::ip::tcp::iostream socket;
    socket.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
    socket.connect(hostname,port);

    ros::Publisher pub_pos_out = nh.advertise<tl_turtle_track::ArenaPositions>("/positions_server", 1);
    
    ros::Rate r(10);
    if(ros::ok){

      socket << "get" << std::endl;
      socket >> nb;
      while(nb != 0) {
	aps_vec.clear();
	auto out = std::back_inserter(aps_vec);
	double x,y;
	for(unsigned int i = 0; i < nb; ++i) {
	  socket >> ap.x >> ap.y;
	  *(out++) = ap;
	}
      }
      aps.ArenaPosition_Array = aps_vec;
      pub_pos_out.publish(aps);
      ros::spinOnce();
      r.sleep();
    }
  }			    
  catch(std::exception& e) {
    std::cerr << "Exception caught : " << e.what() << std::endl;
  }

  return 0;
}
