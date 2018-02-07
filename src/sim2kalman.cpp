#include <ros/ros.h>

#include <cmath>
#include <vector>

#include <turtlesim/Pose.h>

#include "tl_turtle_track/Entities.h"
#include "tl_turtle_track/Entity.h"
#include "tl_turtle_track/State.h"

void sim_callback(const turtlesim::Pose::ConstPtr&              msg,
		  ros::Publisher&                               pub,
		  ros::Publisher&                               pub_tru
		  )
{
  tl_turtle_track::Entity entity;

  entity.nb = 0;
  entity.pos.x = msg->x;
  entity.pos.y = msg->y;

  std::vector<tl_turtle_track::Entity> entities_vec;

  entities_vec.push_back(entity);

  tl_turtle_track::State state;

  state.entity = entity;
  state.angle = msg->theta;
  
  tl_turtle_track::Entities entities_out;

  entities_out.array = entities_vec;
  
  pub.publish(entities_out);
  pub_tru.publish(state);
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "sim2kalman");  

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<tl_turtle_track::Entities>("entities", 1);
  ros::Publisher pub_tru = nh.advertise<tl_turtle_track::State>("state_tru", 1);
  
  ros::Subscriber sub_entities = nh.subscribe<turtlesim::Pose>
    ("pose",1,std::bind(sim_callback,
			std::placeholders::_1,
			std::ref(pub),
			std::ref(pub_tru)
			));
  
  ros::spin();

  return 0;
}
