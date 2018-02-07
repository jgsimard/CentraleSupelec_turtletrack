#include <ros/ros.h>

#include <string>
#include <vector>
#include <utility>
#include <fstream>
#include <Eigen/Dense>
#include <tuple>
#include <unordered_map>
#include <iterator>

#include <ctime>
#include <chrono>


#include "KalmanFilter.hpp"

#include <geometry_msgs/Twist.h>

#include "tl_turtle_track/Axis.h"
#include "tl_turtle_track/PanTilt.h"
#include "tl_turtle_track/PanTilts.h"
#include "tl_turtle_track/record.h"
#include "tl_turtle_track/Entity.h"
#include "tl_turtle_track/Entities.h"
#include "tl_turtle_track/State.h"
#include "tl_turtle_track/States.h"

/*
 * From fix
 */

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include "ukf.h"
#include "ekf.h"
#include <sys/time.h>

using namespace ekf;
//using namespace ukf;

#define SIGMA 10.0
#define RHO 28.0
#define BETA 8.0/3.0
#define DT 0.01
#define X0 0.1
#define Y0 1.2
#define Z0 0.4
#define NOISE_AMPLITUDE 2.0
#define VERBOSE true

#define N_X 6
#define N_Y 2

/*************************************************************************************/
/*            Definition of the evolution and observation functions                  */
/*************************************************************************************/

// Evolution function
void f(gsl_vector * params, gsl_vector * xk_1, gsl_vector * xk, std::pair<double, double>& U)
{
    double x       = gsl_vector_get(xk_1,0);
    double y       = gsl_vector_get(xk_1,1);
    double theta   = gsl_vector_get(xk_1,2);
    double dt      = gsl_vector_get(params, 0);
    
    double v       = U.first; 
    double omega   = U.second;
    
    gsl_vector_set(xk, 0, x     + dt * v * std::cos(theta));
    gsl_vector_set(xk, 1, y     + dt * v * std::sin(theta));
    gsl_vector_set(xk, 2, theta + dt * omega);

    //ROS_INFO_STREAM("v " << v << " omega " << omega);
}

// Jacobian of the evolution function
void df(gsl_vector * params, gsl_vector * xk_1, gsl_matrix * Fxk, std::pair<double, double>& U)
{
    double x       = gsl_vector_get(xk_1,0);
    double y       = gsl_vector_get(xk_1,1);
    double theta   = gsl_vector_get(xk_1,2);
    double dt      = gsl_vector_get(params, 0);
    
    double v       = U.first; 
    double omega   = U.second;
    
    // Derivatives for x(t+1) = ..
    gsl_matrix_set(Fxk, 0, 0, 1.0);
    gsl_matrix_set(Fxk, 0, 1, 0.0);
    gsl_matrix_set(Fxk, 0, 2, -v*std::sin(theta)*dt);
    
    // Derivatives for y(t+1)
    gsl_matrix_set(Fxk, 1, 0, 0.0);
    gsl_matrix_set(Fxk, 1, 1, 1.0);
    gsl_matrix_set(Fxk, 1, 2, v*std::cos(theta)*dt);
    
    // Derivatives for theta(t+1)
    gsl_matrix_set(Fxk, 2, 0, 0.0);
    gsl_matrix_set(Fxk, 2, 1, 0.0);
    gsl_matrix_set(Fxk, 2, 2, 1.0);
}

// Observation function
void h(gsl_vector * params, gsl_vector * xk , gsl_vector * yk)
{
    for(unsigned int i = 0 ; i < yk->size ; ++i)
      //gsl_vector_set(yk, i, gsl_vector_get(xk,i) + NOISE_AMPLITUDE*rand()/ double(RAND_MAX)); // WHY???
      gsl_vector_set(yk, i, gsl_vector_get(xk,i));
}

// Jacobian of the observation function
void dh(gsl_vector * params, gsl_vector * xk , gsl_matrix * Hyk)
{
    gsl_matrix_set_zero(Hyk);
    gsl_matrix_set(Hyk, 0, 0, 1.0);
    gsl_matrix_set(Hyk, 1, 1, 1.0);
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */


using turtlebot = std::pair<ekf_param, ekf_state>;
using map = std::unordered_map<int, turtlebot>;

void cmd_callback(const geometry_msgs::Twist::ConstPtr&              msg,
		  std::pair<double, double>&                         U
		  )
{
  double x_1 = msg->linear.x;
  double y_1 = msg->linear.y;
  double theta_1 = msg->angular.z;
  
  double v = std::sqrt(std::pow(x_1,2) + std::pow(y_1,2));
  
  U.first = v;
  U.second = theta_1;
}

void kalman_callback(const tl_turtle_track::Entities::ConstPtr&              msg,
		     map&                                                    turtlebots,
		     ros::Publisher&                                         pub,
		     gsl_vector*&                                            yi,
		     std::chrono::high_resolution_clock::time_point&         last_time,
		     std::pair<double, double>&                              U
		     )
{
  auto new_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt_s = new_time - last_time;
  double dt = dt_s.count();
  last_time = new_time;

  std::vector<tl_turtle_track::State> states_vec;
   
  // Searches turtlebots (map) for each element of msg->array (Entity[])
  for(const tl_turtle_track::Entity& entity : msg->array) {
    map::iterator got = turtlebots.find(entity.nb);
    
    if(got == turtlebots.end()) { // NOT FOUND
      // BEGIN FIX
      
      // Definition of the parameters and state variables
      ekf_param p;
      ekf_state s;
      
      // The parameters for the evolution equation
      s.params = gsl_vector_alloc(1);
      s.params->data[0] = dt;
      
      // Initialization of the parameters
      p.n  = N_X;
      p.no = N_Y;
      
      //EvolutionNoise * evolution_noise = new ekf::EvolutionAnneal(1e-2, 0.99, 1e-8);
      EvolutionNoise * evolution_noise = new ekf::EvolutionRLS(1e-2, 0.9995);
      //EvolutionNoise * evolution_noise = new ekf::EvolutionRobbinsMonro(1e-5, 1e-6);
      
      p.evolution_noise = evolution_noise;
      
      p.observation_noise = 1e-1;
      
      p.prior_pk = 1.0;
      p.observation_gradient_is_diagonal = true;
      
      // Initialization of the state and parameters
      ekf_init(p,s);
      
      // Initialize the parameter vector to some random valuœes
      for(int i = 0 ; i < p.n ; i++)
	gsl_vector_set(s.xk,i,5.0*(2.0*rand()/double(RAND_MAX-1)-1.0));
      
      s.xk->data[0] = entity.pos.x;
      s.xk->data[1] = entity.pos.y;

      // END FIX


      turtlebot turtlebot(p,s);
      auto result = turtlebots.emplace(entity.nb, turtlebot);
      if(!result.second)
      // if(!turtlebots.insert(std::make_pair(entity.nb, turtlebot)))
	ROS_ERROR_STREAM("EMPLACE FAILED WITH " << entity.nb);

      else
	ROS_INFO_STREAM("SUCCESSFULLY ADDED " << entity.nb);
    }
    
    else {
      ekf_state& s = got->second.second;
      ekf_param& p = got->second.first;

      s.params->data[0] = dt;

      gsl_vector_set(yi, 0, entity.pos.x);
      gsl_vector_set(yi, 1, entity.pos.y);
      

      auto fp = std::bind(f,
			  std::placeholders::_1,
			  std::placeholders::_2,
			  std::placeholders::_3,
			  std::ref(U)
			  );

      auto dfp = std::bind(df,
			   std::placeholders::_1,
			   std::placeholders::_2,
			   std::placeholders::_3,
			   std::ref(U)
			   );
      
      // Provide the observation and iterate
      ekf_iterate(p,s,fp,dfp,h,dh,yi);

      tl_turtle_track::State state_out;
      state_out.entity.pos.x = s.xk->data[0];
      state_out.entity.pos.y = s.xk->data[1];
      state_out.entity.nb = entity.nb;
      state_out.angle = s.xk->data[2];

      states_vec.push_back(state_out);
    }
  }

  // Searches msg->array (Entity[]) for each element of turtlebots (map)
  for(auto it = turtlebots.cbegin(); it != turtlebots.cend(); ) {
    auto begin = msg->array.begin();
    auto end   = msg->array.end();
    
    auto got = find_if(begin, end,
		       [&it](const tl_turtle_track::Entity& entity) {
			 return (entity.nb == it->first);  
		       }
		       );

    if(got == end){
      ROS_INFO_STREAM("SUCCESSFULLY Removed " << it->first);
      //ekf_free(it->second.first,it->second.second);
      turtlebots.erase(it++);
    }
    
    else
      ++it;
  }

  tl_turtle_track::States states_out;
  states_out.array = states_vec;

  pub.publish(states_out);
}


int main(int argc, char * argv[]) {

      ros::init(argc, argv, "sampler");
      ros::NodeHandle nh;

      map turtlebots;
      gsl_vector * xi = gsl_vector_alloc(N_X);
      gsl_vector * yi = gsl_vector_alloc(N_Y);
      gsl_vector_set_zero(yi);
      

      //      fp = std::bind(f,
      //	     std::placeholders::_1,
      //	     std::placeholders::_2,
      //	     std::placeholders::_3,
      //	     std::ref(U)
      //	     ));

      //dfp = std::bind(df,
      //	     std::placeholders::_1,
      //	     std::placeholders::_2,
      //	     std::placeholders::_3,
      //	     std::ref(U)
      //	     ));

      std::pair<double, double> U;

      auto last_time = std::chrono::high_resolution_clock::now();

      ros::Publisher pub = nh.advertise<tl_turtle_track::States>("/states", 1);

      ros::Subscriber sub_entities = nh.subscribe<tl_turtle_track::Entities>
	("entities",1,std::bind(kalman_callback,
				std::placeholders::_1,
				std::ref(turtlebots),
				std::ref(pub),
				std::ref(yi),
				std::ref(last_time),
				std::ref(U)
				));

      ros::Subscriber sub_cmd = nh.subscribe<geometry_msgs::Twist>
	("cmd",1,std::bind(cmd_callback,
				std::placeholders::_1,
				std::ref(U)
				));

      ros::spin();
}
