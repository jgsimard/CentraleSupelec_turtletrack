#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vq2.h>

#include <utility>
#include <vector>
#include <map>
#include <ctime>
#include <cstdlib>
#include <iterator>

#include <limits>
#include <cmath>

#include "tl_turtle_track/Entity.h"
#include "tl_turtle_track/Entities.h"
#include "tl_turtle_track/ArenaPosition.h"
#include "tl_turtle_track/ArenaPositions.h"

// ###############
// #             #
// # GNG-T Stuff #
// #             #
// ###############
//       ###
//       ###
//     #######
//      ##### 
//       ###
//        #

// This is inspired from vq2 examples.

// This defines the graph
typedef vq2::algo::gngt::Unit<pcl::PointXYZ>         Unit;
typedef vq2::Graph<Unit,char,Unit::copy_constructor> GNGT;

// For finding the winner in the GNGT algorithm, we need some
// similarity measure. Here, let us use the squared euclidian
// distance.
class Similarity {
public:
  typedef pcl::PointXYZ value_type;
  typedef pcl::PointXYZ sample_type;
  double operator()(const value_type& arg1,
		    const sample_type& arg2) {
    double dx = arg1.x - arg2.x;
    double dy = arg1.y - arg2.y;
    return dx*dx + dy*dy;
  }
};
typedef vq2::unit::Similarity<Unit,Similarity> UnitSimilarity;

// The learning process.
class Learn {
public:
  typedef pcl::PointXYZ sample_type;
  typedef pcl::PointXYZ weight_type;
  void operator()(double coef,
                  weight_type& prototype,
                  const sample_type& target) {
    prototype.x += coef * (target.x - prototype.x);
    prototype.y += coef * (target.y - prototype.y);
  }
};
typedef vq2::unit::Learn<Unit,Learn> UnitLearn;

// This is the parameter set for GNG-T
struct Params {
public:
  // GNG-T
  int ageMax(void)           {return 20;}
  double learningRate(void)  {return .005;}
  double learningRatio(void) {return .2;}
  double lambda(void)        {return .001;}
  // Evolution
  double target(void)        {return 1e-5;}
  double nbSamples(void)     {return 10000;}
  double lowPassCoef(void)   {return .4;}
  double delta(void)         {return .75;}
  double margin(void)        {return .2;}
};

// Some evolution class is required to control the GNG-T
// evolution. Let us use the default one here.
typedef vq2::by_default::gngt::Evolution<Params> Evolution;

// This functor collects vertices and edges.
class Get {
public:
  std::vector<pcl::PointXYZ>                             vertices;
  std::vector< std::pair<pcl::PointXYZ,pcl::PointXYZ> >  edges;
  
  bool operator()(GNGT::edge_type& e) { 
    if(e.stuff.efficient)
      edges.push_back(std::make_pair((*(e.n1)).value.prototype(),
				     (*(e.n2)).value.prototype()));
    return false;
  }

  bool operator()(GNGT::vertex_type& v) { 
    if(v.stuff.efficient)
      vertices.push_back(v.value.prototype());
    return false;
  }
};


//        #
//       ###
//      ##### 
//     #######
//       ###
//       ###
// ###############
// #             #
// # GNG-T Stuff #
// #             #
// ###############

double uniform(double min, double max) {
  return min + (max-min)*(std::rand()/(1.0+RAND_MAX));
}

// This is a class for allocating a color to each graph component in a
// smart way.
struct Color {
  double r,g,b;
};

class Colormap {
private:
  std::map<unsigned int, Color> color_of;
  std::map<unsigned int, Color> tmp;

  Color new_color() {
    double a = uniform(0,1);
    double b = uniform(0,1);
    int idx = (int)(uniform(0,3));
    if(idx == 0) return {1.0,   a,   b};
    if(idx == 1) return {  a, 1.0,   b};
    else         return {  a,   b, 1.0};
  }

public:
  
  void remap() {
    tmp = color_of;
    color_of.clear();
  }
  
  void allocate(unsigned int label) {
    auto existing = tmp.find(label);
    if(existing != tmp.end())
      color_of[label] = existing->second;
    else
      color_of[label] = new_color();
  }
  
  Color operator()(unsigned int label) {
    return color_of[label];
  }
};

// This class helps to register marker idfs and clean unused ones.
class MarkerIdfs {
private:

  std::map<int,bool> idf_in_use;

public:

  void reset() {
    for(auto& key_val : idf_in_use) key_val.second = false;
  }

  void use(int idf) {
    idf_in_use[idf] = true;
  }

  void cleanup(ros::Publisher& pub) {
    std::vector<int> unused;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "gngt";
    marker.action = visualization_msgs::Marker::DELETE;
    
    for(auto& key_val : idf_in_use)
      if(!(key_val.second)) {
	unused.push_back(key_val.first);
	marker.id = key_val.first;
	pub.publish(marker);
      }
    
    for(auto idf : unused)idf_in_use.erase(idf);
  }

};


void draw_vertices(ros::Publisher& pub,const Color color, int id, const std::vector<pcl::PointXYZ>& vertices);
void draw_edges(ros::Publisher& pub, const Color color, int id, const std::vector< std::pair<pcl::PointXYZ,pcl::PointXYZ> >& edges);


struct bind_data
{
  GNGT             gngt;
  Params           params;
  Similarity       distance;
  UnitSimilarity   unit_distance;
  Learn            learn;
  UnitLearn        unit_learn;
  Evolution        evolution;
  Colormap         cmap;
  MarkerIdfs       marker_idfs;

  bind_data() : unit_distance(distance), unit_learn(learn), evolution(params) {}

};
  
#define NB_STABILIZATION 5
#define NB_SAMPLES 500
#define ARENA_MIN 0
#define ARENA_MAX 10
#define DIST_MAX 0.5
void gngt_callback(ros::Publisher&                         pub,
		   ros::Publisher&                         pub_pc,
		   ros::Publisher&                         pub_entities,
		   bind_data&                              bind_struct,
		   const tl_turtle_track::ArenaPositions::ConstPtr &msg) {
  
  pcl::PointCloud<pcl::PointXYZ> points;

  for(const tl_turtle_track::ArenaPosition& ap : msg->ArenaPosition_Array)
    points.push_back(pcl::PointXYZ(ap.x, ap.y, 0));
  
  int nb_pts = msg->ArenaPosition_Array.size();
  while (nb_pts < NB_SAMPLES){
    double x = uniform(ARENA_MIN, ARENA_MAX);
    double y = uniform(ARENA_MIN, ARENA_MAX);
    for(const tl_turtle_track::ArenaPosition& ap : msg->ArenaPosition_Array){
      double dist = std::sqrt(std::pow((ap.x-x),2.0) + std::pow((ap.y-y),2.0));

      if(dist < DIST_MAX){
  	points.push_back(pcl::PointXYZ(x, y, 0));
  	nb_pts++;
        break;
      }
    }
  }

  pcl::PCLPointCloud2 data_2;
  pcl::toPCLPointCloud2 (points, data_2);

  sensor_msgs::PointCloud2 pc_output;
  pcl_conversions::fromPCL(data_2,pc_output);
  
  pc_output.header.stamp    = ros::Time::now();
  pc_output.header.frame_id = "/map";

  pub_pc.publish(pc_output);
  
  // Let us use few epochs to stabilize the current graph to the new
  // input (no graph evolution, only adjustment).
  
  for(int i=0; i< NB_STABILIZATION; ++i) {
    std::random_shuffle(points.begin(), points.end());
    vq2::algo::gngt::open_epoch(bind_struct.gngt,bind_struct.evolution);
    for(auto& pt : points) 
      vq2::algo::gngt::submit(bind_struct.params,
    			      bind_struct.gngt,
    			      bind_struct.unit_distance,
    			      bind_struct.unit_learn,
    			      pt,false);
    
    vq2::algo::gngt::close_epoch(bind_struct.params,
    				 bind_struct.gngt,
    				 bind_struct.unit_learn,
    				 bind_struct.evolution,
    				 false);
  }

  // Update the graph (add or remove egdes and vertices)
  vq2::algo::gngt::open_epoch(bind_struct.gngt,bind_struct.evolution);
  for(auto& pt : points) 
    vq2::algo::gngt::submit(bind_struct.params,
  			    bind_struct.gngt,
  			    bind_struct.unit_distance,
  			    bind_struct.unit_learn,
  			    pt,true);
  
  vq2::algo::gngt::close_epoch(bind_struct.params,
  			       bind_struct.gngt,
  			       bind_struct.unit_learn,
  			       bind_struct.evolution,
  			       true);
  
  
  // Update the connected components and their labels.
  std::map<unsigned int,GNGT::Component*> components;
  bind_struct.gngt.computeConnectedComponents(components,true);
  bind_struct.cmap.remap();
  for(auto& key_val : components) bind_struct.cmap.allocate(key_val.first);

  // Let us now send the graph to Rviz, component by component.
  bind_struct.marker_idfs.reset();
  for(auto& key_val : components) {
    Get get;
    int idf;
    auto label    = key_val.first;
    auto comp_ptr = key_val.second;
    comp_ptr->for_each_vertex(get);
    comp_ptr->for_each_edge(get);
    
    auto color = bind_struct.cmap(label);
    idf = 2*label; draw_vertices(pub,color,idf,get.vertices); bind_struct.marker_idfs.use(idf);
    ++idf;         draw_edges   (pub,color,idf,get.edges);    bind_struct.marker_idfs.use(idf);
  }
  bind_struct.marker_idfs.cleanup(pub);
}



// RVIZ drawing stuff.
void draw_vertices(ros::Publisher& pub, const Color color, int id, const std::vector<pcl::PointXYZ>& vertices) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "gngt";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  
  auto out = std::back_inserter(marker.points);
  for(auto& pt : vertices) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    *(out++) = p;
  }
  pub.publish(marker);
}

#define EDGE_DARKEN_COEF .5
void draw_edges(ros::Publisher& pub, const Color color, int id, const std::vector< std::pair<pcl::PointXYZ,pcl::PointXYZ> >& edges) {
  visualization_msgs::Marker marker;


  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "gngt";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.a = 1.0; 
  marker.color.r = color.r*EDGE_DARKEN_COEF;
  marker.color.g = color.g*EDGE_DARKEN_COEF;
  marker.color.b = color.b*EDGE_DARKEN_COEF;
  
  auto out = std::back_inserter(marker.points);
  for(auto& e : edges) {
    geometry_msgs::Point p;
    p.x = e.first.x;
    p.y = e.first.y;
    *(out++) = p;
    p.x = e.second.x;
    p.y = e.second.y;
    *(out++) = p;
  }
  pub.publish(marker);
}


int main(int argc, char **argv)
{
  
  bind_data bind_struct;

  std::srand(std::time(0));

  ros::init(argc, argv, "gngt");
  ros::NodeHandle n;

  ros::Publisher pub_entities = n.advertise<tl_turtle_track::Entities>("/entities_out", 0);
  
  ros::Publisher pub_pc = n.advertise<sensor_msgs::PointCloud2> ("/point_cloud", 1);

  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("markers", 0);
  ros::Subscriber sub = n.subscribe<tl_turtle_track::ArenaPositions>("input", 1, 
							      boost::bind(gngt_callback,
									  boost::ref(pub),
									  boost::ref(pub_pc),
									  boost::ref(pub_entities),
									  boost::ref(bind_struct),
									  _1));
  
  ros::spin();

  return 0;
}

