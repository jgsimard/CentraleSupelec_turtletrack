/*

  g++ -o train_svm -Wall -ansi -O3 train_svm.cpp -lsvm -lm -std=c++11 -I/home/myadmin/gaml/gaml-libsvm -I/usr/include/gaml


*/

#include <gaml-libsvm.hpp>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <utility>
#include <string>
#include <fstream>
#include <iostream>
#include <ctime>

#include <ros/ros.h>

typedef std::pair<double,double> XY;
typedef double                   Z;
typedef std::pair<XY,Z>          Data;
typedef std::vector<Data>        DataSet;

// We need a function that builds an array of svm_nodes for
// representing some input. When the input is a collection of values
// that can provide iterators, libsvm::input_of can help. Here, we
// have to write it by ourselves.
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
double    output_of(const Data& data) {return data.second;} // this should return a double for libsvm.

// This is gnuplots a function
#define PLOT_STEP .04
template<typename Func>
void gnuplot(std::string filename,
             std::string title,
             const Func& f) {
  XY x;
  std::ofstream file;
  file.open(filename.c_str());
  if(!file) {
    std::cerr << "Cannot open \"" << filename << "\"." << std::endl;
    return;
  }
  file << "set hidden3d" << std::endl
       << "set title \"" << title << "\"" << std::endl
       << "set view 41,45" << std::endl
       << "set xlabel \"x\"" << std::endl
       << "set ylabel \"y\"" << std::endl
       << "set zlabel \"z\"" << std::endl
       << "set ticslevel 0" << std::endl
       << "splot '-' using 1:2:3 with lines notitle" << std::endl;
  for(x.first=-1;x.first<=1;x.first+=PLOT_STEP,file << std::endl)
    for(x.second=-1;x.second<=1;x.second+=PLOT_STEP,file << std::endl)
      file << x.first << ' ' << x.second << ' ' << f(x);
  file.close();
  std::cout << "Gnuplot file \"" << filename << "\" generated." << std::endl;
}


int main(int argc, char* argv[]) {

  

  ros::init(argc, argv, "share");
  ros::NodeHandle nh;

  std::ifstream file;
  file.open("/record.txt");
  double x_, y_, pan_, tilt_;
  std::vector<double> x, y, pan, tilt;

  while(!file.eof()) {
    file >> x_ >> y_ >> pan_ >> tilt_;
    x.push_back(x_);
    y.push_back(y_);
    pan.push_back(pan_);
    tilt.push_back(tilt_);
  }
  
  // Let us make libsvm quiet
  gaml::libsvm::quiet();
  
  int nb_samples = x.size();

  std::cout << "I am using at least " << nb_samples << " samples." << std::endl;
  
  try {
    // Let us collect samples.
    
    DataSet basis_pan;
    DataSet basis_tilt;
    
    basis_pan.resize(nb_samples);
    basis_tilt.resize(nb_samples);
    
    for(int i = 0; i < nb_samples; i++) {

      XY xy(x[i], // x
            y[i]); // y
	    
      basis_pan[i] = Data(xy,      // (x,y)
                          pan[i]); // z = f(x,y)
    }
    
    for(int i = 0; i < nb_samples; i++) {

      XY xy(x[i], // x
            y[i]); // y
	    
      basis_tilt[i] = Data(xy,      // (x,y)
                          tilt[i]); // z = f(x,y)
    }
    
    // Let us set configure a svm
    struct svm_parameter params;
    gaml::libsvm::init(params);
    params.kernel_type = RBF;          // RBF kernel
    params.gamma       = 10;           // k(u,v) = exp(-gamma*(u-v)^2)
    params.svm_type    = EPSILON_SVR;
    params.p           = .05;          // epsilon
    params.C           = 10;
    params.eps         = 1e-5;         // numerical tolerence
    
    // This sets up a svm learning algorithm.
    auto learner_pan  = gaml::libsvm::supervized::learner<XY,Z>(params, nb_nodes_of, fill_nodes);
    auto learner_tilt = gaml::libsvm::supervized::learner<XY,Z>(params, nb_nodes_of, fill_nodes);
    
    // Let us train it and get some predictor f. f is a function, as the oracle.
    std::cout << "Learning..." << std::endl;
    auto f_pan  = learner(basis_pan.begin(), basis_pan.end(), input_of, output_of);
    auto f_tilt = learner(basis_tilt.begin(), basis_tilt.end(), input_of, output_of);
    
    // Let us plot the result.
    gnuplot("prediction_pan.plot","SVM pan prediction",f_pan);
    gnuplot("prediction_tilt.plot","SVM tilt prediction",f_tilt);
    
    // All libsvm functions related to svm models are implemented.
    std::cout << std::endl
              << "There are " << f_pan.get_nr_sv() << " pan support vectors." << std::endl;
    std::cout << std::endl
              << "There are " << f_tilt.get_nr_sv() << " tilt support vectors." << std::endl;
    
    // We can compute the empirical risk with gaml tools.
    auto evaluator = gaml::risk::empirical(gaml::loss::Quadratic<double>());
    double risk_pan     = evaluator(f_pan, basis_pan.begin(), basis_pan.end(), input_of, output_of);
    double risk_tilt    = evaluator(f_tilt, basis_tilt.begin(), basis_tilt.end(), input_of, output_of);
    
    std::cout << "Empirical quadratic pan risk : " << risk_pan << std::endl
              << "               i.e error : " << sqrt(risk_pan) << std::endl;
    
    std::cout << "Empirical quadratic tilt risk : " << risk_tilt << std::endl
              << "               i.e error : " << sqrt(risk_tilt) << std::endl;
    
    // Let us use a cross-validation procedure
    auto kfold_evaluator = gaml::risk::cross_validation(gaml::loss::Quadratic<double>(),
                                                        gaml::partition::kfold(10), true);
    
    risk_pan                 = kfold_evaluator(learner_pan, basis_pan.begin(), basis_pan.end(), input_of, output_of);
    
    std::cout << "Real quadratic pan risk estimation : " << risk_pan << std::endl
              << "                     i.e error : " << sqrt(risk_pan) << std::endl;
    
    risk_tilt                = kfold_evaluator(learner, basis_tilt.begin(), basis_tilt.end(), input_of, output_of);
    
    std::cout << "Real quadratic tilt risk estimation : " << risk_tilt << std::endl
              << "                     i.e error : " << sqrt(risk_tilt) << std::endl;
    
    // Now, let us save our predictor.
    f_pan.save_model("svm_pan.pred");
    f_tilt.save_model("svm_tilt.pred");
  }
  catch(gaml::exception::Any& e) {
    std::cout << e.what() << std::endl;
  }
  
  return 0;
}
