/*

  g++ -o train_svm  train_svm.cpp -Wall -ansi -O3 -lsvm -lm -std=c++11 -I/home/myadmin/gaml/gaml-libsvm -I/usr/include/gaml

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

typedef std::pair<double,double> PT;
typedef double                   Z;
typedef std::pair<PT,Z>          Data;
typedef std::vector<Data>        DataSet;

// We need a function that builds an array of svm_nodes for
// representing some input. When the input is a collection of values
// that can provide iterators, libsvm::input_of can help. Here, we
// have to write it by ourselves.
// x y are stored with 3 nodes...
int nb_nodes_of(const PT& pt) {
  return 3;
}
// ... as follows.
void fill_nodes(const PT& pt,struct svm_node* nodes) {
  nodes[0].index = 1;
  nodes[0].value = pt.first;  // x 
  nodes[1].index = 2;
  nodes[1].value = pt.second; // y
  nodes[2].index = -1;        // end
}
const PT& input_of (const Data& data) {return data.first;}
double    output_of(const Data& data) {return data.second;} // this should return a double for libsvm.

// This is gnuplots a function
#define PLOT_STEP 1
template<typename Func>
void gnuplot(std::string filename,
             std::string title,
             const Func& f) {
  PT x;
  std::ofstream file;
  file.open(filename.c_str());
  if(!file) {
    std::cerr << "Cannot open \"" << filename << "\"." << std::endl;
    return;
  }
  file << "set hidden3d" << std::endl
       << "set title \"" << title << "\"" << std::endl
       << "set view 41,45" << std::endl
       << "set xlabel \"pan\"" << std::endl
       << "set ylabel \"tilt\"" << std::endl
       << "set zlabel \"x or y\"" << std::endl
       << "set ticslevel 0" << std::endl
       << "splot '-' using 1:2:3 with lines notitle" << std::endl;

  int minimum_pan = -5, maximum_pan = 65;
  int minimum_tilt = -60, maximum_tilt = -20;
  for(x.first=minimum_pan; x.first<=maximum_pan; x.first+=PLOT_STEP,file << std::endl)
    for(x.second=minimum_tilt; x.second<=maximum_tilt; x.second+=PLOT_STEP,file << std::endl)
      file << x.first << ' ' << x.second << ' ' << f(x);
  file.close();
  std::cout << "Gnuplot file \"" << filename << "\" generated." << std::endl;
}


int main(int argc, char* argv[]) {

  std::ifstream file;
  file.open("record.txt");
  double x_, y_, pan_, tilt_;
  std::vector<double> x, y, pan, tilt;
  if (!file)
    return -1;
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
    
    DataSet basis_x;
    DataSet basis_y;
    
    basis_x.resize(nb_samples);
    basis_y.resize(nb_samples);
    
    for(int i = 0; i < nb_samples; i++) {

      PT pt(pan[i], // pan
            tilt[i]); // tilt
	    
      basis_x[i] = Data(pt,      // (pan, tilt)
                          x[i]); // z = f(pan, tilt)
    }
    
    for(int i = 0; i < nb_samples; i++) {

      PT pt(pan[i], // pan
            tilt[i]); // tilt
	    
      basis_y[i] = Data(pt,      // (pan, tilt)
                          y[i]); // z = f(pan, tilt)
    }
    
    // Let us set configure a svm
    struct svm_parameter params;
    gaml::libsvm::init(params);
    params.kernel_type = RBF;          // RBF kernel
    params.gamma       = 0.001;           // k(u,v) = exp(-gamma*(u-v)^2)
    params.svm_type    = EPSILON_SVR;
    params.p           = .05;          // epsilon
    params.C           = 10;
    params.eps         = 1e-5;         // numerical tolerence
    
    // This sets up a svm learning algorithm.
    auto learner_x  = gaml::libsvm::supervized::learner<PT,Z>(params, nb_nodes_of, fill_nodes);
    auto learner_y = gaml::libsvm::supervized::learner<PT,Z>(params, nb_nodes_of, fill_nodes);
    
    // Let us train it and get some predictor f. f is a function, as the oracle.
    std::cout << "Learning..." << std::endl;
    auto f_x  = learner_x(basis_x.begin(), basis_x.end(), input_of, output_of);
    auto f_y = learner_y(basis_y.begin(), basis_y.end(), input_of, output_of);
    
    // Let us plot the result.
    gnuplot("prediction_x.plot","SVM x prediction",f_x);
    gnuplot("prediction_y.plot","SVM y prediction",f_y);
    
    // All libsvm functions related to svm models are implemented.
    std::cout << std::endl
              << "There are " << f_x.get_nr_sv() << " x support vectors." << std::endl;
    std::cout << std::endl
              << "There are " << f_y.get_nr_sv() << " y support vectors." << std::endl;
    
    // We can compute the empirical risk with gaml tools.
    auto evaluator = gaml::risk::empirical(gaml::loss::Quadratic<double>());
    double risk_x     = evaluator(f_x, basis_x.begin(), basis_x.end(), input_of, output_of);
    double risk_y    = evaluator(f_y, basis_y.begin(), basis_y.end(), input_of, output_of);
    
    std::cout << "Empirical quadratic x risk : " << risk_x << std::endl
              << "               i.e error : " << sqrt(risk_x) << std::endl;
    
    std::cout << "Empirical quadratic y risk : " << risk_y << std::endl
              << "               i.e error : " << sqrt(risk_y) << std::endl;
    
    // Let us use a cross-validation procedure
    auto kfold_evaluator = gaml::risk::cross_validation(gaml::loss::Quadratic<double>(),
                                                        gaml::partition::kfold(10), true);
    
    risk_x                 = kfold_evaluator(learner_x, basis_x.begin(), basis_x.end(), input_of, output_of);
    
    std::cout << "Real quadratic x risk estimation : " << risk_x << std::endl
              << "                     i.e error : " << sqrt(risk_x) << std::endl;
    
    risk_y                = kfold_evaluator(learner_y, basis_y.begin(), basis_y.end(), input_of, output_of);
    
    std::cout << "Real quadratic y risk estimation : " << risk_y << std::endl
              << "                     i.e error : " << sqrt(risk_y) << std::endl;
    
    // Now, let us save our predictor.
    f_x.save_model("svm_x.pred");
    f_y.save_model("svm_y.pred");
  }
  catch(gaml::exception::Any& e) {
    std::cout << e.what() << std::endl;
  }
  
  return 0;
}
