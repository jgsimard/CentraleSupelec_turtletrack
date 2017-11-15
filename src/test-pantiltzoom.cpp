/* 
   g++ -o test-pantiltzoom -O3 -Wall test-pantiltzoom.cpp pantiltzoom.cpp -std=c++11
*/

#include <tuple>
#include <cstdlib>
#include <iostream>
#include "pantiltzoom.hpp"

int main(int argc,char* argv[])
{
  double pan0,tilt0,u,v,u0,v0,zoom;
  if (argc!=8) {
    std::cerr << "usage : " << argv[0] << "pan tilt zoom u0 v0 u v " << std::endl;
    return -1;
  }

  pan0   = atof(argv[1]);
  tilt0  = atof(argv[2]);
  zoom   = atof(argv[3]);
  u0     = atof(argv[4]);
  v0     = atof(argv[5]);
  u      = atof(argv[6]);
  v      = atof(argv[7]);
  
  double pan, tilt;
  std::tie(pan,tilt) = pantiltzoom(u,v,u0,v0,pan0,tilt0,zoom);
  std::cout << "pan = " << pan << " deg, tilt = " << tilt << " deg." << std::endl;
  return 0;
}
