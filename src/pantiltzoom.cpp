#include <cmath>
#include "pantiltzoom.hpp"

std::pair<double,double> pantiltzoom(double u,   double v,
				     double u0,  double v0,
				     double pan, double tilt, double zoom)
{
  double focale,theta,alpha0,beta0,alpha,beta;
  double x,y,z,X,Y,Z,norme;
  theta=4.189301e+001-6.436043e-003*zoom+2.404497e-007*zoom*zoom;
  focale=u0/std::tan((M_PI*theta/180.0)/2);

  x=u-u0;y=v-v0;z=focale;
  norme=std::sqrt(x*x+y*y+z*z);
  x/=norme;y/=norme;z/=norme;

  beta0=-(M_PI*pan/180.0);
  alpha0=-(M_PI*tilt/180.0);
  X=std::cos(beta0)*x+std::sin(alpha0)*std::sin(beta0)*y-std::cos(alpha0)*std::sin(beta0)*z;
  Y=std::cos(alpha0)*y+std::sin(alpha0)*z;
  Z=std::sin(beta0)*x-std::sin(alpha0)*std::cos(beta0)*y+std::cos(alpha0)*std::cos(beta0)*z;
  alpha=std::atan2(Y,sqrt(X*X+Z*Z));
  beta=-std::atan2(X,Z);

  return {-(180.0*beta/M_PI), -(180.0*alpha/M_PI)};
}
