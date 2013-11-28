#include <stdio.h>
#include "rviz/color.hh"
namespace ros{
  Color::Color(){
    r=0.6;g=0;b=0.6;a=0.4;
  }
  Color::Color(const Color& c){
    r=c.r;g=c.g;b=c.b;a=c.a;
  }
  Color::Color(double r, double g, double b, double a){
    this->r = r;this->g=g;this->b=b;this->a=a;
  }
  void Color::print(){
    printf("COLOR -> %f %f %f %f", r,g,b,a);
  }
};
