#include "rviz/visualmarker.h"

namespace ros{
PrimitiveMarkerBox::PrimitiveMarkerBox(double x, double y, double w, double l, double h): PrimitiveMarkerTriangle() {
  g_.x_ = x;
  g_.y_ = y;
  g_.z_ = 0.0;
  g_.setRPYRadian(0,0,0);

  box_h_ = h;
  box_w_ = w;
  box_l_ = l;

  g_.sx_=1.0;
  g_.sy_=1.0;
  g_.sz_=1.0;

  initPrimitiveMarker(this);
}
std::string PrimitiveMarkerBox::name(){
  return std::string("toolbox_triangles");
}
#ifdef FCL_COLLISION_CHECKING
std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > PrimitiveMarkerBox::getVerticesAndTriangles(){

  std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> triangles;

  //ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", bvh->num_tris, bvh->num_vertices);
  double x = g_.x_;
  double y = g_.y_;
  double z = g_.z_;
  double sx = g_.sx_;
  double sy = g_.sy_;
  double sz = g_.sz_;
  double yaw = g_.getYawRadian();

  //bottom
  uint i = 0;

  //xlu - x left up, xrd - x right down
  double xlu = box_l_/2;
  double ylu = box_w_/2;
  double zlu = 0;

  double xru = box_l_/2;
  double yru = -box_w_/2;
  double zru = 0;

  double xld = -box_l_/2;
  double yld = box_w_/2;
  double zld = 0;

  double xrd = -box_l_/2;
  double yrd = -box_w_/2;
  double zrd = 0;

  ROS_INFO("%f %f %f %f", xlu, xru, xld, xrd);

  fcl::Vec3f luB(xlu, ylu, zlu);
  fcl::Vec3f ruB(xru, yru, zru);
  fcl::Vec3f ldB(xld, yld, zld);
  fcl::Vec3f rdB(xrd, yrd, zrd);

  double xlut = box_l_/2;
  double ylut = box_w_/2;
  double zlut = box_h_;

  double xrut = box_l_/2;
  double yrut = -box_w_/2;
  double zrut = box_h_;

  double xldt = -box_l_/2;
  double yldt = box_w_/2;
  double zldt = box_h_;

  double xrdt = -box_l_/2;
  double yrdt = -box_w_/2;
  double zrdt = box_h_;

  fcl::Vec3f luT(xlut, ylut, zlut);
  fcl::Vec3f ruT(xrut, yrut, zrut);
  fcl::Vec3f ldT(xldt, yldt, zldt);
  fcl::Vec3f rdT(xrdt, yrdt, zrdt);

  vertices.push_back(luB);
  vertices.push_back(ruB);
  vertices.push_back(rdB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(luB);
  vertices.push_back(ldB);
  vertices.push_back(rdB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(luT);
  vertices.push_back(ruT);
  vertices.push_back(rdT);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(luT);
  vertices.push_back(ldT);
  vertices.push_back(rdT);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  //LEFT SIDE
  vertices.push_back(luT);
  vertices.push_back(ldT);
  vertices.push_back(ldB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(luT);
  vertices.push_back(luB);
  vertices.push_back(ldB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  //TOP SIDE
  vertices.push_back(luT);
  vertices.push_back(ruT);
  vertices.push_back(luB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(ruB);
  vertices.push_back(ruT);
  vertices.push_back(luB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  //RIGHT SIDE
  vertices.push_back(ruT);
  vertices.push_back(rdT);
  vertices.push_back(rdB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(ruT);
  vertices.push_back(ruB);
  vertices.push_back(rdB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  //SOUTH SIDE
  vertices.push_back(ldT);
  vertices.push_back(rdT);
  vertices.push_back(ldB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  vertices.push_back(rdB);
  vertices.push_back(rdT);
  vertices.push_back(ldB);
  triangles.push_back(fcl::Triangle(i,i+1,i+2));i=i+3;

  return std::make_pair( vertices, triangles );

}
#endif

};

