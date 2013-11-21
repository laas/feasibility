#include "rviz/visualmarker.h"

namespace ros{
PrimitiveMarkerBox::PrimitiveMarkerBox(double x, double y, double w, double l, double h): PrimitiveMarkerTriangle() {
  this->g.x = x;
  this->g.y = y;
  this->g.z = 0.0;
  this->g.setRPYRadian(0,0,0);
  this->g.sx=w;
  this->g.sy=l;
  this->g.sz=h;

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
  double x = this->g.x;
  double y = this->g.y;
  double z = this->g.z;
  double sx = this->g.sx;
  double sy = this->g.sy;
  double sz = this->g.sz;
  double yaw = this->g.getYawRadian();

  //bottom
  uint i = 0;

  //xlu - x left up, xrd - x right down
  double xlu = sx/2;
  double ylu = sy/2;
  double zlu = 0;

  double xru = sx/2;
  double yru = -sy/2;
  double zru = 0;

  double xld = -sx/2;
  double yld = sy/2;
  double zld = 0;

  double xrd = -sx/2;
  double yrd = -sy/2;
  double zrd = 0;

  ROS_INFO("%f %f %f %f", xlu, xru, xld, xrd);

  fcl::Vec3f luB(xlu, ylu, zlu);
  fcl::Vec3f ruB(xru, yru, zru);
  fcl::Vec3f ldB(xld, yld, zld);
  fcl::Vec3f rdB(xrd, yrd, zrd);

  double xlut = sx/2;
  double ylut = sy/2;
  double zlut = sz;

  double xrut = sx/2;
  double yrut = -sy/2;
  double zrut = sz;

  double xldt = -sx/2;
  double yldt = sy/2;
  double zldt = sz;

  double xrdt = -sx/2;
  double yrdt = -sy/2;
  double zrdt = sz;

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

