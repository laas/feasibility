#include "rviz/visualmarker.h"

namespace ros{
	PrimitiveMarkerBox::PrimitiveMarkerBox(double x, double y, double w, double l, double h): PrimitiveMarkerTriangle() {
		this->g.x = x;
		this->g.y = y;
		this->g.z = 0.0;
		this->g.setRPYRadian(0,0,0);
		this->g.sx=1;
		this->g.sy=1;
		this->g.sz=1;

		this->w = w;
		this->l = l;
		this->h = h;

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

		//xlu - x left up, xrd - x right down
		double xlu = w/2;
		double ylu = l/2;
		double zlu = 0;

		double xru = w/2;
		double yru = -l/2;
		double zru = 0;

		double xld = -w/2;
		double yld = l/2;
		double zld = 0;

		double xrd = -w/2;
		double yrd = -l/2;
		double zrd = 0;

		fcl::Vec3f luB(xlu, ylu, zlu);
		fcl::Vec3f ruB(xru, yru, zru);
		fcl::Vec3f ldB(xld, yld, zld);
		fcl::Vec3f rdB(xrd, yrd, zrd);
		ROS_INFO("x %f, y %f, w %f, l %f", xlu, ylu, w, l);

		double xlut = w/2;
		double ylut = l/2;
		double zlut = h;

		double xrut = w/2;
		double yrut = -l/2;
		double zrut = h;

		double xldt = -w/2;
		double yldt = l/2;
		double zldt = h;

		double xrdt = -w/2;
		double yrdt = -l/2;
		double zrdt = h;

		fcl::Vec3f luT(xlut, ylut, zlut);
		fcl::Vec3f ruT(xrut, yrut, zrut);
		fcl::Vec3f ldT(xldt, yldt, zldt);
		fcl::Vec3f rdT(xrdt, yrdt, zrdt);
		uint i = 0;

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

