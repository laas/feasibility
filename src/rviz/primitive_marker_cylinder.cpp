#include "rviz/visualmarker.h"

namespace ros{
	void PrimitiveMarkerCylinder::reloadCylinderBVH(double radius, double height){
		delete this->bvh;
		this->bvh = NULL;
		this->bvh = new fcl::BVHModel< BoundingVolume >();
		uint N=20;
		std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > vt = this->getVerticesAndTriangles();
		this->primitiveMarker2BVH(this->bvh, vt);
	}
	PrimitiveMarkerCylinder::PrimitiveMarkerCylinder(double x, double y, double r, double h): PrimitiveMarkerTriangle() {
		this->g.x = x;
		this->g.y = y;
		this->g.z = 0.0;
		this->g.setRPYRadian(0,0,0);
		this->g.sx=1;
		this->g.sy=1;
		this->g.sz=1;
		this->g.z = 0.0;
		this->g.radius = r;
		this->g.height = h;
		height=h;
		radius=r;
		Ntriangles = 20;

		initPrimitiveMarker(this);
	}
	std::string PrimitiveMarkerCylinder::name(){
		return std::string("cylinder_triangles");
	}
#ifdef FCL_COLLISION_CHECKING
	std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > PrimitiveMarkerCylinder::getVerticesAndTriangles(){

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;

		//ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", bvh->num_tris, bvh->num_vertices);
		uint Nvertices = Ntriangles+Ntriangles+2*Ntriangles; //top,bottom and connect the points by double triangles

		uint i=0;//counter of vertices
		for(double h=0;h<=height;h+=height){
			double oldx = radius;
			double oldy = 0.0;
			for(double t=0;t<=2*M_PI;t+=2*M_PI/Ntriangles){
				
				//first vertex at middle point
				fcl::Vec3f a(0, 0, h);
				//second vertex at old pos
				fcl::Vec3f b(oldx, oldy, h);
				//third vertex at next pos
				//t= 2*M_PI/(N-i-1);
				double newx = cos(t)*radius;
				double newy = sin(t)*radius;
				fcl::Vec3f c(newx, newy, h);

				vertices.push_back(a);
				vertices.push_back(b);
				vertices.push_back(c);

				fcl::Triangle t(i,i+1,i+2);
				triangles.push_back(t);
				i=i+3;

				oldx=newx;
				oldy=newy;
			}
		}


		double oldx = radius;
		double oldy = 0.0;
		for(double t=0;t<=2*M_PI;t+=2*M_PI/Ntriangles){
			double newx = cos(t)*radius;
			double newy = sin(t)*radius;

			fcl::Vec3f a(oldx, oldy, 0);
			fcl::Vec3f b(oldx, oldy, height);
			fcl::Vec3f c(newx, newy, 0);

			vertices.push_back(a);
			vertices.push_back(b);
			vertices.push_back(c);

			fcl::Triangle t(i,i+1,i+2);
			triangles.push_back(t);
			i=i+3;

			fcl::Vec3f d(newx, newy, 0);
			fcl::Vec3f e(oldx, oldy, height);
			fcl::Vec3f f(newx, newy, height);

			vertices.push_back(d);
			vertices.push_back(e);
			vertices.push_back(f);

			fcl::Triangle t2(i,i+1,i+2);
			triangles.push_back(t2);
			i=i+3;

			oldx = newx;
			oldy = newy;
		}
		return std::make_pair( vertices, triangles );

	}
#endif

};
