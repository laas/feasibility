#include "rviz/rviz_visualmarker.h"

namespace ros{
	void CylinderMarkerTriangles::reloadCylinderBVH(double radius, double height){
		delete this->bvh;
		this->bvh = NULL;
		this->bvh = new fcl::BVHModel< BoundingVolume >();
		uint N=20;
		this->cylinder2BVH(this->bvh, N, radius, height);
	}
	CylinderMarkerTriangles::CylinderMarkerTriangles(double x, double y, double r, double h): TriangleObject() {
		this->g.x = x;
		this->g.y = y;
		this->g.z = 0.0;
		this->g.setYawRadian(0);
		this->g.sx=1;
		this->g.sy=1;
		this->g.sz=1;
		this->g.z = 0.0;
		this->g.radius = r;
		this->g.height = h;
		uint N=20; //number of triangles = 4*N -> we simulate a 80 triangles object

		this->bvh = new fcl::BVHModel< BoundingVolume >();
		cylinder2BVH(this->bvh, N, r, h);
		this->pqp_model = new PQP_Model;
		cylinder2PQP(this->pqp_model, N, r, h);
		cylinder2marker(this->marker, N, r, h);
		init_marker();
	}
	std::string CylinderMarkerTriangles::name(){
		return std::string("cylinder_triangles");
	}
	uint32_t CylinderMarkerTriangles::get_shape(){
		return visualization_msgs::Marker::TRIANGLE_LIST;
	}
	Color CylinderMarkerTriangles::get_color(){
		return ros::Color(1,0,0,0.3);
	}
#ifdef FCL_COLLISION_CHECKING
	std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > CylinderMarkerTriangles::getCylinderVerticesAndTriangles(uint N, double radius, double height){

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;

		//ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", bvh->num_tris, bvh->num_vertices);
		uint Nvertices = N+N+2*N; //top,bottom and connect the points by double triangles

		uint i=0;//counter of vertices
		for(double h=0;h<=height;h+=height){
			double oldx = radius;
			double oldy = 0.0;
			for(double t=0;t<=2*M_PI;t+=2*M_PI/N){
				
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
		for(double t=0;t<=2*M_PI;t+=2*M_PI/N){
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
	void CylinderMarkerTriangles::cylinder2PQP(PQP_Model *m, uint N, double radius, double height){
		std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > vt;
		vt = getCylinderVerticesAndTriangles(N, radius, height);

		std::vector<fcl::Vec3f> vertices = vt.first;
		std::vector<fcl::Triangle> triangles = vt.second;

		m->BeginModel();
			
		uint counter=0;
		for (uint i = 0; i < vertices.size(); i+=3){
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = vertices.at(i)[0];
			p1[1] = vertices.at(i)[1];
			p1[2] = vertices.at(i)[2];

			p2[0] = vertices.at(i+1)[0];
			p2[1] = vertices.at(i+1)[1];
			p2[2] = vertices.at(i+1)[2];

			p3[0] = vertices.at(i+2)[0];
			p3[1] = vertices.at(i+2)[1];
			p3[2] = vertices.at(i+2)[2];

			m->AddTri(p1,p2,p3,counter);
			counter++;
		}
		m->EndModel();

	}
	void CylinderMarkerTriangles::cylinder2BVH(fcl::BVHModel< BoundingVolume > *m, uint N, double radius, double height){
		std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > vt;
		vt = getCylinderVerticesAndTriangles(N, radius, height);

		std::vector<fcl::Vec3f> vertices = vt.first;
		std::vector<fcl::Triangle> triangles = vt.second;

		bvh->bv_splitter.reset (new fcl::BVSplitter<BoundingVolume>(fcl::SPLIT_METHOD_MEAN));
		bvh->beginModel();
		bvh->addSubModel(vertices, triangles);
		bvh->endModel();
	}
	void CylinderMarkerTriangles::cylinder2marker(visualization_msgs::Marker &marker, uint N, double radius, double height){
		
		std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > vt;
		vt = getCylinderVerticesAndTriangles(N, radius, height);

		std::vector<fcl::Vec3f> vertices = vt.first;
		std::vector<fcl::Triangle> triangles = vt.second;

		Color cc = get_color();
		std_msgs::ColorRGBA c;
		c.r = cc.r;
		c.g = cc.g;
		c.b = cc.b;
		c.a = cc.a;

		for (uint i = 0; i < vertices.size(); i++){
			geometry_msgs::Point p;
			p.x = vertices.at(i)[0];
			p.y = vertices.at(i)[1];
			p.z = vertices.at(i)[2];
			marker.points.push_back(p);
			marker.colors.push_back(c);
		}
	}
#endif

};
