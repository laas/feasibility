#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "ros_util.h"

namespace ros{
	uint TriangleObject::mesh_counter=0;
	RVIZInterface *RVIZVisualMarker::rviz = NULL;
	uint RVIZVisualMarker::global_id = 0;

	double TriangleObject::distance_to(TriangleObject &rhs){
		//rotation z-axis (as visualized in rviz)
		
		double t = g.tz;
		double tr = rhs.g.tz;
		fcl::Matrix3f r1 (cos(t),-sin(t),0,
				  sin(t),cos(t) ,0,
				  0     ,0      ,1);
		fcl::Matrix3f r2 (cos(tr),-sin(tr),0,
				  sin(tr),cos(tr) ,0,
				  0     ,0      ,1);

		fcl::Vec3f d1(g.x,g.y,g.z);
		fcl::Vec3f d2(rhs.g.x,rhs.g.y,rhs.g.z);

		fcl::Transform3f Tlhs(r1, d1);
		fcl::Transform3f Trhs(r2, d2);

		fcl::DistanceRequest request;
		fcl::DistanceResult result;
		double d = fcl::distance (&this->bvh, Tlhs, &rhs.bvh, Trhs, request, result);
		//double md = result.penetration_depth;
		ROS_INFO("distance: %f", d);
		//result.clear();

		return d;
	}
	void TriangleObject::read_tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname ){
		
		int ntris;
		FILE *fp = fopen_s(fname,"r");
		int res=fscanf(fp, "%d", &ntris);
		CHECK(res==1, "fscanf failed");

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		for (int i = 0; i < 3*ntris; i+=3){
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			CHECK(res==9, "fscanf failed");
			
			fcl::Vec3f a(p1x, p1y, p1z);
			fcl::Vec3f b(p2x, p2y, p2z);
			fcl::Vec3f c(p3x, p3y, p3z);
			vertices.push_back(a);
			vertices.push_back(b);
			vertices.push_back(c);

			fcl::Triangle t(i,i+1,i+2);
			triangles.push_back(t);
		}
		fclose(fp);

		bvh.bv_splitter.reset (new fcl::BVSplitter<BoundingVolume>(fcl::SPLIT_METHOD_MEAN));
		bvh.beginModel();
		bvh.addSubModel(vertices, triangles);
		bvh.endModel();
		ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", bvh.num_tris, bvh.num_vertices);
	}

	void TriangleObject::read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname){
		
		int ntris;
		FILE *fp = fopen_s(fname,"r");

		int res=fscanf(fp, "%d", &ntris);
		CHECK(res==1, "fscanf failed");


		Color cc = get_color();
		std_msgs::ColorRGBA c;
		c.r = cc.r;
		c.g = cc.g;
		c.b = cc.b;
		c.a = cc.a;

		for (int i = 0; i < 3*ntris; i+=3){
			geometry_msgs::Point p;
			geometry_msgs::Point p1;
			geometry_msgs::Point p2;
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			
			double sX = 1.0;
			double sY = 1.0;
			double sZ = 1.0;
			p.x = sX*p1x;p.y = sY*p1y;p.z = sZ*p1z;
			p1.x = sX*p2x;p1.y = sY*p2y;p1.z = sZ*p2z;
			p2.x = sX*p3x;p2.y = sY*p3y;p2.z = sZ*p3z;
			marker.points.push_back(p);
			marker.points.push_back(p1);
			marker.points.push_back(p2);
			marker.colors.push_back(c);
			marker.colors.push_back(c);
			marker.colors.push_back(c);
		}
		fclose(fp);
	}
	void RVIZVisualMarker::init_marker(){
		char fname[50];
		std::string name = this->name();
		sprintf(fname, "%d_%s",this->id, name.c_str());

		marker.ns = fname;
		marker.id = this->id;
		marker.type = get_shape();
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = g.x;
		marker.pose.position.y = g.y;
		marker.pose.position.z = g.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = g.tz;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = g.sx;
		marker.scale.y = g.sy;
		marker.scale.z = g.sz;

		Color c = get_color();
		marker.color.r = c.r;
		marker.color.g = c.g;
		marker.color.b = c.b;
		marker.color.a = c.a;
	}
	void RVIZVisualMarker::drawLine(double x_in, double y_in){

		visualization_msgs::Marker line;
		uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

		char fname[50];
		sprintf(fname, "%d_line",this->id);
		global_id++;

		line.ns = fname;
		line.id = this->id;
		line.type = shape;
		line.action = visualization_msgs::Marker::ADD;

		line.scale.x = 0.005;
		geometry_msgs::Point p;
		p.x = g.x;
		p.y = g.y;
		p.z = g.z;

		geometry_msgs::Point p2;
		p2.x = x_in;
		p2.y = y_in;
		p2.z = 0.0;

		line.points.push_back(p);
		line.points.push_back(p2);

		line.color.r = 1.0f;
		line.color.g = 0.5f;
		line.color.b = 0.0f;
		line.color.a = 1.0f;

		line.header.frame_id = FRAME_NAME;
		line.header.stamp = ros::Time::now();
		line.lifetime = ros::Duration();
		this->rviz->publish(line);
	}
};
