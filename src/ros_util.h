#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <string>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include "util.h"

struct FCLInterface;

struct RVIZInterface{
private:
	ros::Publisher publisher;
	ros::NodeHandle n;

public:
	RVIZInterface(){
		char *topic_name = "visualization_marker";
		publisher = n.advertise<visualization_msgs::Marker>(topic_name, 1);
	}
	void publish(visualization_msgs::Marker &m){
		publisher.publish(m);
	}
};

typedef fcl::AABB BoundingVolume;
struct TriangleObject{
private:

	std::string tris_file_name;
	double x,y,z; //position in the global world frame
	uint id; //unique id for this object
	static RVIZInterface *rviz; //singletons
	static FCLInterface *fcl;
	static uint mesh_counter;
	fcl::BVHModel< BoundingVolume > bvh;
	visualization_msgs::Marker marker;

public:
	TriangleObject(char *tris_file_name, double x, double y, double z);
	void updatePosition(double x, double y, double z);
	void rviz_publish();
	double distance_to(TriangleObject &rhs);
	void init_marker_default();

	void getFCL_TMatrix(fcl::Transform3f &f);
	template <class T>
	void getFCL_BVHModel(fcl::BVHModel<T> &b);
	void read_tris_to_marker(visualization_msgs::Marker &marker, char *fname);
};

struct FCLInterface{
	//typedef fcl::AABB BoundingVolume;
	//fcl::BVHModel< BoundingVolume > c1;
	//fcl::BVHModel< BoundingVolume > c2;
 
 	FCLInterface(){
	}

	/*
	void conduct_collision_analysis(){
		using namespace fcl;
		CollisionResult result;
		CollisionRequest request;

		boost::shared_ptr<fcl::CollisionGeometry> b1( &c1);
		fcl::CollisionObject co1(b1);
		boost::shared_ptr<fcl::CollisionGeometry> b2( &c2);
		fcl::CollisionObject co2(b2);

		collide( &co1, &co2, request, result);
		size_t contacts = result.numContacts();
		bool coll = result.isCollision();
		ROS_INFO("Collision: %d (contacts: %d)\n", coll, contacts);
		ROS_INFO("Finalized!");
	}
	double conduct_distance_analysis(fcl::BVHModel< BoundingVolume > &c1, fcl::BVHModel< BoundingVolume > &c2){
		
		using namespace fcl;
	
		fcl::Matrix3f r1 (1,0,0,
				0,1,0,
				0,0,1);
		fcl::Vec3f d1(0,0,0);
		fcl::Transform3f t1(r1, d1);

		Transform3f t2;
		t2.setIdentity();

		DistanceRequest request;
		DistanceResult result;
		double d = fcl::distance (&c1, t1, &c2, t2, request, result);
		//double d = fcl::distance (&co1, &co2, request, result);

		//bool coll = result.isCollision();
		//ROS_INFO("Collision: %d (contacts: %d)\n", coll, contacts);
		ROS_INFO("Distance: %f\n", d);
		return d;
	}
	*/

	/*
	void load_collision_pair(const char *c1name, const char *c2name){
		tris_to_BVH(c1, c1name);
		tris_to_BVH(c2, c2name);
	}
	*/

	void tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname ){
		
		int res;
		int ntris;
		FILE *fp = fopen_s(fname,"r");                                                                                                                               
		res=fscanf(fp, "%d", &ntris);

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		for (int i = 0; i < 3*ntris; i+=3){
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			
			fcl::Vec3f a(p1x, p1y, p1z);
			fcl::Vec3f b(p2x, p2y, p2z);
			fcl::Vec3f c(p3x, p3y, p3z);
			vertices.push_back(a);
			vertices.push_back(b);
			vertices.push_back(c);

			fcl::Triangle t(i,i+1,i+2);
			triangles.push_back(t);

			//return;
		}
	  //m.bv_splitter.reset (new fcl::BVSplitter<fcl::OBB>(fcl::SPLIT_METHOD_MEAN));
		m.beginModel();
		m.addSubModel(vertices, triangles);
		m.endModel();
		ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", m.num_tris, m.num_vertices);
		fclose(fp);
	}
};
