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
		std::string topic_name = "visualization_marker";
		publisher = n.advertise<visualization_msgs::Marker>(topic_name.c_str(), 1);
	}
	void publish(visualization_msgs::Marker &m){
		publisher.publish(m);
	}
};

struct RVIZVisualMarker{
protected:
	uint id; //unique id for this object
	static RVIZInterface *rviz; 
	visualization_msgs::Marker marker;
public:
	double x,y,theta;
public:
	void rviz_publish();
	void init_marker_default(double x, double y, double theta);
};

struct SphereMarker{
	uint id; //unique id for this object
	static RVIZInterface *rviz; 
	visualization_msgs::Marker marker;
public:
	double x,y,r;
public:
	explicit SphereMarker(int id, double x, double y, double r);
	~SphereMarker();
	void update_position(double x, double y);
	void rviz_publish();
	void init_marker_default(double x, double y, double r);
};

struct FootStepObject{
	uint id; //unique id for this object
	static RVIZInterface *rviz; 
	visualization_msgs::Marker marker;
public:
	double x,y,theta; //position in the global world frame
public:
	explicit FootStepObject(int id, double x, double y, double theta);
	~FootStepObject();
	void update_position(double x, double y, double theta);
	void rviz_publish();
	void init_marker_default(double x, double y, double theta);
};

typedef fcl::AABB BoundingVolume;
struct TriangleObject{
private:

	std::string tris_file_name;
	uint id; //unique id for this object
	static RVIZInterface *rviz; //singletons
	static FCLInterface *fcl;
	static uint mesh_counter;
	fcl::BVHModel< BoundingVolume > bvh;
	visualization_msgs::Marker marker;
public:
	double x,y,z; //position in the global world frame

public:
	explicit TriangleObject(std::string tris_file_name, double x, double y, double z);
	~TriangleObject();
	void update_position(double x, double y, double z);
	void rviz_publish();

	double distance_to(TriangleObject &rhs);
	double distance_to2(TriangleObject &rhs);
	void init_marker_default(double x, double y, double z);

	void getFCL_TMatrix(fcl::Transform3f &f);
	template <class T>
	void getFCL_BVHModel(fcl::BVHModel<T> &b);
	void read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname);
};

struct FCLInterface{
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
	*/

	void update_bvh_model(fcl::BVHModel< BoundingVolume > &m, double x, double y, double z){
		
		//update vertices
		std::vector<fcl::Vec3f> vertices;
		uint N = m.num_vertices;
	
		m.beginUpdateModel();
		//Vec3f *vertexP = m.prev_vertices;

		for (uint i=0; i<N; i++){
			fcl::Vec3f v(x,y,z);
			m.updateVertex( v );
		}
		m.endUpdateModel();
	}
	void tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname ){
		
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

			//return;
		}
	  //m.bv_splitter.reset (new fcl::BVSplitter<fcl::OBB>(fcl::SPLIT_METHOD_MEAN));
		m.beginModel();
		m.addSubModel(vertices, triangles);
		m.endModel();
		//ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", m.num_tris, m.num_vertices);
		fclose(fp);
	}
};
