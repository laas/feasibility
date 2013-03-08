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
	void changeColor(double r, double g, double b);
	void drawLine(double x_in, double y_in);
};

typedef fcl::AABB BoundingVolume;
struct TriangleObject{
private:

	std::string tris_file_name;
	uint id; //unique id for this object
	static RVIZInterface *rviz; //singletons
	static uint mesh_counter;
	fcl::BVHModel< BoundingVolume > bvh;
	visualization_msgs::Marker marker;
public:
	double x,y,t; //position in the global world frame

public:
	explicit TriangleObject(std::string tris_file_name, double x, double y, double t);
	~TriangleObject();
	void clean();
	void update_position(double x, double y, double t);
	void rviz_publish();

	double distance_to(TriangleObject &rhs);
	void init_marker_default(double x, double y, double t);

	void read_tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname );
	void read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname);
};
