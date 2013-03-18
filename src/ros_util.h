#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <utility> //std::pair
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

const char *FRAME_NAME = "/base_link";
static const double ROS_DURATION = 1;

struct RVIZInterface{
private:
	ros::Publisher publisher;
	ros::NodeHandle n;

	typedef std::pair< std::string, uint> MarkerIdentifier;
	typedef std::vector< MarkerIdentifier > MarkerIdentifierVector;
	MarkerIdentifierVector published_marker;


public:
	RVIZInterface(){
		std::string topic_name = "visualization_marker";
		publisher = n.advertise<visualization_msgs::Marker>(topic_name.c_str(), 1);
	}
	void reset(){
		MarkerIdentifierVector::iterator it;
		printf("reset %d marker\n", published_marker.size());
		ROS_INFO("marker contains %d footsteps", published_marker.size());
		for(it = published_marker.begin(); it != published_marker.end(); it++){
			visualization_msgs::Marker tmp;
			MarkerIdentifier m = (*it);
			uint32_t shape = visualization_msgs::Marker::CUBE;
			tmp.type = shape;
			tmp.ns = m.first;
			tmp.id = m.second;
			tmp.color.r = 0.0f;
			tmp.color.g = 0.0f;
			tmp.color.b = 1.0f;
			tmp.color.a = 1.0;
			tmp.action = visualization_msgs::Marker::DELETE;
			tmp.header.frame_id = FRAME_NAME;
			tmp.header.stamp = ros::Time::now();
			tmp.lifetime = ros::Duration(1);
			publisher.publish(tmp);
			ROS_INFO("deleted marker %s", m.first.c_str());
		}
		published_marker.clear();
	}
	void publish(visualization_msgs::Marker &m){
		publisher.publish(m);
	}
	void footstep_publish(visualization_msgs::Marker &m){
		publisher.publish(m);
		MarkerIdentifier cur_m;
		cur_m = std::make_pair( std::string(m.ns), m.id );
		this->published_marker.push_back(cur_m);
		ROS_INFO("marker contains %d footsteps", published_marker.size());
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
public:
	static RVIZInterface *rviz; 
	visualization_msgs::Marker marker;
	uint id; //unique id for this object
	double x,y,theta; //position in the global world frame

public:
	explicit FootStepObject(int id, double x, double y, double theta);
	~FootStepObject();
	void update_position(double x, double y, double theta);
	void reset();
	void rviz_publish();
	void clean();
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
