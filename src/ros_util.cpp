#include "ros_util.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
/*
#include <boost/timer.hpp>
#include "fcl_resources/config.h"
#include <boost/filesystem.hpp>
*/

uint TriangleObject::mesh_counter=0;
RVIZInterface *TriangleObject::rviz = NULL;
FCLInterface *TriangleObject::fcl = NULL;

TriangleObject::TriangleObject(std::string tris_file_name, double x, double y, double z){
	this->tris_file_name = (tris_file_name);
	this->x=x;
	this->y=y;
	this->z=z;
	//ROS_INFO("x=%f, y=%f, z=%f\n", x, y, z);

	if(rviz == NULL){
		rviz = new RVIZInterface();
		mesh_counter=0;
	}

	if(fcl == NULL){
		fcl = new FCLInterface();
	}

	//id = mesh_counter++;
	id = hashit(tris_file_name.c_str());
	this->read_tris_to_marker( marker, tris_file_name.c_str() );
	this->init_marker_default(x,y,z);

	fcl->tris_to_BVH(bvh, this->tris_file_name.c_str() );
	ROS_INFO("Created new tris object");
}
TriangleObject::~TriangleObject(){
	if(fcl!=NULL){
		delete fcl;
		fcl = NULL;
	}
	if(rviz!=NULL){
		delete rviz;
		rviz = NULL;
	}
}
void TriangleObject::update_position(double x, double y, double z){
	//Update RVIZ position marker
	this->marker.pose.position.x = x;
	this->marker.pose.position.y = y;
	this->marker.pose.position.z = z;
	this->x=x;
	this->y=y;
	this->z=z;
	//Update FCL BVH model
	//
	//possibilities: (1) map(vertices, x,y,z)
	//fcl->update_bvh_model(bvh, x, y, z);

}
void TriangleObject::init_marker_default(double x=0, double y=0, double z=0){
	uint32_t shape = visualization_msgs::Marker::TRIANGLE_LIST;

	marker.ns = tris_file_name;
	marker.id = id;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// 1x1x1 => 1m
	marker.scale.x = 2.0;
	marker.scale.y = 2.0;
	marker.scale.z = 2.0;
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

}
void TriangleObject::rviz_publish(){
	this->marker.header.frame_id = "/my_frame";
	this->marker.header.stamp = ros::Time::now();

	this->marker.lifetime = ros::Duration();
	this->rviz->publish(this->marker);
}

double TriangleObject::distance_to(TriangleObject &rhs){
	//fcl->load_collision_pair(this->tris_file_name.c_str() , rhs.tris_file_name.c_str());
	
	fcl::Matrix3f r1 (1,0,0,
			0,1,0,
			0,0,1);
	//ROS_INFO("LHS: x=%f, y=%f, z=%f\n", lhs.x, lhs.y, lhs.z);
	//ROS_INFO("RHS: x=%f, y=%f, z=%f\n", rhs.x, rhs.y, rhs.z);
	fcl::Vec3f d1(this->x,this->y,this->z);
	fcl::Vec3f d2(rhs.x,rhs.y,rhs.z);

	fcl::Transform3f Tlhs(r1, d1);
	fcl::Transform3f Trhs(r1, d2);

	fcl::DistanceRequest request;
	fcl::DistanceResult result;
	//fcl::CollisionRequest request;
	//fcl::CollisionResult result;
	double d = fcl::distance (&this->bvh, Tlhs, &rhs.bvh, Trhs, request, result);
	//double md = result.penetration_depth;
	//ROS_INFO("distance: %f , min_dist: %f", d,md);
	//result.clear();

	return d;
}
double TriangleObject::distance_to2(TriangleObject &rhs){
	//fcl->load_collision_pair(this->tris_file_name.c_str() , rhs.tris_file_name.c_str());
	
	using namespace fcl;
	fcl::Matrix3f r1 (1,0,0,
			0,1,0,
			0,0,1);
	fcl::Vec3f d1(this->x,this->y,this->z);
	fcl::Vec3f d2(rhs.x,rhs.y,rhs.z);
	fcl::Transform3f Tlhs(r1, d1);
	fcl::Transform3f Trhs(r1, d2);

	DistanceResult local_result;
	//MeshDistanceTraversalNode<BoundingVolume> node;
	//if(!initialize(node, (const BVHModel<BoundingVolume>&)this->bvh, Tlhs, &(const BVHModel<BoundingVolume>&)(*(&rhs.bvh)), Trhs, DistanceRequest(true), local_result))

	//node.enable_statistics = true;

	//int qsize = 0;
	//distance(&node, NULL, qsize);

	//double d = local_result.min_distance;
	//return d;
	return -1;

}

void TriangleObject::read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname){
	
	int ntris;
	FILE *fp = fopen_s(fname,"r");

	int res=fscanf(fp, "%d", &ntris);
	CHECK(res==1, "fscanf failed");

	for (int i = 0; i < 3*ntris; i+=3){
		geometry_msgs::Point p;
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
		       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		
		double sX = 1;
		double sY = 1;
		double sZ = 1;
		p.x = sX*p1x;p.y = sY*p1y;p.z = sZ*p1z;
		p1.x = sX*p2x;p1.y = sY*p2y;p1.z = sZ*p2z;
		p2.x = sX*p3x;p2.y = sY*p3y;p2.z = sZ*p3z;
		marker.points.push_back(p);
		marker.points.push_back(p1);
		marker.points.push_back(p2);
		std_msgs::ColorRGBA c;
		c.r = 1;
		c.g = 0;
		c.b = 0;
		marker.colors.push_back(c);
		marker.colors.push_back(c);
		marker.colors.push_back(c);
	}
	fclose(fp);
}
//######################################################
// FootStepObject
//######################################################
RVIZInterface *FootStepObject::rviz = NULL;

FootStepObject::FootStepObject(int id, double x, double y, double theta){
	this->x=x;
	this->y=y;
	this->theta=theta;
	if(rviz == NULL){
		rviz = new RVIZInterface();
	}

	this->id = id;
	this->init_marker_default(x,y,theta);
}
FootStepObject::~FootStepObject(){
	if(rviz!=NULL){
		delete rviz;
		rviz = NULL;
	}
}
void FootStepObject::update_position(double x, double y, double theta){
	//Update RVIZ position marker
	this->marker.pose.position.x = x;
	this->marker.pose.position.y = y;
	this->marker.pose.orientation.x = 0;
	this->marker.pose.orientation.y = 0;
	this->marker.pose.orientation.z = theta;
	this->x=x;
	this->y=y;
	this->theta=theta;
}
void FootStepObject::rviz_publish(){
	this->marker.header.frame_id = "/my_frame";
	this->marker.header.stamp = ros::Time::now();

	//this->marker.lifetime = ros::Duration();
	this->marker.lifetime = ros::Duration(1);
	this->rviz->publish(this->marker);
}

void FootStepObject::init_marker_default(double x=0, double y=0, double theta=0){
	uint32_t shape = visualization_msgs::Marker::CUBE;

	char fname[50];
	sprintf(fname, "%d_footstep",this->id);


	marker.ns = fname;
	marker.id = this->id;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = theta;
	marker.pose.orientation.w = 1.0;

	// 1x1x1 => 1m
	marker.scale.x = 0.2;
	marker.scale.y = 0.10;
	marker.scale.z = 0.01;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

}
//######################################################
// SphereMarker
//######################################################
RVIZInterface *SphereMarker::rviz = NULL;

SphereMarker::SphereMarker(int id, double x, double y, double r){
	this->x=x;
	this->y=y;
	if(rviz == NULL){
		rviz = new RVIZInterface();
	}

	this->id = id;
	this->init_marker_default(x,y,r);
}
SphereMarker::~SphereMarker(){
	if(rviz!=NULL){
		delete rviz;
		rviz = NULL;
	}
}
void SphereMarker::update_position(double x, double y){
	//Update RVIZ position marker
	this->marker.pose.position.x = x;
	this->marker.pose.position.y = y;
	this->x=x;
	this->y=y;
}
void SphereMarker::rviz_publish(){
	this->marker.header.frame_id = "/my_frame";
	this->marker.header.stamp = ros::Time::now();

	this->marker.lifetime = ros::Duration();
	this->rviz->publish(this->marker);
}

void SphereMarker::init_marker_default(double x=0, double y=0, double r=1){
	uint32_t shape = visualization_msgs::Marker::CYLINDER;

	char fname[50];
	sprintf(fname, "%d_footstep",this->id);


	marker.ns = fname;
	marker.id = this->id;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// 1x1x1 => 1m
	marker.scale.x = r;
	marker.scale.y = r;
	marker.scale.z = 0.01;
	marker.color.r = 1.0f;
	marker.color.g = 0.5f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

}
