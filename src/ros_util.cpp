#include "ros_util.h"
uint TriangleObject::mesh_counter=0;
RVIZInterface *TriangleObject::rviz = NULL;
FCLInterface *TriangleObject::fcl = NULL;

TriangleObject::TriangleObject(char *tris_file_name, double x, double y, double z){
	this->tris_file_name = std::string(tris_file_name);
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
	id = hashit(tris_file_name);

	this->read_tris_to_marker( marker, tris_file_name );
	this->init_marker_default();
	this->updatePosition(x,y,z);

	fcl->tris_to_BVH(bvh, this->tris_file_name.c_str() );
}
void TriangleObject::updatePosition(double x, double y, double z){
	//Update RVIZ position marker
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	//Update FCL BVH model
}
void TriangleObject::init_marker_default(){
	uint32_t shape = visualization_msgs::Marker::TRIANGLE_LIST;

	marker.ns = tris_file_name;
	marker.id = id;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// 1x1x1 => 1m
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
}
void TriangleObject::rviz_publish(){
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	marker.lifetime = ros::Duration();
	this->rviz->publish(marker);
}

double TriangleObject::distance_to(TriangleObject &rhs){
	//fcl->load_collision_pair(this->tris_file_name.c_str() , rhs.tris_file_name.c_str());
	
	const TriangleObject lhs = *this;
	fcl::Matrix3f r1 (1,0,0,
			0,1,0,
			0,0,1);
	//ROS_INFO("LHS: x=%f, y=%f, z=%f\n", lhs.x, lhs.y, lhs.z);
	//ROS_INFO("RHS: x=%f, y=%f, z=%f\n", rhs.x, rhs.y, rhs.z);
	fcl::Vec3f d1(lhs.x,lhs.y,lhs.z);
	fcl::Vec3f d2(rhs.x,rhs.y,rhs.z);

	fcl::Transform3f Tlhs(r1, d1);
	fcl::Transform3f Trhs(r1, d2);

	fcl::DistanceRequest request;
	fcl::DistanceResult result;
	double d = fcl::distance (&lhs.bvh, Tlhs, &rhs.bvh, Trhs, request, result);
	return d;
}
void TriangleObject::read_tris_to_marker(visualization_msgs::Marker &marker, char *fname){
	
	int ntris;
	FILE *fp = fopen_s(fname,"r");

	int res=fscanf(fp, "%d", &ntris);

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

