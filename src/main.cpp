#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector> //std::vector
#include <iostream> //cout

#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>

#include <fast-replanning/fast-replanning-interface.hh>
//bounding vertex hierarchy

#include "ros_util.h"
#include "util.h"

double draw_gaussian_proposal_dist(double x){
	double stddev = 0.3;
	return randn(x, stddev);
}
void accept_sample(double x, double y, TriangleObject &robot, TriangleObject &chair){
	chair.update_position(x,y,0);
	chair.rviz_publish();
	robot.rviz_publish();
}
double p(double x, double y, TriangleObject &robot, TriangleObject &chair){
	chair.update_position(x,y,0);

	//return unnormalized distribution
	double d=chair.distance_to(robot);
	double displacement = 0.2*sqrtf((chair.x - robot.x)*(chair.x-robot.x) + (chair.y-robot.y)*(chair.y-robot.y));

	double lambda = 5.0;
	double lambda2 = 5.0;

	double p = exp(-lambda * d*d) + exp( -lambda * displacement*displacement);

	//redefined function
	if( d>0 ){
		p=exp(-lambda* d*d);
	}else{
		p=1.0 - exp( -lambda2 * displacement*displacement);
	}


	//ROS_INFO("POS: %f %f %f -- distance %f displacement %f -> %f", x, y, 0.0, d, displacement, p);
	return p;
}
int main( int argc, char** argv )
{
	ros::init(argc, argv, "main_project");
	ros::NodeHandle n;
	ros::Rate r(1);
	//Logger log("mcmc.tmp");

	std::string prefix = get_data_path();
	ROS_INFO("PREFIX %s", prefix.c_str());

	printf("%s", prefix.c_str());
	//fastreplanning::FastReplanningInterface *planner = fastreplanning::fastReplanningInterfaceFactory(path, argc, argv);
	//planner->mainLoop();

	char chair_file[200];
	sprintf(chair_file, "%s%s", prefix.c_str(), "chairLabo.tris");
	//std::string chair_file = "/home/aorthey/git/fastReplanningData/data/chairLabo.tris";
	ROS_INFO("%s", chair_file);
	//std::string robot_file = "fullbody_-14_-21_-29.tris";
	//TriangleObject robot(robot_file.c_str(), 0, 0, 0);
	TriangleObject chair(chair_file, 0, 0, 0);
	while (ros::ok())
	{

		chair.rviz_publish();
		//robot.rviz_publish();
		//for(uint i=0;i<Nsamples;i++){
		r.sleep();
	}
}
