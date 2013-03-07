#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector> //std::vector
#include <iostream> //cout

#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>
//bounding vertex hierarchy

#include "ros_util.h"
#include "util.h"

double draw_gaussian_proposal_dist(double x){
	double stddev = 0.8;
	return randn(x, stddev);
}
static int accepted_samples = 0;
void accept_sample(double x, double y, TriangleObject &robot, TriangleObject &chair){
	chair.update_position(x,y,0);
	accepted_samples++;
	//chair.rviz_publish();
	//robot.rviz_publish();
}
double p(double x, double y, TriangleObject &robot, TriangleObject &chair){
	chair.update_position(x,y,0);

	//return unnormalized distribution
	double d=chair.distance_to(robot);

	double p = normpdf(d, 1.0, 0.17);
	ROS_INFO("POS: %f %f %f -- distance %f -> %f", x, y, 0.0, d, p);

	return p;
}
int main( int argc, char** argv )
{
	ros::init(argc, argv, "main_project");
	ros::NodeHandle n;
	ros::Rate r(1);
	Logger log("mcmc.tmp");

	std::string chair_file = get_chair_str();
	std::string robot_file = get_robot_str();
	if (ros::ok())
	{
		TriangleObject chair(chair_file.c_str(), 2, 1, 0);
		TriangleObject robot(robot_file.c_str(), 0, 0, 0);

		//for(uint i=0;i<Nsamples;i++){
		uint Nsamples = 10000;

		//upper lower bound on robot workspace
		int lInt = 0.5;
		int hInt = 0.8;

		double x= rand(lInt, hInt);
		double y= rand(lInt, hInt);
		//double x= 0.0;
		//double y= 0.0;

		//proposal distribution
		double x_old = x;
		double y_old = y;
		for(uint i=0;i<Nsamples;i++){
			double x_cand = draw_gaussian_proposal_dist(x);
			double y_cand = draw_gaussian_proposal_dist(y);

			double p_old = p(x_old, y_old, robot, chair);
			double p_cur = p(x_cand, y_cand, robot, chair);

			double a = p_cur / p_old;

			if(a>=1){
				x_old = x_cand;
				y_old = y_cand;
				accept_sample(x_old, y_old, robot, chair);
				log("%f %f %f", x_old, y_old, chair.distance_to(robot));
			}else{
				double u = rand(0,1);
				if(u<a){
					x_old = x_cand;
					y_old = y_cand;
					accept_sample(x_old, y_old, robot, chair);
					log("%f %f %f", x_old, y_old, chair.distance_to(robot));
				}else{
					x_old = x_old;
					y_old = y_old;
				}
			}

			//r.sleep();
		}
		ROS_INFO("accepted samples %d/%d -- %f", accepted_samples, Nsamples, accepted_samples/(double)Nsamples);
		r.sleep();
	}
}
