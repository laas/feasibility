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
#include "sampler.h"

double draw_gaussian_proposal_dist(double x){
	double stddev = 1.2;
	return randn(x, stddev);
}
static int accepted_samples = 0;
static int rejected_samples = 0;
const uint Nsamples = 10000;
void accept_sample(double x, double y, double t, TriangleObject &robot, TriangleObject &chair){
	chair.update_position(x,y,t);
	accepted_samples++;
	//chair.rviz_publish();
	//robot.rviz_publish();
}
double p(double x, double y, double t, TriangleObject &robot, TriangleObject &chair){
	chair.update_position(x,y,t);

	//return unnormalized distribution
	double d=chair.distance_to(robot);
	double d2=robot.distance_to(chair);

	double p = normpdf(d, 1.0, 0.17);
	ROS_INFO("POS: [accept rate %f] %f %f %f -- distance %f,%f -> %f", (double)accepted_samples/(double)(accepted_samples+rejected_samples+1.0), x, y, t, d, d2, p);

	return p;
}
int main( int argc, char** argv )
{
	ros::init(argc, argv, "mcmc_sampler");
	ros::NodeHandle n;
	ros::Rate r(1);

	if(argc!=2){
		printf("usage: mcmc_sampler <SweptVolumeFileName>\n");
		return -1;
	}
	printf("%s\n", argv[1]);
	if (ros::ok())
	{
		std_seed();
		std::string chair_file = get_chair_str();
		std::string robot_file = get_robot_str(argv[1]);

		Logger log(get_logging_str("data/hmc/", robot_file));
		TriangleObject chair(chair_file.c_str(), 2, 1, 0);
		TriangleObject robot(robot_file.c_str(), 0, 0, 0);

		SampleGenerator sampler(log);
		sampler.mcmc( robot, chair, Nsamples);
		//for(uint i=0;i<Nsamples;i++){

		//upper lower bound on robot workspace
		/*
		int lInt = 1.0;
		int hInt = 1.4;

		double x= rand(lInt, hInt);
		double y= rand(lInt, hInt);
		double t= rand(lInt, hInt);
		//double x= 0.0;
		//double y= 0.0;

		//proposal distribution
		double x_old = x;
		double y_old = y;
		double t_old = t;
		double p_old = p(x_old, y_old, t_old, robot, chair);
		for(uint i=0;i<Nsamples;i++){
			double x_cand = draw_gaussian_proposal_dist(x);
			double y_cand = draw_gaussian_proposal_dist(y);
			double t_cand = draw_gaussian_proposal_dist(t);
			double p_cur = p(x_cand, y_cand, t_cand, robot, chair);

			double a = p_cur / p_old;

			if(a>=1){
				x_old = x_cand;
				y_old = y_cand;
				t_old = t_cand;
				accept_sample(x_old, y_old, t_old, robot, chair);
				log("%f %f %f %f", x_old, y_old, t_old, chair.distance_to(robot));
			}else{
				double u = rand(0,1);
				if(u<a){
					x_old = x_cand;
					y_old = y_cand;
					t_old = t_cand;
					accept_sample(x_old, y_old, t_old, robot, chair);
					log("%f %f %f %f", x_old, y_old, t_old, chair.distance_to(robot));
				}else{
					x_old = x_old;
					y_old = y_old;
					t_old = t_old;
					rejected_samples++;
				}
			}
			p_old = p_cur;

			r.sleep();
		}
		ROS_INFO("accepted samples %d/%d -- %f", accepted_samples, Nsamples, accepted_samples/(double)Nsamples);
		*/
	}
}
