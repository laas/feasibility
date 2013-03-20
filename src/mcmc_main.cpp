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
		uint Nsamples = 6;
		std_seed();
		std::string chair_file = get_chair_str();
		std::string robot_file = get_robot_str(argv[1]);

		//Logger log(get_logging_str("data/hmc/", robot_file));
		Logger log("test_log4");
		TriangleObject chair(chair_file.c_str(), 2, 1, 0);
		TriangleObject robot(robot_file.c_str(), 0, 0, 0);

		SampleGenerator sampler(log);
		sampler.mcmc( robot, chair, Nsamples);
	}
}
