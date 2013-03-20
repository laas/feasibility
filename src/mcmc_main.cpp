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
//#include "sampler.h"

using namespace ros;
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
	ROS_INFO("%s", argv[1]);
	if (ros::ok())
	{
		uint Nsamples = 6;
		std_seed();
		std::string chair_file = get_chair_str();
		std::string robot_file = get_robot_str(argv[1]);

		//Logger log(get_logging_str("data/hmc/", robot_file));
		Geometry chair_pos;
		chair_pos.x = 0.8;
		chair_pos.y = 0.05;
		chair_pos.tz = 0.5;

		Geometry robot_pos;
		robot_pos.x = 0;
		robot_pos.y = 0;
		robot_pos.tz = 0;

		TriangleObject chair(chair_file.c_str(), chair_pos);
		TriangleObject robot(robot_file.c_str(), robot_pos);
		chair.publish();
		robot.publish();
		r.sleep();

		for(uint i=0;i<10;i++){
			LeftFootMarker lfm(0+i/10.0,0,0);
			//RightFootMarker rfm(0+i/10.0,1,0);
			//SphereMarker ss(-1+i/10,0.5,0.2);
			lfm.publish();
			//ss.publish();
		}


		//SampleGenerator sampler(log);
		//sampler.mcmc( robot, chair, Nsamples);
	}
}
