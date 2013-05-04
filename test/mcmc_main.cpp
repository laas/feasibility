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

#include "rviz/rviz_visualmarker.h"
#include "util.h"
#include "sampler/sampler.h"

using namespace ros;
int main( int argc, char** argv )
{

	if(argc!=5){
		printf("usage: mcmc_sampler <SweptVolumeFileName> <Height> <DistanceToBoundary> <Nsamples>\n");
		return -1;
	}
	double h=atof(argv[2]);
	double m=atof(argv[3]);
	std::string robot_file = get_tris_str(argv[1]);


	char pname[100];
	sprintf(pname, "mcmc_sampler_h_%.2f_m_%.2f_%s", h, m, robot_file.c_str());

	ros::init(argc, argv, pname);
	ros::NodeHandle n;
	ros::Rate r(1);
	ROS_INFO("%s", argv[1]);
	char command[100];
	char folder[80];
	if (ros::ok())
	{
		uint Nsamples = atoi(argv[4]);
		std_seed();

		sprintf(folder, "data/cylinder/h_%.2f_m_%.2f/", h, m);
		sprintf(command, "mkdir -p %s", folder);
		system(command);

		Logger log(get_logging_str(folder, robot_file));

		SamplingInterface sampler(log);
		sampler.init( new SamplingCTOCylinder(argv[1], h, m));
		sampler.mcmc(Nsamples);
		//sampler.uniform(Nsamples);

	}
}
