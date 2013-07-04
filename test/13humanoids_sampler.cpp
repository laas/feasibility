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
#include "util/util.h"
#include "sampler/sampler.h"

using namespace ros;
int main( int argc, char** argv )
{

	if(argc!=4){
		printf("usage: mcmc_sampler <SweptVolumeFileName> <Nsamples> <rmax>\n");
		return -1;
	}
	std::string robot_file = get_tris_str(argv[1]);

	char pname[100];
	sprintf(pname, "humanoids13_sampler_%d", hashit(argv[1]));
	char folder[80];
	sprintf(folder, "data/13humanoids/");

	ros::init(argc, argv, pname);
	ros::NodeHandle n;
	ros::Rate r(1);

	ROS_INFO("%s", pname);

	if (ros::ok())
	{
		uint Nsamples = atoi(argv[2]);
		double rmax = atof(argv[3]);
		printf("input: swept volume: %s, Nsamples: %d\n",argv[1],Nsamples);

		std_seed(); //debug

		system2("mkdir -p %s", folder);

		Logger log(get_logging_str(folder, robot_file));

		SamplingInterface sampler(log);
		sampler.init( new SamplingCTOCylinder(argv[1]) );
		//sampler.mcmc(Nsamples);
		sampler.uniform_normalized(Nsamples);
		sampler.hmc(Nsamples, rmax);

	}
}
