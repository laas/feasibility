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
	sprintf(pname, "sampler_h_%d_m_%d_%d", (int)floor(100*h), (int)floor(100*m), hashit(argv[1]));

	ros::init(argc, argv, pname);
	ros::NodeHandle n;
	ros::Rate r(1);
	ROS_INFO("%s", argv[1]);
	ROS_INFO("%s", pname);
	char command[100];
	char folder[80];
	if (ros::ok())
	{
		uint Nsamples = atoi(argv[4]);
		printf("input: swept volume: %s, height: %f, distance: %f, Nsamples: %d\n",argv[1],h,m,Nsamples);

		std_seed();
		sprintf(folder, "data/cylinderSpecial/h_%d_m_%d/", (int)floor(100*h), (int)floor(100*m));
		sprintf(command, "mkdir -p %s", folder);
		system(command);

		Logger log(get_logging_str(folder, robot_file));

		SamplingInterface sampler(log);
		sampler.init( new SamplingCTOCylinder(argv[1], h, m));
		//sampler.mcmc(Nsamples);
		sampler.hmc(Nsamples);
		//sampler.uniform_normalized(Nsamples);


		//system("convert_points_to_train_data(X, fname, rmax)");
	}
}
