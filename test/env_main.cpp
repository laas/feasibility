#include <ros/ros.h>
#include <ros/time.h>
#include "environment.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "environment_tester");
	ros::NodeHandle n;
	ros::Rate r(1);

	EnvironmentSalleBauzil environment;

	while (ros::ok())
	{
		environment.publish();
		r.sleep();
	}
}
