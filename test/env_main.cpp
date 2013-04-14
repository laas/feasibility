#include <ros/ros.h>
#include <ros/time.h>
#include "environment.h"

#include "motionplannerhyperplanar.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "environment_tester");
	ros::NodeHandle n;
	ros::Rate r(1);

	Environment* environment = Environment::getSalleBauzil();

	//MotionPlannerPerrin planner(*environment, argc, argv);
	MotionPlannerHyperPlanar planner(*environment, argc, argv);
	r.sleep();
	while (ros::ok())
	{
		planner.plan();
		planner.publish();
		r.sleep();
	}
}
