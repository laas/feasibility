#include <ros/ros.h>
#include <ros/time.h>
#include "environment.h"
#include "planner.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "environment_tester");
	ros::NodeHandle n;
	ros::Rate r(1);

	EnvironmentSalleBauzil environment;
	MotionPlannerPerrin planner(environment, argc, argv);
	//MotionPlannerHyperPlanar planner(environment, argc, argv);

	int counter=0;
	while (ros::ok())
	{
		//environment.publish();
		planner.plan();
		planner.publish();
		r.sleep();
		//if(counter++>1) return 0;
	}
}
