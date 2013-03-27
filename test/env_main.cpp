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

	ros::Geometry goal;
	goal.x = 1.5;
	goal.y = -1;
	goal.tz = 0;
	planner.setGoal( goal );
	while (ros::ok())
	{
		environment.publish();
		planner.plan();
		planner.publish();
		r.sleep();
	}
}
