#include <ros/ros.h>
#include <ros/time.h>
#include "environment.h"
#include "motionplannerperrin.h"
#include "motionplannerhyperplanar.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "environment_tester");
	ros::NodeHandle n;
	ros::Rate r(1);

	EnvironmentSalleBauzil salle_bauzil;
	Environment* environment = &salle_bauzil;
	environment->init(); //3 instructions because of the slicing problem

	//MotionPlannerPerrin planner(*environment, argc, argv);
	MotionPlannerHyperPlanar planner(*environment, argc, argv);
	r.sleep();
	if (ros::ok())
	{
//		r.sleep();
		planner.plan();
		planner.publish();
		r.sleep();
	}
}
