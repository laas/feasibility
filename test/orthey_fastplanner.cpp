#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector> //std::vector
#include <iostream> //cout

#include "rviz/visualmarker.h"
#include "util/util.h"
#include "environment/environment.h"
#include "planner/motionplanner_astar.h"
#include "planner/constraints_checker_ann.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "orthey_fast_replanner");
	ros::NodeHandle n;
	ros::Rate r(1);

	Environment* environment;
	MotionPlannerAStar *astar;

	environment->Nobjects = 60;
	environment = Environment::get13HumanoidsReal();
	astar = new MotionPlannerAStar(environment, argc, argv);

	ConstraintsChecker *cc = new ConstraintsCheckerANN(16);
	astar->setConstraintsChecker(cc);

	while (ros::ok())
	{
		astar->plan();
		if(astar->success()){
			astar->clean_publish();
			astar->update_planner();
			astar->publish_onestep_next();
		}
	}
}

