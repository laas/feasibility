#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <vector> //std::vector
#include <iostream> //cout

#include "rviz/visualmarker.h"
#include "util/util.h"
#include "environment/environment.h"
#include "planner/motionplanner_astar.h"
#include "planner/constraints_checker_swept_volumer.h"
#include "planner/constraints_checker_ann.h"

using namespace ros;
bool plan;

void update(const std_msgs::Bool& stopPlanner){
	plan = !stopPlanner.data;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "perrin_fast_replanner");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Subscriber subscriber;

	subscriber = n.subscribe("/planner/stop", 100, &update);

	Environment* environment;
	MotionPlannerAStar *astar;

	environment = Environment::getSalleBauzil();
	astar = new MotionPlannerAStar(environment, argc, argv);

	ConstraintsChecker *cc = new ConstraintsCheckerSweptVolume();
	astar->setConstraintsChecker(cc);

	while (ros::ok())
	{
		if(plan){
			astar->plan();
			if(astar->success()){
				astar->clean_publish();
				astar->update_planner();
				astar->publish_onestep_next();
			}
		}else{
			r.sleep();
			ROS_INFO("Planner in STOPPING mode (change it on /planner/stop)");
		}
	}
}
