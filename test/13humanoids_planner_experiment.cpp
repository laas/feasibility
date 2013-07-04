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
#include <fann.h>

#include "rviz/rviz_visualmarker.h"
#include "util.h"
#include "environment/environment.h"
#include "planner/motionplanner_astar.h"
#include "planner/constraints_checker_ann.h"
#include "planner/constraints_checker_swept_volumer.h"
#include "planner/constraints_checker_wallconstraints_decorator.h"
using namespace ros;
int main( int argc, char** argv )
{

	if(argc!=3){
		printf("usage: planner <name> <objects>\n");
		return -1;
	}
	char pname[100];
	sprintf(pname, "humanoids_planner_%d", hashit(argv[1]));

	ros::init(argc, argv, pname);
	ros::NodeHandle n;
	ros::Rate r(1);

	Environment* environment = NULL;
	MotionPlannerAStar *astar = NULL;

	if(astar!=NULL) delete astar;

	MResults results;
	ConstraintsChecker *ccANN6 = new ConstraintsCheckerANN(6);
	ConstraintsChecker *ccANN5 = new ConstraintsCheckerANN(5);
	ConstraintsChecker *ccANN4 = new ConstraintsCheckerANN(4);
	ConstraintsChecker *ccSweptVolume = new ConstraintsCheckerSweptVolume();

	uint objects = atoi(argv[2]);
	if (ros::ok())
	{
		char lname[100];
		sprintf(lname, "planning_%dobj.tmp", objects);
		Logger logger(lname);
		for(uint i=0;i<100;i++){

			environment->resetInstance();
			Environment::Nobjects = objects;
			environment = Environment::get13Humanoids();

			astar = new MotionPlannerAStar(environment, argc, argv);

			//
			//####################
			//
			astar->setConstraintsChecker(ccANN4);
			astar->plan();
			astar->publish("red", "green");
			astar->clean_publish();
			results = astar->getResults();
			results.print();
			logger("ann6 %d %d %f %f %f %f\n", objects, results.success, results.time, (double)results.steps, (double)results.iterations, (double)results.feasibilityChecks);
			astar->setConstraintsChecker(ccANN5);
			astar->plan();
			astar->publish("red", "green");
			astar->clean_publish();
			results = astar->getResults();
			results.print();
			logger("ann5 %d %d %f %f %f %f\n", objects, results.success, results.time, (double)results.steps, (double)results.iterations, (double)results.feasibilityChecks);
			//
			//####################
			//
			//astar = new MotionPlannerAStar(environment, argc, argv);
			astar->setConstraintsChecker(ccANN4);
			astar->plan();
			astar->publish("red", "green");
			astar->clean_publish();
			results = astar->getResults();
			results.print();
			logger("ann4 %d %d %f %f %f %f\n", objects, results.success, results.time, (double)results.steps, (double)results.iterations, (double)results.feasibilityChecks);
			astar->setConstraintsChecker(ccSweptVolume);
			astar->plan();
			astar->publish("red", "red");
			astar->clean_publish();
			results = astar->getResults();
			results.print();
			logger("sv %d %d %f %f %f %f\n", objects, results.success, results.time, (double)results.steps, (double)results.iterations, (double)results.feasibilityChecks);
			ROS_INFO("Finished trial %d/100 with %d objects", i, objects);
		}//conduct enough trials to get sufficient statistics
	}//if ros
}
