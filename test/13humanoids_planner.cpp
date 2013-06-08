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
#include "environment.h"
#include "planner/motionplanner_astar.h"
#include "planner/constraints_checker_ann.h"
#include "planner/constraints_checker_swept_volumer.h"
#include "planner/constraints_checker_wallconstraints_decorator.h"
using namespace ros;
int main( int argc, char** argv )
{

	if(argc!=2){
		printf("usage: planner <NeuralNet>\n");
		return -1;
	}
	char pname[100];
	sprintf(pname, "humanoids13_planner_%d", hashit(argv[1]));

	ros::init(argc, argv, pname);
	ros::NodeHandle n;
	ros::Rate r(1);

	Environment* environment;



	MotionPlannerAStar *astar;

	environment = Environment::get13Humanoids();
	astar = new MotionPlannerAStar(environment, argc, argv);

	ConstraintsChecker *ccANN = new ConstraintsCheckerANN();
	ConstraintsChecker *ccSweptVolume = NULL;//new ConstraintsCheckerSweptVolume();
	while (ros::ok())
	{
		//printf("input: neural net %s\n",argv[1]);
		printf("Press \n[R] reload environment\n[1] for swept volume approximation\n[2] for ANN approximationx\n[3] clean footsteps\n[4] quit\n");
		char c;
		do
		    c = getchar();
		while (isspace(c));

		if(c=='R'){
			environment->resetInstance();
			environment = Environment::get13Humanoids();
			astar = new MotionPlannerAStar(environment, argc, argv);
		}
		if(c=='1'){
			astar->setConstraintsChecker(ccSweptVolume);
			astar->plan();
			astar->publish("green", "green");
		}

		if(c=='2'){
			astar->setConstraintsChecker(ccANN);
			astar->plan();
			astar->publish("red", "red");
		}
		if(c=='3'){
			astar->clean_publish();
		}
		if(c=='4'){
			break;
		}
	}
}
