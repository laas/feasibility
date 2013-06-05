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

	if (ros::ok())
	{
		printf("input: neural net %s\n",argv[1]);

		//struct fann *ann = fann_create_from_file(argv[1]);
		//fann_type cyl[4];
		//cyl[0]=1;
		//cyl[1]=1;
		//cyl[2]=0.1;
		//cyl[3]=0.1;
		//fann_type *calc_out = fann_run(ann, cyl);
		//printf("outcome %f\n", *calc_out);

		Environment* environment = Environment::get13Humanoids();

		MotionPlannerAStar planner(*environment, argc, argv);
		planner.setConstraintsChecker( 
			//new ConstraintsCheckerWallConstraints(
				//new ConstraintsCheckerANN()
				new ConstraintsCheckerSweptVolume()
				//, -10, 10, -10, 10
			//)
		);
		r.sleep();
		planner.plan();
		planner.publish();
		r.sleep();

	}
}
