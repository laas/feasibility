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
#include "planner/constraints_checker_swept_volumer.h"

using namespace ros;


void store_footsteps( std::vector< std::vector<double> > *fsv, Logger &log ){
	if(fsv->size()==0) return;
	for(uint i=0;i<fsv->size();i++){
		log("%f %f %f\n", fsv->at(i).at(0)*100, 
				fsv->at(i).at(1)*100,
				toDeg(fsv->at(i).at(2)));
	}
}
int main( int argc, char** argv )
{
	Logger log("footsteps_used.dat");
	ros::init(argc, argv, "contact_pdf");
	ros::NodeHandle n;
	ros::Rate r(1);

	Environment* environment;
	MotionPlannerAStar *astar;

	environment = Environment::getSalleBauzil();
	astar = new MotionPlannerAStar(environment, argc, argv);

	ConstraintsChecker *cc = new ConstraintsCheckerSweptVolume();
	astar->setConstraintsChecker(cc);

	double x_low = -1;
	double x_high = 4;
	double y_low = -1;
	double y_high = 4;
	uint Ntraj = 500;
	for (uint t = 0; t<Ntraj && ros::ok(); t++)
	{
		astar->plan();
		if(astar->success()){
			astar->clean_publish();
			astar->update_planner();
			astar->publish_onestep_next();
			astar->steps_to_results();
			MResults results = astar->getResults();
			store_footsteps(results.step_vector, log);
		}

		double x_rand = rand(x_low, x_high);
		double y_rand = rand(y_low, y_high);
		environment->setStart(x_rand, y_rand);

		x_rand = rand(x_low, x_high);
		y_rand = rand(y_low, y_high);
		environment->setGoal(x_rand, y_rand);

		x_rand = rand(x_low, x_high);
		y_rand = rand(y_low, y_high);
		environment->setObjectPosition(0, x_rand, y_rand, 0);
	}
}

