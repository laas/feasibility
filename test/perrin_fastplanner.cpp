#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <vector> //std::vector
#include <iostream> //cout

#include "rviz/visualmarker.h"
#include "util/util.h"
#include "planner/constraints_checker_swept_volumer.h"
#include "planner/constraints_checker_naive.h"
#include "planner/constraints_checker.h"
#include "environment/environment.h"
#include "planner/motionplanner_astar.h"

static Environment* environment;
static MotionPlannerAStar *astar;
static ConstraintsChecker *cc;
static FootStepTrajectory *fst;

#define DEBUG(x) x

using namespace ros;
bool plan=false;

boost::shared_ptr<boost::thread> astar_thread;
void update(const std_msgs::Bool& stopPlanner){
	plan = !stopPlanner.data;
}

void thread_publish(){
  ros::Rate r(20); //Hz

  ros::Geometry goalG = environment->getGoal();
  astar->setGoal( goalG );

  ros::Geometry startG = environment->getStart();
  startG.setFoot('L');
  astar->setStart( startG );

  while(1){
			astar->plan();
			//if(astar->success()){
				FootStepTrajectory fst_new = astar->get_footstep_trajectory();

				fst->lock();
				fst->append(astar->getStart(), fst_new);
        //fst->publish();
				astar->setStart( fst->getStart() );
        fst->unlock();
			//}
  }
}

void thread_start(){
  if(astar_thread==NULL){
		ROS_INFO("starting ASTAR thread");
		astar_thread = boost::shared_ptr<boost::thread>(new boost::thread(&thread_publish ) );
	}
}
void thread_stop(){
  if(astar_thread!=NULL){
    astar_thread->interrupt();
    std::string id = boost::lexical_cast<std::string>(astar_thread->get_id());
    DEBUG(ROS_INFO("waiting for thread %s to terminate", id.c_str());)
		astar_thread->join();
  }
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "perrin_fast_replanner");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Subscriber subscriber;
	plan=false;

	subscriber = n.subscribe("/planner/stop", 100, &update);
	environment = Environment::getSalleBauzil();
	astar = new MotionPlannerAStar(environment, argc, argv);

	//cc = new ConstraintsCheckerNaive();
	cc = new ConstraintsCheckerSweptVolume();
	astar->setConstraintsChecker(cc);
  fst = new FootStepTrajectory();

	while (ros::ok())
	{
		if(plan){
			thread_start();
			fst->execute_one_step();
		}else{
			r.sleep();
			thread_stop();
			ROS_INFO("Planner in STOPPING mode (change it on /planner/stop)");
		}
	}
}
