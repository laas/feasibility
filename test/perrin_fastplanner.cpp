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

#define DEBUG(x) 

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

      FootStepTrajectory fst_new = astar->get_footstep_trajectory();

      if(fst_new.size()<=3){
        //avoid constant blocking of the trajectory, if the planner obtains
        //small but very fast plans
        ros::Rate r(1);
        r.sleep();
      }

      fst->lock();
      fst->append(astar->getStart(), fst_new);
      astar->setStart( fst->getStart() );

      if(fst->isFinished()){
        ros::Geometry goal = environment->getGoal();
        ros::Geometry waist_evart = environment->getStart();
        ros::Geometry waist_expected = fst->getWaist();

        //waist_evart.print();
        //waist_expected.print();
        //goal.print();

        ros::Geometry evart_to_goal;
        evart_to_goal.setX( goal.getX() - waist_evart.getX() );
        evart_to_goal.setY( goal.getY() - waist_evart.getY() );

        ros::Geometry goal_in_current;

        goal_in_current.setX( waist_expected.getX() + evart_to_goal.getX() );
        goal_in_current.setY( waist_expected.getY() + evart_to_goal.getY() );

        double diff_expected_evart = 0;
        if(fabs(waist_expected.getYawRadian() - waist_evart.getYawRadian()) < M_PI){
          diff_expected_evart = waist_expected.getYawRadian() - waist_evart.getYawRadian();
        }else{
          diff_expected_evart = waist_evart.getYawRadian() - waist_expected.getYawRadian();
        }
        double new_goal_yaw = goal.getYawRadian() + diff_expected_evart;
        goal_in_current.setYawRadian( new_goal_yaw );
        //goal_in_current.print();

        //ROS_INFO("OLD STEP LENGTH %d", fst->size());
        fst->add_prescripted_end_sequence( goal_in_current );
        //ROS_INFO("NEW STEP LENGTH %d", fst->size());
        fst->unlock();
        return;
      }

      fst->unlock();
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
			while(ros::ok()){
        fst->execute_one_step();
      }
		}else{
			r.sleep();
			thread_stop();
			ROS_INFO("Planner in STOPPING mode (change it on /planner/stop)");
		}
	}
}
