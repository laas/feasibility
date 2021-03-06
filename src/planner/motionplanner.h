#pragma once
#include <vector>
#include <pqp/PQP.h>
#include "rviz/visualmarker.h"
#include "environment/environment.h"

struct MResults{
	uint iterations;
	double time;
	uint steps;
	std::vector< std::vector<double> > *step_vector;
	unsigned long long feasibilityChecks; //guaranteed bit sizes (at least): long (32 bit), long long (64 bit)
	bool success;
	void print(){
		std::cout << "success: " << success << ", steps: " << steps << ", iterations: " << iterations << ", time: " << time << ", feasibilityChecks: " << feasibilityChecks << std::endl;
	}
};
struct MotionPlanner{
protected:
	static Environment *environment;
	ros::Geometry goal;
	MResults results;
	bool environment_changed;

	double cur_sf_x, cur_sf_y, cur_sf_yaw;
	char cur_sf_foot;
	double cur_com_x, cur_com_y, cur_com_t;
public:
	MotionPlanner(Environment *env){
		if(environment!=NULL){
			environment=NULL; //only delete pointer not object, so that we can handle multiple environments in the main loop
		}
		environment = env;
		environment_changed = true;
	}
	void changeEnvironment(){
	  environment_changed = true;
  }
	virtual ~MotionPlanner(){}
	
	void update(){
		//ROS_INFO("***update environment***");
		//ros::Geometry goalG = environment->getGoal();
		//setGoal( goalG );
		//ros::Geometry start = environment->getStart();
		//setStart( start );

		cleanObjects();
		std::vector<ros::RVIZVisualMarker*> objects = environment->getObjects();
		std::vector<ros::RVIZVisualMarker*>::iterator it;
		for(it=objects.begin(); it!=objects.end(); it++){
			addObjectToPlanner(*it);
		}
	}

	void plan(){
		//if(environment->isChanged()){
			environment_changed=true;
			update();
			//ROS_INFO("[MOTION_PLANNER] replanning (ENV CHANGED)");
			start_planner();
			//ROS_INFO("[MOTION_PLANNER] finished replanning");
		//}else{
			//environment_changed=false;
		  //if(results.success==false){
        //update();
        //ROS_INFO("replanning (NO PREVIOUS PLAN)");
        //start_planner();
      //}
		//}
	}

	virtual void publish() = 0;
	virtual bool success() = 0;
	virtual MResults getResults(){
		return results;
	}
protected:
	virtual void start_planner() = 0;
	virtual void setGoal( ros::Geometry &goal ) = 0;
	virtual void setStart( ros::Geometry &start ) = 0;
	virtual void addObjectToPlanner(ros::RVIZVisualMarker *m) = 0;
	virtual void cleanObjects() = 0;

};
Environment *MotionPlanner::environment;
