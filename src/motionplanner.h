#pragma once
#include <vector>
#include <pqp/PQP.h>
#include <fast-replanning/fast-replanning-interface.hh>
#include "ros_util.h"

struct MotionPlanner{
protected:
	Environment *environment;
	ros::Geometry goal;

public:
	MotionPlanner(Environment &env){
		environment = &env;
	}

	void plan(){
		ros::Geometry goalG = environment->getGoal();
		setGoal( goalG );
		ros::Geometry start = environment->getStart();
		setStart( start );

		std::vector<ros::RVIZVisualMarker*> objects = environment->getObjects();
		std::vector<ros::RVIZVisualMarker*>::iterator it;
		ROS_INFO("%d objects found", objects.size());
		for(it=objects.begin(); it!=objects.end(); it++){
			addObjectToPlanner(*it);
		}
		start_planner();
	}

	virtual void publish() = 0;
protected:
	virtual void start_planner() = 0;
	virtual void setGoal( ros::Geometry &goal ) = 0;
	virtual void setStart( ros::Geometry &start ) = 0;
	virtual void addObjectToPlanner(ros::RVIZVisualMarker *m) = 0;

};
