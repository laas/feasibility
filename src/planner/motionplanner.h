#pragma once
#include <vector>
#ifdef PQP_COLLISION_CHECKING
#include <pqp/PQP.h>
#endif
#include "rviz/rviz_visualmarker.h"

struct MotionPlanner{
protected:
	Environment *environment;
	ros::Geometry goal;

public:
	MotionPlanner(Environment &env){
		environment = &env;
	}

	void plan(){
		if(environment->isChanged()){
			ros::Geometry goalG = environment->getGoal();
			setGoal( goalG );
			ros::Geometry start = environment->getStart();
			setStart( start );

			std::vector<ros::RVIZVisualMarker*> objects = environment->getObjects();
			std::vector<ros::RVIZVisualMarker*>::iterator it;
			for(it=objects.begin(); it!=objects.end(); it++){
				addObjectToPlanner(*it);
			}
		}
		ROS_INFO("start planner");
		start_planner();
	}

	virtual void publish() = 0;
	virtual bool success() = 0;
protected:
	virtual void start_planner() = 0;
	virtual void setGoal( ros::Geometry &goal ) = 0;
	virtual void setStart( ros::Geometry &start ) = 0;
	virtual void addObjectToPlanner(ros::RVIZVisualMarker *m) = 0;

};
