#pragma once
#include <vector>
#include "ros_util.h"

struct Environment{
	std::vector<ros::RVIZVisualMarker*> objects;

	Environment(){
	}
	~Environment(){
		clean();
	}
	void publish(){
		std::vector<ros::RVIZVisualMarker*>::iterator obj;
		for(obj = objects.begin();obj!=objects.end();obj++){
			(*obj)->publish();
		}
	}
	void clean(){
		std::vector<ros::RVIZVisualMarker*>::iterator obj;
		for(obj = objects.begin();obj!=objects.end();obj++){
			delete (*obj);
		}
		objects.clear();
	}
};

struct EnvironmentSalleBauzil: public Environment{
	EnvironmentSalleBauzil(): Environment(){
		ros::TriangleObjectChair *chair = new ros::TriangleObjectChair("/evart/chair/origin");
		ros::TriangleObjectRobot *robot = new ros::TriangleObjectRobot("/evart/helmet/origin");
		objects.push_back(chair);
		objects.push_back(robot);
	}
};

