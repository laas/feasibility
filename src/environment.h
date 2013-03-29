#pragma once
#include <vector>
#include <boost/thread.hpp>
#include "ros_util.h"

struct Environment{
protected:
	std::vector<ros::RVIZVisualMarker*> objects;
	ros::RVIZVisualMarker *goal;
	ros::RVIZVisualMarker *start;
	boost::shared_ptr<boost::thread> m_thread;

private:
	void publish(){
		ros::Rate r(10); //Hz
		while(1){
			std::vector<ros::RVIZVisualMarker*>::iterator obj;
			for(obj = objects.begin();obj!=objects.end();obj++){
				(*obj)->publish();
			}
			this->goal->publish();
			//ROS_INFO("environment: published objects and goal");
			boost::this_thread::interruption_point();
			r.sleep();
		}
	}
protected:

	void startThread(){
		assert(!m_thread);
		m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&Environment::publish, this) );
	}

	virtual void setGoalObject(){ }
	virtual void setStartObject(){ }
	virtual void setObjects(){ }
public:
	Environment(){
		setGoalObject();
		setStartObject();
		setObjects();
	}

	~Environment(){
		clean();
		if(this->m_thread!=NULL){
			this->m_thread->interrupt();
			std::string id = boost::lexical_cast<std::string>(this->m_thread->get_id());
			ROS_INFO("waiting for thread %s to terminate", id.c_str());
			this->m_thread->join();
		}
	}
	void clean(){
		std::vector<ros::RVIZVisualMarker*>::iterator obj;
		for(obj = objects.begin();obj!=objects.end();obj++){
			delete (*obj);
		}
		objects.clear();
	}
	std::vector<ros::RVIZVisualMarker*> getObjects(){
		return objects;
	}
	ros::Geometry getGoal(){
		return goal->g;
	}
	ros::Geometry getStart(){
		return start->g;
	}
};

struct EnvironmentSalleBauzil: public Environment{
	EnvironmentSalleBauzil(): Environment(){
	}
	virtual void setObjects(){
		//ros::TriangleObjectChair *chair = new ros::TriangleObjectChair("/evart/chair/origin");
		ros::TriangleObjectChair *chair = new ros::TriangleObjectChair("/evart/chair2/PO");
		//ros::TriangleObjectRobot *robot = new ros::TriangleObjectRobot("/evart/helmet/origin");
		//goal = new ros::TriangleObjectFloor(1, 0, "tris/TRIS_head0.tris", "/evart/helmet/origin");
		//goal = new ros::TriangleObjectFloor(1.5, -0.5, "wall.tris", "/evart/helmet/origin");
		objects.push_back(chair);
		//objects.push_back(robot);
		//

	}
	virtual void setGoalObject(){
		goal = new ros::TriangleObjectFloor(1.5, -0.5, "box.tris", "/evart/helmet2/PO");
	}
	virtual void setStartObject(){
		start = new ros::TriangleObjectFloor(0.0, -0.0, "box.tris", "");
	}

};

