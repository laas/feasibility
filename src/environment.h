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
			this->start->publish();
			boost::this_thread::interruption_point();
			r.sleep();
		}
	}
protected:

	void startThread(){
		assert(!m_thread);
		m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&Environment::publish, this) );
	}

	virtual void setGoalObject() = 0;
		//ROS_INFO("Environment setGoalObject not yet implemented");
	virtual void setStartObject() = 0;
	virtual void setObjects() = 0;
public:
	Environment(){
		goal = NULL;
		start = NULL;
		objects.empty();
	}

	void init(){
		setGoalObject();
		setStartObject();
		setObjects();
		CHECK(goal!=NULL, "goal state was not initialized in your Environment!");
		CHECK(start!=NULL, "start state was not initialized in your Environment!");
		CHECK(objects.size()>0, "objects were not initialized in your Environment!");
		startThread();
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
		assert(goal);
		return goal->g;
	}
	ros::Geometry getStart(){
		assert(start);
		return start->g;
	}
};

struct EnvironmentSalleBauzil: public Environment{
	EnvironmentSalleBauzil(): Environment(){
	}
	void setObjects(){
		ros::TriangleObjectFloor *chair = new ros::TriangleObjectFloor(0.49, 0.25, "chairLabo.tris", "/evart/chair2/PO");
		chair->addText("/evart/chair2/PO");
		objects.push_back(chair);
	}
	void setGoalObject(){
		goal = new ros::SphereMarker(2.5, 1.5, 0.2, 0.0);
		goal->addText("goal");
		goal->subscribeToEvart("/evart/helmet2/PO");
	}
	void setStartObject(){
		start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
		start->addText("start");
		start->subscribeToEvart("/evart/robot/PO");
	}

};

