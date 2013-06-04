#include <rviz/rviz_visualmarker.h>
#include "environment.h"
#include "util.h"

Environment* Environment::singleton = NULL;
#define DEBUG(x) x
Environment* Environment::get13Humanoids(){
	if(singleton==NULL){
		singleton = Environment13Humanoids::getInstance();
	}else{
		ABORT("One instance of environment already exists!");
	}
	singleton->init();
	return singleton;
}
Environment* Environment::getSalleBauzil(){
	if(singleton==NULL){
		singleton = EnvironmentSalleBauzil::getInstance();
	}else{
		ABORT("One instance of environment already exists!");
	}
	singleton->init();
	return singleton;
}
void Environment::cleanObjects(){
	std::vector<ros::RVIZVisualMarker*>::iterator obj;
	for(obj = objects.begin();obj!=objects.end();obj++){
		delete (*obj);
	}
	objects.clear();
	delete goal;
	delete start;
}
void Environment::reloadObjects(){
	DEBUG( ROS_INFO("Thread stopping"));
	thread_stop();

	cleanObjects();

	setGoalObject();
	setStartObject();
	setObjects();
	CHECK(goal!=NULL, "goal state was not initialized in your Environment!");
	CHECK(start!=NULL, "start state was not initialized in your Environment!");
	CHECK(objects.size()>0, "objects were not initialized in your Environment!");

	thread_start();
}
void Environment::thread_publish(){
	ros::Rate r(10); //Hz
	while(1){
		bool change = false;
		std::vector<ros::RVIZVisualMarker*>::iterator obj;
		for(obj = objects.begin();obj!=objects.end();obj++){
			(*obj)->publish();
			change |= (*obj)->isChanged();
		}
		this->goal->publish();
		this->start->publish();
		change |= this->start->isChanged();
		change |= this->goal->isChanged();

		if(change && !changedEnv){
			changedEnv = true;
			ROS_INFO("Environment changed!");
		}

		boost::this_thread::interruption_point();
		r.sleep();
	}
}

void Environment::thread_start(){
	//assert(!m_thread);
	ROS_INFO("starting Environment thread");
	m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&Environment::thread_publish, this) );
}
void Environment::thread_stop(){
	assert(m_thread);
	DEBUG(ROS_INFO("thread stop"));
	if(this->m_thread!=NULL){
		this->m_thread->interrupt();
		std::string id = boost::lexical_cast<std::string>(this->m_thread->get_id());
		ROS_INFO("waiting for thread %s to terminate", id.c_str());
		this->m_thread->join();
		this->m_thread.reset();
	}
}

bool Environment::isChanged(){
	if(changedEnv){
		changedEnv = false;
		return true;
	}else{
		return false;
	}
}
Environment::Environment(){
	goal = NULL;
	start = NULL;
	objects.empty();
	changedEnv = true;
}

void Environment::init(){
	setGoalObject();
	setStartObject();
	setObjects();
	CHECK(goal!=NULL, "goal state was not initialized in your Environment!");
	CHECK(start!=NULL, "start state was not initialized in your Environment!");
	CHECK(objects.size()>0, "objects were not initialized in your Environment!");
	thread_start();
}

Environment::~Environment(){
	clean();
	thread_stop();
}
void Environment::clean(){
	std::vector<ros::RVIZVisualMarker*>::iterator obj;
	for(obj = objects.begin();obj!=objects.end();obj++){
		delete (*obj);
	}
	objects.clear();
}
std::vector<ros::RVIZVisualMarker*> Environment::getObjects(){
	return objects;
}
ros::Geometry Environment::getGoal(){
	assert(goal);
	return goal->g;
}
ros::Geometry Environment::getStart(){
	assert(start);
	return start->g;
}

