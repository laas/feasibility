#include <rviz/rviz_visualmarker.h>
#include "environment.h"
#include "util.h"

Environment* Environment::singleton = NULL;
uint Environment::Nobjects = 10;
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
void Environment::reloadObjects(){
	DEBUG( ROS_INFO("Thread stopping"));
	thread_stop();

	clean();
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
	ROS_INFO("starting Environment thread");
	m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&Environment::thread_publish, this) );
}
void Environment::thread_stop(){
	if(this->m_thread!=NULL){
		this->m_thread->interrupt();
		std::string id = boost::lexical_cast<std::string>(this->m_thread->get_id());
		DEBUG(ROS_INFO("waiting for thread %s to terminate", id.c_str());)
		this->m_thread->join();
		//this->m_thread.reset();
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
void Environment::resetInstance()
{
	DEBUG(ROS_INFO("***** DELETE SINGLETON *****");)
	delete singleton;
	singleton = NULL;
}

Environment::Environment(){
	DEBUG(ROS_INFO("***** NEW ENVIRONMENT CREATED *****");)
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
	ROS_INFO("deleting environment");
	clean();
}
void Environment::clean(){
	thread_stop();
	std::vector<ros::RVIZVisualMarker*>::iterator obj;
	for(obj = objects.begin();obj!=objects.end();obj++){
		if(*obj!=NULL) delete (*obj);
		*obj=NULL;
	}
	objects.clear();
	DEBUG(ROS_INFO("***** DELETED OBJECTS *****");)
	if(goal!=NULL){ delete goal; goal=NULL;}
	if(start!=NULL){ delete start; start=NULL;}
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

