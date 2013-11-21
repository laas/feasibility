#include <string>
#include "rviz/visualmarker.h"
#include "environment/environment.h"
#include "util/util.h"

Environment* Environment::singleton = NULL;
uint Environment::Nobjects = 10;
#define DEBUG(x)
Environment* Environment::get13HumanoidsReal(){
  return getInstance("13humanoidsReal");
}
Environment* Environment::get13Humanoids(){
  return getInstance("13humanoids");
}
Environment* Environment::getSalleBauzil(){
  return getInstance("SalleBauzil");
}
Environment* Environment::getTestBed(){
  return getInstance("TestBed");
}
Environment* Environment::getInstance(const char* iname){
  if(singleton==NULL){
    if(strcmp("13humanoidsReal", iname)==0){
      singleton = new Environment13HumanoidsReal();
    }else if(strcmp("13humanoids", iname)==0){
      singleton = new Environment13Humanoids();
    }else if(strcmp("SalleBauzil", iname)==0){
      singleton = new EnvironmentSalleBauzil();
    }else if(strcmp("TestBed", iname)==0){
      singleton = new EnvironmentFeasibilityTest();
    }else{
      PRINT("No class for name " << iname );
      ABORT("No class error");
    }
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
  setDecorations();
  CHECK(goal!=NULL, "goal state was not initialized in your Environment!");
  CHECK(start!=NULL, "start state was not initialized in your Environment!");
  CHECK(objects.size()>0, "objects were not initialized in your Environment!");

  thread_start();
}
void Environment::thread_publish(){
  ros::Rate r(20); //Hz
  while(1){
		{
			boost::mutex::scoped_lock lock(util_mutex);
			DEBUG(std::cout << "[ENV] >>>>>>||" << std::flush;)
			bool change = false;
			std::vector<ros::RVIZVisualMarker*>::iterator obj;
			for(obj = objects.begin();obj!=objects.end();obj++){
				(*obj)->publish();
				change |= (*obj)->isChanged();
			}
			for(obj = decorations.begin();obj!=decorations.end();obj++){
				(*obj)->publish();
			}
			this->goal->publish();
			this->start->publish();
			change |= this->start->isChanged();
			change |= this->goal->isChanged();

			if(change && !changedEnv){
				changedEnv = true;
				ROS_INFO("Environment changed!");
			}

			DEBUG( std::cout << "<<<<<<" << std::endl; )
			boost::this_thread::interruption_point();
		}
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

void Environment::setDecorations(){
}

void Environment::init(){
  setGoalObject();
  setStartObject();
  setObjects();
  setDecorations();
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
  for(obj = decorations.begin();obj!=decorations.end();obj++){
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

void Environment::setObjectPosition(uint id, double x, double y, double yaw){
  if(id>objects.size()){
    ROS_INFO("WARNING: setObjectPosition, ID not existant");
    return;
  }
  objects.at(id)->g_.x_ = x;
  objects.at(id)->g_.y_ = y;
  objects.at(id)->g_.setRPYRadian(0,0,yaw);
}

void Environment::setStart(double x, double y){
  this->start->g_.x_ = x;
  this->start->g_.y_ = y;
}
void Environment::setGoal(double x, double y){
  this->goal->g_.x_ = x;
  this->goal->g_.y_ = y;
}
ros::Geometry Environment::getGoal(){
  assert(goal);
  return goal->g_;
}
ros::Geometry Environment::getStart(){
  assert(start);
  return start->g_;
}

