#include <ros/console.h>
#include "util_timer.h"

void Timer::reset(){
	StopperMap::iterator it;
	Stopper s;
	s.start= 0.0;
	s.end= 0.0;
	s.sum= 0.0;
	for(it=stoppermap.begin(); it!=stoppermap.end(); ++it){
		s.description = (*it).second.description;
		(*it).second = s;
	}
}
void Timer::checkExist(const char *name){
	if(stoppermap.find(std::string(name)) == stoppermap.end()){
		ROS_INFO("Stopper %s not found, printing all...", name);
		print_summary();
		throw "stopper not found, please register first";
	}
}
Timer::~Timer(){
	stoppermap.clear();
}
double Timer::getFinalTime(const char* name){
	checkExist(name);
	return stoppermap[std::string(name)].sum;
}
void Timer::begin(const char* name){
	checkExist(name);
	stoppermap[std::string(name)].start = ros::Time::now().toSec();
}
void Timer::end(const char* name){
	checkExist(name);
	std::string s = std::string(name);
	stoppermap[s].end = ros::Time::now().toSec();
	stoppermap[s].sum += stoppermap[s].end - stoppermap[s].start;
}
void Timer::print_summary(){
	StopperMap::const_iterator it;
	ROS_INFO("----------------------------------------");
	for(it=stoppermap.begin(); it!=stoppermap.end(); ++it){
		ROS_INFO("Stopper \"%s\" >> %f s", (*it).second.description.c_str(), (*it).second.sum);
	}
	ROS_INFO("----------------------------------------");
}
void Timer::register_stopper(const char* name, const char* description){
	Stopper s;
	s.start= 0.0;
	s.end= 0.0;
	s.sum= 0.0;
	s.description = std::string(description);
	stoppermap.insert( std::make_pair( std::string(name), s ) );
}
