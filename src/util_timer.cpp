#include <ros/console.h>
#include "util_timer.h"

void Timer::begin(const char* name){
	if(stopper.find(std::string(name)) == stopper.end()){
		ROS_INFO("Stopper %s not found, printing all...", name);
		print_summary();
		throw "stopper not found, please register first";
	}
	m_start_time = ros::Time::now().toSec();
}
void Timer::end(const char* name){
	if(stopper.find(std::string(name)) == stopper.end()){
		ROS_INFO("Stopper %s not found, printing all...", name);
		print_summary();
		throw "stopper not found, please register first";
	}

	m_stop_time = ros::Time::now().toSec();
	stopper[std::string(name)].first += m_stop_time - m_start_time;
}
void Timer::print_summary(){
	std::map<std::string, std::pair<double, std::string> >::const_iterator it;
	ROS_INFO("----------------------------------------");
	ROS_INFO("Time stopper summary -----");
	ROS_INFO("----------------------------------------");
	for(it=stopper.begin(); it!=stopper.end(); ++it){
		ROS_INFO("Stopper \"%s\" >> %f s", (*it).second.second.c_str(), (*it).second.first);
	}
	ROS_INFO("----------------------------------------");
}
void Timer::register_stopper(const char* name, const char* description){
	stopper.insert( std::make_pair( std::string(name), std::make_pair( 0.0 , std::string(description))));
}
