#pragma once
#include <ros/console.h>
#include <string> //std::string
class Timer{
	typedef std::map< std::string, std::pair<double, std::string> > StopperMap;
	StopperMap stopper;
	double m_start_time;
	double m_stop_time;
public:
	Timer(){};
	void begin(const char* name);
	void end(const char* name);
	void reset();
	void print_summary();
	void register_stopper(const char* name, const char* description);
};
