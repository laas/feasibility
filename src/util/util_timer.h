#pragma once
#include <ros/console.h>
#include <string> //std::string
struct Stopper{
	double start;
	double end;
	double sum;
	std::string description;
};
class Timer{
	typedef std::map< std::string, Stopper > StopperMap;
	StopperMap stoppermap;
	void checkExist(const char *name);
public:
	Timer(){};
	~Timer();
	double getFinalTime(const char* name);
	void begin(const char* name);
	void end(const char* name);
	void reset();
	void print_summary();
	void register_stopper(const char* name, const char* description);
};
