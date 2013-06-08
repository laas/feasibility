#pragma once
#include <vector>
#include <boost/thread.hpp>
#include "rviz/rviz_visualmarker.h"

struct Environment{
private:
	void thread_publish();
protected:
	std::vector<ros::RVIZVisualMarker*> objects;
	ros::RVIZVisualMarker *goal;
	ros::RVIZVisualMarker *start;
	boost::shared_ptr<boost::thread> m_thread;
	bool changedEnv;
	virtual void setGoalObject() = 0;
	virtual void setStartObject() = 0;
	virtual void setObjects() = 0;

	void thread_start();
	void thread_stop();

	static Environment *singleton;
	Environment();
	~Environment();
public:
	static uint Nobjects;
	static void resetInstance();
	static Environment* getSalleBauzil();
	static Environment* get13Humanoids();
	void init();
	void clean();
	bool isChanged();
	void reloadObjects();
	std::vector<ros::RVIZVisualMarker*> getObjects();
	ros::Geometry getGoal();
	ros::Geometry getStart();
};

struct Environment13Humanoids: public Environment{
private:
	Environment13Humanoids(): Environment(){ }
public:
	static Environment *getInstance(){
		return new Environment13Humanoids();
	}
	void setObjects();
	void setGoalObject();
	void setStartObject();
};
struct EnvironmentSalleBauzil: public Environment{
private:
	EnvironmentSalleBauzil(): Environment(){ }
public:
	static Environment *getInstance(){
		return new EnvironmentSalleBauzil();
	}
	void setObjects();
	void setGoalObject();
	void setStartObject();
};

