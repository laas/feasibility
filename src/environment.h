#pragma once
#include <vector>
#include <boost/thread.hpp>
#include "rviz/rviz_visualmarker.h"

struct Environment{
private:
	void thread_publish();
protected:
	std::vector<ros::RVIZVisualMarker*> objects;
	std::vector<ros::RVIZVisualMarker*> decorations;
	ros::RVIZVisualMarker *goal;
	ros::RVIZVisualMarker *start;
	boost::shared_ptr<boost::thread> m_thread;
	bool changedEnv;
	virtual void setGoalObject() = 0;
	virtual void setStartObject() = 0;
	virtual void setObjects() = 0;
	virtual void setDecorations(); //optional: set decoration objects, which are not considered for planning etc.

	void thread_start();
	void thread_stop();

	static Environment *singleton;
	Environment();
	virtual ~Environment();
public:
	static Environment* getInstance(const char* iname);
	static uint Nobjects;
	static void resetInstance();
	static Environment* getSalleBauzil();
	static Environment* get13Humanoids();
	static Environment* get13HumanoidsReal();
	void init();
	void clean();
	bool isChanged();
	void reloadObjects();
	std::vector<ros::RVIZVisualMarker*> getObjects();
	ros::Geometry getGoal();
	ros::Geometry getStart();
};

struct Environment13HumanoidsReal: public Environment{
	Environment13HumanoidsReal(): Environment(){ }
	void setObjects();
	void setGoalObject();
	void setStartObject();
	virtual void setDecorations();
};
struct Environment13Humanoids: public Environment{
	Environment13Humanoids(): Environment(){ }
	/*
	static Environment *getInstance(){
		return new Environment13Humanoids();
	}
	*/
	void setObjects();
	void setGoalObject();
	void setStartObject();
};
struct EnvironmentSalleBauzil: public Environment{
	EnvironmentSalleBauzil(): Environment(){ }
	/*
	static Environment *getInstance(){
		return new EnvironmentSalleBauzil();
	}
	*/
	void setObjects();
	void setGoalObject();
	void setStartObject();
};

