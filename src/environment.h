#pragma once
#include <vector>
#include <boost/thread.hpp>
#include "rviz/rviz_visualmarker.h"

struct Environment{
protected:
	std::vector<ros::RVIZVisualMarker*> objects;
	ros::RVIZVisualMarker *goal;
	ros::RVIZVisualMarker *start;
	boost::shared_ptr<boost::thread> m_thread;
	bool changedEnv;
	void startThread();
	virtual void setGoalObject() = 0;
	virtual void setStartObject() = 0;
	virtual void setObjects() = 0;

	void publish();

	static Environment *singleton;
	Environment();
	~Environment();
public:
	static Environment* getSalleBauzil();
	void init();
	void clean();
	bool isChanged();
	std::vector<ros::RVIZVisualMarker*> getObjects();
	ros::Geometry getGoal();
	ros::Geometry getStart();
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

