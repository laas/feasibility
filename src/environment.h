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
public:
	Environment();
	~Environment();
	void init();
	void clean();
	bool isChanged();
	std::vector<ros::RVIZVisualMarker*> getObjects();
	ros::Geometry getGoal();
	ros::Geometry getStart();
};

struct EnvironmentSalleBauzil: public Environment{
	EnvironmentSalleBauzil(): Environment(){
	}
	void setObjects(){
		//ros::TriangleObjectFloor *chair = new ros::TriangleObjectFloor(0.8, 0.5, "chairLabo.tris", "/evart/chair2/PO");
		ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
		chair->addText("/evart/chair2/PO");
		chair->subscribeToEvart("/evart/chair2/PO");
		//ros::BlenderMesh *mesh = new ros::BlenderMesh("file://home/aorthey/git/feasibility/data/kellkore.stl",0,0,0);
		//mesh->addText("<Fresh Blender Export>");
		objects.push_back(chair);
	}
	void setGoalObject(){
		goal = new ros::SphereMarker(2.5, 1.5, 0.2, 0.0);
		goal->addText("goal");
		goal->subscribeToEvart("/evart/helmet2/PO");
	}
	void setStartObject(){
		//start = new ros::BlenderMesh("package://romeo_description/meshes/Torso.dae",0,0,0);
		//start->addText("start");
		//start->subscribeToEvart("/evart/robot/PO");
		start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
		start->addText("start");
		start->subscribeToEvart("/evart/robot/PO");
	}

};

