#include "environment.h"
Environment* Environment::singleton = NULL;

void EnvironmentSalleBauzil::setObjects(){
	//ros::TriangleObjectFloor *chair = new ros::TriangleObjectFloor(0.8, 0.5, "chairLabo.tris", "/evart/chair2/PO");
	ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
	chair->addText("/evart/chair2/PO");
	chair->subscribeToEvart("/evart/chair2/PO");

	objects.push_back(chair);
}
void EnvironmentSalleBauzil::setGoalObject(){
	goal = new ros::SphereMarker(2.5, 1.0, 0.2, 0.0);
	goal->addText("goal");
	goal->subscribeToEvart("/evart/helmet2/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
	start->addText("start");
	start->subscribeToEvart("/evart/robot/PO");
}
