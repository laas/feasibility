#include "environment/environment.h"

void EnvironmentSalleBauzil::setDecorations(){
	ros::RVIZVisualMarker *c;
	c = new ros::ColladaObject("package://feasibility/data/wall_laas8.obj");

	c->setXYZ(1.5,-2.5,-0.01);
	c->setRPYRadian(0,0,M_PI);

	decorations.push_back(c);
}
void EnvironmentSalleBauzil::setObjects(){
	ros::TriangleObjectFloor *chair = new ros::TriangleObjectFloor(1.5, 0.5, "chairLabo.tris", "/evart/chair2/PO");
	//ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
	chair->addText("/evart/chair2/PO");
	//chair->subscribeToEvart("/evart/chair2/PO");
	chair->make_interactive(0.4);
	objects.push_back(chair);

	//chair = new ros::TriangleObjectFloor(1.5, 1.5, "chairLabo.tris", "/evart/chair3/PO");
	//ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
	//chair->addText("/evart/chair3/PO");
	//chair->subscribeToEvart("/evart/chair2/PO");
	//chair->make_interactive(0.4);
	//objects.push_back(chair);

}
void EnvironmentSalleBauzil::setGoalObject(){
	goal = new ros::SphereMarker(2.5, 0.0, 0.2, 0.0);
	goal->make_interactive(0.3);
	goal->addText("goal");
	//goal->subscribeToEvart("/evart/helmet2/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
	start = new ros::SphereMarker(0.0, 1.0, 0.2, 0.0);
	start->addText("start");
	//start->subscribeToEvart("/evart/robot/PO");
}
