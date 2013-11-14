#include "environment/environment.h"

void EnvironmentSalleBauzil::setDecorations(){
	ros::RVIZVisualMarker *c;
	c = new ros::ColladaObject("package://feasibility/data/wall_laas9.obj");

	//c->setXYZ(1.5,-2.5,-0.01);
	c->setXYZ(0.0,0,-0.01);
	c->setRPYRadian(0,0,M_PI);
	decorations.push_back(c);

  c = new ros::TriangleObjectFloor(0.0, 0.0, "data/wall.tris", std::string("fastReplanningData"));
	decorations.push_back(c);
}
void EnvironmentSalleBauzil::setObjects(){
	ros::RVIZVisualMarker *c;
  	
  //c = new ros::TriangleObjectFloor(1.5, 0.5, "data/chairLabo.tris", std::string("fastReplanningData"));
  c = new ros::CuboidMarker(1.5, 0.5, 0.72, 0.5, 0.98);
	//ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
	c->addText("RandomToolBoxCUBOID");
	//chair->subscribeToEvart("/evart/chair2/PO");
	c->set_color(ros::OBSTACLE);
	c->make_interactive(0.6);
	objects.push_back(c);

	//ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
}
void EnvironmentSalleBauzil::setGoalObject(){
	goal = new ros::SphereMarker(2.5, 0.0, 0.2, 0.0);
	//goal->setRPYRadian(0,0,M_PI);
	goal->make_interactive(0.3);
	goal->addText("goal");
	//goal->subscribeToEvart("/evart/helmet2/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
	start = new ros::SphereMarker(0.0, 1.0, 0.2, 0.0);
	start->addText("start");
	//start->subscribeToEvart("/evart/robot/PO");
}
