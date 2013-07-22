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
	chair->addText("/evart/chair2/origin");
	//chair->subscribeToEvart("/evart/chair2/PO");
	chair->make_interactive(0.4);
	objects.push_back(chair);

	//EXPERIMENTAL:
	
	//objects.push_back(chair);
	//chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,1.5,M_PI);

	//ros::TriangleObjectFloor *tmp = new ros::TriangleObjectFloor(0.8, 0.5, "bar.tris", "/evart/chair2/PO");
	//tmp->g.setRPYRadian(0,0,M_PI/6);
	//objects.push_back(tmp);

	//for(uint i=0;i<30;i++){
		//objects.push_back( new ros::TriangleObjectFloor(0.8, 0.5, "part.tris", "/evart/chair2/PO") );
		//objects.push_back( new ros::CubeMarker(rand(0.8,2), rand(0.5,3)));
	//}
}
void EnvironmentSalleBauzil::setGoalObject(){
	goal = new ros::SphereMarker(2.5, 0.0, 0.2, 0.0);
	goal->make_interactive(0.2);
	goal->addText("goal");
	//goal->subscribeToEvart("/evart/helmet2/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
	start->addText("start");
	//start->subscribeToEvart("/evart/robot/PO");
}
