#include "environment.h"

void EnvironmentSalleBauzil::setObjects(){
	ros::TriangleObjectFloor *chair = new ros::TriangleObjectFloor(0.8, 0.5, "chairLabo.tris", "/evart/chair2/PO");
	//ros::BlenderMeshTriangleObject *chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,0.5,M_PI);
	chair->addText("/evart/chair2/PO");
	chair->subscribeToEvart("/evart/chair2/PO");
	objects.push_back(chair);

	//EXPERIMENTAL:
	
	//objects.push_back(chair);
	//chair = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,1.5,M_PI);

	//ros::TriangleObjectFloor *tmp = new ros::TriangleObjectFloor(0.8, 0.5, "bar.tris", "/evart/chair2/PO");
	//tmp->g.setYawRadian(M_PI/6);
	//objects.push_back(tmp);

	//for(uint i=0;i<30;i++){
		//objects.push_back( new ros::TriangleObjectFloor(0.8, 0.5, "part.tris", "/evart/chair2/PO") );
		//objects.push_back( new ros::CubeMarker(rand(0.8,2), rand(0.5,3)));
	//}
}
void EnvironmentSalleBauzil::setGoalObject(){
	goal = new ros::SphereMarker(1.5, 0.0, 0.2, 0.0);
	goal->addText("goal");
	goal->subscribeToEvart("/evart/helmet2/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
	start->addText("start");
	start->subscribeToEvart("/evart/robot/PO");
}
