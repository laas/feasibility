#include "environment.h"

void Environment13HumanoidsReal::setDecorations(){
	//ros::RVIZVisualMarker *c;
	//c = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,-0.0,M_PI);
	//decorations.push_back(c);
}
void Environment13HumanoidsReal::setObjects(){
	//chair->addText("/evart/chair2/PO");
	//chair->subscribeToEvart("/evart/chair2/PO");
	//objects.push_back(chair);

	ros::RVIZVisualMarker *c;

	//ros::TriangleObjectFloor *chair = new ros::TriangleObjectFloor(0.8, 0.5, "chairLabo.tris", "/evart/chair2/PO");
	//c=new ros::CylinderMarkerTriangles(1,0, rand(0.1,0.1), rand(0.5,0.5));
	c=new ros::CylinderMarkerTriangles(1,0, rand(0.3,0.3), rand(0.5,0.5));
	c->set_color(0,0.8,0,0.2);
	c->make_interactive();
	objects.push_back(c);

	c=new ros::CylinderMarkerTriangles(2,0, rand(0.3,0.3), rand(0.5,0.5));
	c->set_color(0,0.8,0,0.2);
	c->make_interactive();
	objects.push_back(c);

	c=new ros::CylinderMarkerTriangles(3,0, rand(0.3,0.3), rand(0.5,0.5));
	c->set_color(0,0.8,0,0.2);
	c->make_interactive();
	objects.push_back(c);
	//RADIUS \elem [0.01, 0.1]
	//HEIGHT \elem [0.01, 0.5]
	// otherwise the MLP function is undefined
	/*
	for(uint i=0;i<Nobjects;i++){
		c=new ros::CylinderMarkerTriangles(rand(0.2,2.8),rand(-1.5,1.5), rand(0.01,0.04), rand(0.01,0.06));
		c->set_color(1.0,0.3,0.3,0.3);
		c->make_interactive();
		objects.push_back(c);
	}
	for(uint i=0;i<10;i++){
		c=new ros::CylinderMarkerTriangles(rand(0.2,2.8),rand(-1.5,1.5), rand(0.01,0.08), rand(0.1,0.5));
		c->set_color(0,0.8,0,0.3);
		c->make_interactive();
		objects.push_back(c);
	}
	*/
}
void Environment13HumanoidsReal::setGoalObject(){
	goal = new ros::SphereMarker(3.0, 0.0, 0.2, 0.0);
	goal->addText("goal");
	goal->subscribeToEvart("/evart/helmet2/PO");
}
void Environment13HumanoidsReal::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
	start->addText("start");
	start->subscribeToEvart("/evart/robot/PO");
}
