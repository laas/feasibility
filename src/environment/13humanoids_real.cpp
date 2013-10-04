#include <algorithm>    // std::min
#include "environment/environment.h"

void Environment13HumanoidsReal::setDecorations(){
	ros::RVIZVisualMarker *c;
	//c = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,-0.0,M_PI);
	//c = new ros::ColladaObject("package://feasibility/data/wall.obj",1.5,-1.5);
	//c->setRPYRadian(M_PI/2,0,0);
	//decorations.push_back(c);
	c = new ros::ColladaObject("package://feasibility/data/wall_laas8.obj");
	c->setXYZ(1.5,-2.0,-0.01);
	c->setRPYRadian(0,0,M_PI);
	decorations.push_back(c);
}
void Environment13HumanoidsReal::createApproxObj(const char* name, double x, double y, double yaw){
	ros::RVIZVisualMarker *c, *d;
	c = new ros::CylinderMarkerTriangles(x,y, 0.33, 0.8),
	c->set_color(0,0.8,0,0.2);
	objects.push_back(c);

	d = new ros::ColladaObject(name);
	d->setXYZ( c->g.x, c->g.y, c->g.z);
	d->setScale( 0.4,0.4,0.4);
	d->setRPYRadian( M_PI/2, 0, yaw);
	decorations.push_back(d);
}
void Environment13HumanoidsReal::setObjects(){
	//chair->addText("/evart/chair2/PO");
	//chair->subscribeToEvart("/evart/chair2/PO");
	//objects.push_back(chair);

	ros::RVIZVisualMarker *c;
	//RADIUS \elem [0.01, 0.1]
	//HEIGHT \elem [0.01, 0.5]
	// otherwise the MLP function is undefined at the moment
	//seed(Nobjects);
	for(uint i=0;i<Nobjects;i++){
		//c=new ros::CylinderMarkerTriangles(rand(0.2,2.8),rand(-0.8,0.8), rand(0.03,0.04), rand(0.03,0.05));
		c=new ros::CylinderMarkerTriangles(rand(0.3,4.0),rand(-1.2,1.2), rand(0.04,0.06), rand(0.03,0.05));
		c->set_color(0,0.8,0,0.8);
		//c->set_color(0.1,0.1,0.1,1.0);
		objects.push_back(c);
	}
	for(int i=0;i< std::min((int)objects.size(),5);i++){
		objects.at(i)->make_interactive(0.15);
	}

	createApproxObj("package://feasibility/data/AluminumChair.dae",1.8,-0.9,rand(M_PI-0.2,M_PI+0.2));
	createApproxObj("package://feasibility/data/AluminumChair.dae",0,0.8,rand(M_PI/2-0.2,M_PI/2+0.2));

}
void Environment13HumanoidsReal::setGoalObject(){
	goal = new ros::SphereMarker(1.5, 0.0, 0.3, 0.0);
	goal->addText("goal");
	goal->make_interactive(0.8);
	goal->subscribeToEvart("/evart/helmet2/PO");
}
void Environment13HumanoidsReal::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
	start->addText("start");
	start->subscribeToEvart("/evart/robot/PO");
}
