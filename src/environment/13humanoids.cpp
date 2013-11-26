#include "environment/environment.h"
#include "rviz/primitive_marker.h"

void Environment13Humanoids::setDecorations(){
	ros::RVIZVisualMarker *c;
	//c = new ros::BlenderMeshTriangleObject("package://feasibility/data/AluminumChair.dae","chairLabo.tris",1.2,-0.0,M_PI);
	//c = new ros::ColladaObject("package://feasibility/data/wall.obj",1.5,-1.5);
	//c->setRPYRadian(M_PI/2,0,0);
	//decorations.push_back(c);
	c = new ros::ColladaObject("package://feasibility/data/wall_laas8.obj");
	c->setXYZ(1.5,-1.0,-0.01);
	c->setRPYRadian(0,0,M_PI);
	decorations.push_back(c);
}
void Environment13Humanoids::setObjects(){

	ros::PrimitiveMarkerCylinder *c;
	c=new ros::PrimitiveMarkerCylinder( 5.0, 8.0, 0.1, 0.1);
	objects.push_back(c);

	for(uint i=0;i<Nobjects;i++){
		c=new ros::PrimitiveMarkerCylinder(rand(0.2,2.8),rand(-0.8,0.8), rand(0.01,0.03), rand(0.01,0.1));//rand(0.01,0.05));
		c->set_color(0,0.8,0,0.8);
		objects.push_back(c);
	}

}
void Environment13Humanoids::setGoalObject(){
	goal = new ros::SphereMarker(3.0, 0.0, 0.1, 0.0);
	goal->addText("goal");
}
void Environment13Humanoids::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.1, 0.0);
	start->addText("start");
}
