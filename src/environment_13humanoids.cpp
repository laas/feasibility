#include "environment.h"

void Environment13Humanoids::setObjects(){

	ros::CylinderMarkerTriangles *c;
	c=new ros::CylinderMarkerTriangles( 5.0, 8.0, 0.1, 0.1);
	objects.push_back(c);

	for(uint i=0;i<Nobjects;i++){
		c=new ros::CylinderMarkerTriangles(rand(0.2,2.8),rand(-0.8,0.8), rand(0.01,0.03), rand(0.01,0.1));//rand(0.01,0.05));
		c->set_color(1,0,0,0.5);
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
