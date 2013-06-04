#include "environment.h"

void Environment13Humanoids::setObjects(){

	ros::CylinderMarkerTriangles *c;
	//c=new ros::CylinderMarkerTriangles(1.0,-0.1, 0.2, 0.3);
	//objects.push_back(c);
	//c=new ros::CylinderMarkerTriangles(0.5,0.0, 0.1, 0.03);
	//objects.push_back(c);
	//c=new ros::CylinderMarkerTriangles(0.2,0.0, 0.1, 0.03);
	//objects.push_back(c);
	for(uint i=0;i<100;i++){
		c=new ros::CylinderMarkerTriangles(rand(0,3),rand(-1,1), 0.03, 0.08);
		//c=new ros::CylinderMarkerTriangles(rand(0,3),rand(-0.5,0.5), rand(0.01,0.06), rand(0.01,0.04));
		c->set_color(1,0,0,0.5);
		objects.push_back(c);
	}

}
void Environment13Humanoids::setGoalObject(){
	goal = new ros::SphereMarker(3.0, 0.0, 0.2, 0.0);
	goal->addText("goal");
}
void Environment13Humanoids::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.2, 0.0);
	start->addText("start");
}
