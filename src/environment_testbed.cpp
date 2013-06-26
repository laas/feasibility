#include "environment.h"

void EnvironmentFeasibilityTest::setDecorations(){
	ros::RVIZVisualMarker *c;
	c = new ros::ColladaObject("package://feasibility/data/wall_laas8.obj");
	c->setXYZ(1.5,-1.2,-0.01);
	c->setRPYRadian(0,0,M_PI);
	decorations.push_back(c);

	std::string robot_file = get_tris_str("fullBodyApprox/fullbody_35_-11_29.tris");

//%../fastReplanningData/data/fullBodyApprox/fullbody_35_-11_29.tris

	ros::Geometry robot_pos;
	robot_pos.x = 0;
	robot_pos.y = 0;

	ros::TrisTriangleObject *robot = new ros::TrisTriangleObject(robot_file.c_str(), robot_pos);
	robot->set_color(0.6,0.0,0.6,0.3);
	decorations.push_back(robot);

}
void EnvironmentFeasibilityTest::setObjects(){
	ros::RVIZVisualMarker *c;
	c=new ros::CylinderMarkerTriangles(0.5,-0.2,0.03,0.03);
	c->set_color(0,0.8,0,1);
	c->make_interactive();
	objects.push_back(c);
}
void EnvironmentFeasibilityTest::setGoalObject(){
	goal = new ros::SphereMarker(1.0, 1.0, 0.02, 0.0);
	goal->addText("goal");
}
void EnvironmentFeasibilityTest::setStartObject(){
	start = new ros::SphereMarker(0.0, 0.0, 0.02, 0.0);
	start->addText("start");
}

