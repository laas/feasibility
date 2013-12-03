#include "environment/environment.h"
#include "rviz/primitive_marker.h"

void EnvironmentSalleBauzil::setDecorations(){
  ros::RVIZVisualMarker *c;
  c = new ros::ColladaObject("package://feasibility/data/wall_laas9.obj");
  c->setXYZ(0.0,0,-0.01);
  decorations.push_back(c);

  c = new ros::ColladaObject("package://feasibility/data/blue_column.obj");
  c->setXYZ(0.3,3.3,-0.01);
  decorations.push_back(c);

  c = new ros::ColladaObject("package://feasibility/data/blue_column.obj");
  c->setXYZ(-3.795,3.3,-0.01);
  decorations.push_back(c);

  c = new ros::ColladaObject("package://feasibility/data/blue_column.obj");
  c->setXYZ(4.96,3.3,-0.01);
  decorations.push_back(c);

  c = new ros::ColladaObject("package://feasibility/data/estrade.obj");
  c->setXYZ(0.0,0,-0.01);
  decorations.push_back(c);

  c = new ros::ColladaObject("package://feasibility/data/barrier.obj");
  c->setXYZ(0.0,0,-0.01);
  decorations.push_back(c);
}

void EnvironmentSalleBauzil::setObjects(){
  ros::RVIZVisualMarker *c;
  //c = new ros::TriangleObjectFloor(1.5, 0.5, "data/chairLabo.tris", std::string("fastReplanningData"));
  c = new ros::PrimitiveMarkerBox(-0.5, 0, 0.5, 0.72, 0.98);
  //c = new ros::PrimitiveMarkerCylinder(0.0, 0.0, 1, 2);
	c->addText("FACOM<<(o,,,o)>>");
	//c->set_constant_rotation_radian(0,0,M_PI);
	c->make_interactive(1.5);
	//c->subscribeToEvart("/evart/facom_box/PO");
	c->set_color(ros::OBSTACLE);
	objects.push_back(c);

  //c = new ros::PrimitiveMarkerBox(-3.0, 2.0, 0.6, 0.8, 1.1);
  //c->addText("AIRPLANE<@GOAL>");
  //c->subscribeToEvart("/evart/red_airbus_screw/PO");
  //c->set_color(ros::OBSTACLE);
  //objects.push_back(c);
}

void EnvironmentSalleBauzil::setGoalObject(){
  goal = new ros::SphereMarker(-2.0, 2.0, 0.02, 0.0);
  goal->addText("<GOAL>");
  goal->setRPYRadian(0,0,M_PI);
  goal->set_constant_offset(0.75, 0);
  goal->subscribeToEvart("/evart/red_airbus_screw/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
  start = new ros::SphereMarker(2.0, -1.00, 0.2, 0.0);
  start->addText("<START>");
  //start->setRPYRadian(0,0,3*M_PI/4);
  start->set_csf_in_chest_yaw(1.0438);
  start->subscribeToEvart("/evart/HRP2_waist_sensor_frame/PO",true,20);
}
