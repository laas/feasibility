 #include "environment/environment.h"

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
}


void EnvironmentSalleBauzil::setObjects(){
  ros::RVIZVisualMarker *c;
  
  //c = new ros::TriangleObjectFloor(1.5, 0.5, "data/chairLabo.tris", std::string("fastReplanningData"));
  c = new ros::PrimitiveMarkerBox(1.5, 0.5, 0.72, 0.5, 0.98);
	c->addText("FACOM<<(o,,,o)>>");
	c->set_constant_offset(0, 0);
	//c->set_constant_rotation_radian(0,0,M_PI);
	c->subscribeToEvart("/evart/facom_box/PO");
	c->set_color(ros::OBSTACLE);
	objects.push_back(c);

  c = new ros::PrimitiveMarkerBox(2.75, -1.5, 0.6, 0.8, 1.1);
	c->addText("AIRPLANE<@GOAL>");
	//c->set_constant_rotation_radian(0,0,M_PI);
	c->subscribeToEvart("/evart/red_airbus_screw/PO");
	c->set_color(ros::OBSTACLE);
	objects.push_back(c);
}

void EnvironmentSalleBauzil::setGoalObject(){
	goal = new ros::SphereMarker(0.0, 2.0, 0.2, 0.0);
	//goal->make_interactive(0.3);
	goal->addText("<GOAL>");
	goal->set_constant_offset(0.8, 0);
	//goal->set_constant_rotation_radian(0,0,M_PI);
	//goal->subscribeToEvart("/evart/red_airbus_screw/PO");
}
void EnvironmentSalleBauzil::setStartObject(){
	start = new ros::SphereMarker(-1.0, -1.0, 0.2, 0.0);
	start->setRPYRadian(0,0,3*M_PI/4);
	start->addText("<START>");
}
