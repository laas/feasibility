#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include "ros_util.h"
#include "util.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "mocap_tester");
	ros::NodeHandle n;
	ros::Rate r(1);

	TriangleObjectChair chair("/evart/chair/origin");
	TriangleObjectRobot robot("/evart/helmet/origin");

	while (ros::ok())
	{
		chair.publish();
		robot.publish();
		r.sleep();
	}
}
