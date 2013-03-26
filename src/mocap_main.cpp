#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>

#include "ros_util.h"
#include "util.h"

using namespace ros;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "mocap_tester");
	ros::NodeHandle n;
	ros::Rate r(1);

	std::string prefix = get_data_path();
	std::string chair_file = get_chair_str();
	std::string robot_file = get_robot_str();

	Geometry chair_pos;
	chair_pos.x = 0.49;
	chair_pos.y = 0.05;
	chair_pos.tz = 0.0;

	Geometry robot_pos;
	robot_pos.x = 0;
	robot_pos.y = 0;
	robot_pos.tz = 0;

	TriangleObject chair(chair_file, chair_pos);
	TriangleObject robot(robot_file, robot_pos);
	std::string mocap = std::string("/evart/helmet/origin");
	chair.subscribeToEvart( mocap );

	while (ros::ok())
	{
		ROS_INFO("ping");
		chair.publish();
		r.sleep();
	}
}
