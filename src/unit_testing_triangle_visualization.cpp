#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <vector>
#include <sstream>
#include <iostream>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>
//bounding vertex hierarchy

#include "ros_util.h"
#include "util.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "unit_test_rviz_visualization");
	ros::NodeHandle n;
	ros::Rate r(1);

	char *chair_file = "../data/chairLabo.tris";
	char *robot_file = "../data/fullbody_-14_-21_-29.tris";
	while (ros::ok())
	{
		TriangleObject chair(chair_file, 1, 1, 0);
		TriangleObject robot(robot_file, 0, 0, 0);
		chair.rviz_publish();
		robot.rviz_publish();

		r.sleep();
	}
}
