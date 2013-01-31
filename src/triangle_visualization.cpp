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
	ros::init(argc, argv, "triangle_shapes");
	ros::NodeHandle n;
	ros::Rate r(1);

	char *chair_file = "/home/orthez/git/fastReplanningData/data/chairLabo.tris";
	char *robot_file = "/home/orthez/git/fastReplanningData/data/fullBodyApprox/fullbody_-14_-21_-29.tris";
	if (ros::ok())
	{
		FILE *fp = fopen_s("result.tmp", "w");
		fprintf(fp, "xdistance time distance\n");

		uint Mrepeats=20;

		for(double x=3;x>-3;x=x-0.1){
			double tsum = 0;
			double dsum = 0;
			TriangleObject chair(chair_file, x, 0, 0);
			TriangleObject robot(robot_file, 0, 0, 0);
			for(uint i=0;i<Mrepeats;i++){
				chair.rviz_publish();
				robot.rviz_publish();

				double Tstart =ros::Time::now().toSec();
				double d=chair.distance_to(robot);
				double Tend = ros::Time::now().toSec();

				double Tdur = Tend - Tstart;
				tsum += Tdur;
				dsum += d;
				ROS_INFO("%f:%d %f %f", x, i, Tdur, d);
			}

			double Tavg = tsum / (double)Mrepeats;
			double Davg = dsum / (double)Mrepeats;
			ROS_INFO("%f %f %f", x, Tavg, Davg);
			fprintf(fp, "%f %f %f\n", x, Tavg, Davg);
		}
		fclose(fp);

		r.sleep();
	}
}
