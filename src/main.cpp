#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector> //std::vector
#include <iostream> //cout

#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>

#include <fast-replanning/fast-replanning-interface.hh>
#include <fast-replanning/types.hh>
//bounding vertex hierarchy

#include "ros_util.h"
#include "util.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "main_project");
	ros::NodeHandle n;
	ros::Rate r(1);
	//Logger log("mcmc.tmp");


	//######################################################
	// robotDATA
	//######################################################
	std::string prefix = get_data_path(); //robotDATA.dat path
	ROS_INFO("PREFIX %s", prefix.c_str());
	printf("%s", prefix.c_str());

	//######################################################
	// Tris files loader
	//######################################################
	char chair_file[200];
	char robot_file[200];
	sprintf(chair_file, "%s%s", prefix.c_str(), "chairLabo.tris");
	sprintf(robot_file, "%s%s", prefix.c_str(), "fullBodyApprox/fullbody_-14_-21_-29.tris");
	ROS_INFO("%s", chair_file);
	ROS_INFO("%s", robot_file);

	TriangleObject chair(chair_file, 2, 1, 0);
	TriangleObject robot(robot_file, 3, 1, 0);


	//######################################################
	// fastPlanner
	//######################################################
	fastreplanning::FastReplanningInterface *planner = fastreplanning::fastReplanningInterfaceFactory(prefix, argc, argv);
	planner->setVerboseLevel(10);
	/*
	std::vector<double> goal;
	goal.push_back(2.0);
	goal.push_back(1.0);
	planner->update3DGoalPositionProtected(goal);
	std::vector<double> curgoal;
	planner->getCurrentSetGoal(curgoal);
	ROS_INFO("curgoal: %d", curgoal.size());
	ROS_INFO("curgoal: %f %f", curgoal.at(0), curgoal.at(1));
	*/

	//ROS_INFO("add obstacle!");
	//planner->addObstacleFromDatabase(CHAIR, 0.49, -0.11, 0, 1,1,1); 
	//ROS_INFO("added chair");
	 //addObstacleFromDatabase
	//int type, double x, double y, double z, double theta, double length, double width, double heigth)

	while (ros::ok())
	{
		ROS_INFO("foot step planner START->");
		planner->mainLoop();
		
		std::vector<fastreplanning::footStepInterface> fsi;
		planner->getInterfaceSteps(fsi);
		ROS_INFO("size fsi: %d", fsi.size());
		std::vector<FootStepObject> fso;

		for(uint i=0;i<fsi.size();i++){
			ROS_INFO("data size fsi %d: %d", i, fsi.at(i).data.size());
			ROS_INFO("format fsi %s", fsi.at(i).format.c_str());
			for(uint j=0;j<fsi.at(i).data.size();j++){
				ROS_INFO("%f", fsi.at(i).data.at(j));
			}
			double x = fsi.at(i).data.at(0);
			double y = fsi.at(i).data.at(1);
			double theta = fsi.at(i).data.at(2);
			FootStepObject f(i,x,y,theta);
			f.rviz_publish();
			//fso.push_back(f);
			ROS_INFO("new footstep [%d] at x=%f, y=%f, theta=%f", i,x,y,theta);
		}
		for(uint i=0;i<fso.size();i++){
			//ROS_INFO("publishing footstep %d",i);
		}

		chair.rviz_publish();
		robot.rviz_publish();
		r.sleep();
	}
}
