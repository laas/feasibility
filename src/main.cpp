#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>

#include <fast-replanning/fast-replanning-interface.hh>

#include "ros_util.h"
#include "util.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "main_project");
	ros::NodeHandle n;
	ros::Rate r(1);

	std::string prefix = get_data_path();
	std::string chair_file = get_chair_str();
	std::string robot_file = get_robot_str();
	TriangleObject chair(chair_file, 2, 1, 0);
	TriangleObject robot(robot_file, 3, 1, 0);

	//######################################################
	// fastPlanner
	//######################################################
	fastreplanning::FastReplanningInterface *planner 
		= fastreplanning::fastReplanningInterfaceFactory(prefix, argc, argv);
	planner->setVerboseLevel(15);

	//######################################################
	// set robot start
	//######################################################
	std::vector<double> start;
	start.push_back(0.0);
	start.push_back(0.0);
	start.push_back(0.0);
	planner->updateLocalizationProtected(start);

	//planner->addObstacleFromDatabase(CHAIR, 0.49, -0.11, 0, 1,1,1); 
	//planner->addObstacleFromDatabase(BOX, 0.49, 0.21, 0, 1,1,1); 

	int c=0;
	while (ros::ok())
	{
		ROS_INFO("foot step planner START->");
		planner->mainLoop();
		
		std::vector<fastreplanning::footStepInterface> fsi;
		planner->getInterfaceSteps(fsi);
		ROS_INFO("NUMBER OF FOOTSTEPS: %d", fsi.size());
		std::vector<FootStepObject> fso;

		for(uint i=0;i<fsi.size();i++){
			//ROS_INFO("data size fsi %d: %d", i, fsi.at(i).data.size());
			//ROS_INFO("format fsi %s", fsi.at(i).format.c_str());

			//half-foot-step-format v.3.0:
			// 1: x
			// 2: y
			// 3: theta
			// 4: ascii code for L or R
			// 5-11: do not know

			for(uint j=0;j<fsi.at(i).data.size();j++){
				//ROS_INFO("%f", fsi.at(i).data.at(j));
			}
			double x = fsi.at(i).data.at(0);
			double y = fsi.at(i).data.at(1);
			double theta = fsi.at(i).data.at(2);
			double scale=10;
			FootStepObject f(i,10*x,10*y,10*theta);
			f.rviz_publish();
			ROS_INFO("new footstep [%d] at x=%f, y=%f, theta=%f", i,x,y,theta);
		}

		//######################################################
		// set goal 
		//######################################################
		std::vector<double> curgoal;
		planner->getCurrentSetGoal(curgoal);
		ROS_INFO("curgoal[%d]: %f %f", curgoal.size(), curgoal.at(0), curgoal.at(1));

		std::vector<double> goal;
		goal.push_back(4.0); goal.push_back(0.0); goal.push_back(0.0);
		planner->update3DGoalPositionProtected(goal);

		SphereMarker sm(NULL, goal.at(0), goal.at(1), 0.2);
		sm.rviz_publish();
		//######################################################

		chair.rviz_publish();
		robot.rviz_publish();
		r.sleep();
	}
}
