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
	ros::init(argc, argv, "footstep_planner");
	ros::NodeHandle n;
	ros::Rate r(1);

	std::string prefix = get_data_path();
	std::string chair_file = get_chair_str();
	std::string robot_file = get_robot_str();

	//######################################################
	// fastPlanner
	//######################################################
	fastreplanning::FastReplanningInterface *planner 
		= fastreplanning::fastReplanningInterfaceFactory(prefix, argc, argv);
	planner->setVerboseLevel(15); //0 5 15

	//######################################################
	// set robot start
	//######################################################
	std::vector<double> start;
	start.push_back(1.0);
	start.push_back(-1.0);
	start.push_back(0.0);
	start.push_back(0.0);
	start.push_back(0.0);
	start.push_back(0.0);
	SphereMarker ss(998, start.at(0), start.at(1), 0.05);
	ss.rviz_publish();

	double cx=0.49,cy=0.05,cz=0.0;
	double rx=1.0,ry=0.00,rz=0.0;
	TriangleObject chair(chair_file, cx, cy, cz);
	TriangleObject robot(robot_file, rx, ry, rz);

	planner->mainLoop();
	planner->addObstacleFromDatabase(CHAIR, cx, cy, cz, 1,1,1); 
	//planner->addObstacleFromDatabase(BOX, 0.49, 0.21, 0, 1,1,1); 
	planner->updateLocalizationProtected(start);

	std::vector<FootStepObject> fso;

	while (ros::ok())
	{


		std::vector<double> goal;
		goal.push_back(1.6); goal.push_back(0.1); goal.push_back(0.0);
		planner->update3DGoalPositionProtected(goal);

		ROS_INFO("foot step planner START->");
		planner->mainLoop();

		chair.rviz_publish();
		robot.rviz_publish();
		ROS_INFO("published chair and sv");
		
		std::vector<fastreplanning::footStepInterface> fsi;
		planner->getInterfaceSteps(fsi);
		ROS_INFO("NUMBER OF FOOTSTEPS: %d", fsi.size());

		double last_xL = 0;
		double last_yL = 0;
		double last_tL = 0;
		double last_xR = 0;
		double last_yR = 0;
		double last_tR = 0;
		for(uint i=0;i<fsi.size();i++){
			if(i==0){
				for(uint j=0;j<fso.size();j++){
					//ROS_INFO("erasing step %d",j);
					//ROS_INFO("id= %d", fso.at(j).id);
					//%fso.at(j).remove();
				}
				//fso.clear();
			}
			//half-foot-step-format v.3.0:
			// 1: x
			// 2: y
			// 3: theta
			// 4: ascii code for L or R
			// 5-11: do not know
			for(uint j=0;j<fsi.at(i).data.size();j++){
				printf("%f - ",fsi.at(i).data.at(j));
			}
			printf("\n");
			double scale=1;
			double x,y,t;

//double radV = -state_prev->x[2]*PI/180.0;
//s.x = cos(radV)*(state_curr->x[0] - state_prev->x[0]) - 
//  sin(radV)*(state_curr->x[1] - state_prev->x[1]);
//s.y = sin(radV)*(state_curr->x[0] - state_prev->x[0]) + 
//  cos(radV)*(state_curr->x[1] - state_prev->x[1]);
//s.theta = (state_curr->x[2] - state_prev->x[2])*PI/180.0;
//s.f = state_prev->x[3];

			double 
			if(fsi.at(i).data.at(3) == 'R'){
				x = fsi.at(i).data.at(0)+last_xL;
				y = fsi.at(i).data.at(1)+last_yL;
			}else{
				x = fsi.at(i).data.at(0)+last_xR;
				y = fsi.at(i).data.at(1)+last_yR;
			}
			FootStepObject f(i,scale*x,scale*y,scale*t);

			if(fsi.at(i).data.at(3) == 'R'){
				f.changeColor(1,0,0);
				f.drawLine(scale*last_xR, scale*last_yR);
				last_xR = x;
				last_yR = y;
			}else{
				f.changeColor(0,1,0);
				f.drawLine(scale*last_xL, scale*last_yL);
				last_xL = x;
				last_yL = y;
			}

			if(i==0 || i==1){
				f.changeColor(1,1,0);
			}
			if(i==fsi.size()-1 || i==fsi.size()-2){
				f.changeColor(1,0.6,0);
			}

			f.rviz_publish();
			fso.push_back(f);
			ROS_INFO("new footstep [%d] at x=%f, y=%f, theta=%f", i,x,y,theta);
		}

		//######################################################
		// set goal 
		//######################################################
		std::vector<double> curgoal;
		planner->getCurrentSetGoal(curgoal);
		ROS_INFO("curgoal[%d]: %f %f", curgoal.size(), curgoal.at(0), curgoal.at(1));

		SphereMarker sm(999, goal.at(0), goal.at(1), 0.05);
		sm.rviz_publish();
		//######################################################

		r.sleep();
	}
}
