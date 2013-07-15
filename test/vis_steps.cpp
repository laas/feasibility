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

#include "util/util.h"
#include "planner/trajectory_visualizer.h"
#include "rviz/visualmarker.h"
#include "environment/environment.h"

int main( int argc, char** argv )
{
	using namespace ros;
	ros::init(argc, argv, "footstep_visualizer");
	ros::NodeHandle n;
	ros::Rate r(1);

	std::string prefix = get_data_path();
	std::string chair_file = get_chair_str();
	std::string robot_file = get_robot_str();

	//######################################################
	// set robot start
	//######################################################
	std::vector<double> start;
	start.push_back(0.0);
	start.push_back(0.0);

	double cx=0.49,cy=0.05,cz=0.0;
	double rx=1.0,ry=0.00,rz=0.0;

	//######################################################
	// set goal 
	//######################################################
	std::vector<double> goal;
	goal.push_back(2.5); goal.push_back(0.0); goal.push_back(0.0);

	TrajectoryVisualizer *tv = new TrajectoryVisualizer(0,0);
	CSVReader data_q("playback_q.dat");
	std::vector<double> q = data_q.getV();
	ROS_INFO("q size: %d", q.size());

	r.sleep();
	FootMarker marker(0,0,0);

	Environment* environment = Environment::getSalleBauzil();

	while (ros::ok())
	{
		marker.reset();
		SphereMarker ss(start.at(0), start.at(1));
		ss.publish();

		SphereMarker sm(goal.at(0), goal.at(1));
		sm.publish();

		//ros::RVIZVisualMarker *c;
		//c = new ros::ColladaObject("package://feasibility/data/wall_laas8.obj");
		//c->setXYZ(1.5,-2.5,-0.01);
		//c->setRPYRadian(0,0,M_PI);
		//c->publish();
		
		CSVReader data_steps("playback_steps.dat");
		
		std::vector<std::vector<double> > fsi;
		fsi = data_steps.getVV(4);

		double last_xL = 0;
		double last_yL = 0;
		double last_tL = 0;
		double last_xR = 0;
		double last_yR = 0;
		double last_tR = 0;

		double xold = fsi.at(0).at(0);
		double yold = fsi.at(0).at(1);
		double told = fsi.at(0).at(2);

		for(uint i=1;i<fsi.size();i++){
			//half-foot-step-format v.3.0:
			// 1: x
			// 2: y
			// 3: theta
			// 4: ascii code for L or R
			// 5-11: absolute x,y,theta?
			printf("[%d] ", i);
			for(uint j=0;j<fsi.at(i).size();j++){
				printf("%f ",fsi.at(i).at(j));
			}
			printf("\n");
			double scale=1;
			double x,y,t;

			x = fsi.at(i).at(0);
			y = fsi.at(i).at(1);
			t = fsi.at(i).at(2);

			double abs_x = xold;
			double abs_y = yold;
			double abs_t = told;
			char foot = fsi.at(i).at(3);

			double newX = abs_x + cos(abs_t)*x - sin(abs_t)*y;
			double newY = abs_y + sin(abs_t)*x + cos(abs_t)*y;
			double newT = (abs_t + t);

			while(newT<-M_PI) newT+=2*M_PI;
			while(newT>M_PI)  newT-=2*M_PI;

			FootMarker f(newX,newY,newT);
			xold=newX;
			yold=newY;
			told=newT;

			if(foot == 'R'){
				f.set_color(1,0,0);
				f.drawLine(last_xR,last_yR);
				last_xR = newX;
				last_yR = newY;
			}else{
				f.set_color(0,1,0);
				f.drawLine(last_xR,last_yR);
				last_xR = newX;
				last_yR = newY;

			}
			f.publish();
			ROS_INFO("published footstep [%d] at x=%f, y=%f, theta=%f", i,newX,newY,newT);

		}
		//######################################################

		if(q.size()>0){
			//Replay trajectory
			tv->init(q);
			ros::Rate rq(500);
			while(tv->next()){
				ros::spinOnce();
				rq.sleep();
			}
		}
		r.sleep();

	}
}
