#pragma once
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <string>

// We are using frames only in the form of the Perrin Fast Footstep
// Planner
//
// q vector is filled like this:
//
//q[0]=ID
//q[1]=end of trajectory
//q[2]=time start
//q[3]=time end
//--- 17 values per frame:
//q[4:15]=q values (12 values)
//q[16]=comX
//q[17]=comY
//q[18]=waistOrient in radian
//q[19]=zmpX
//q[20]=zmpY
//
//the next starting points for frames i=1:T
//-> q[4 + i*17 + k], k=0:16;

class TrajectoryVisualizer{
private:
	uint _Nframes;
	uint _ctrFrames;
	uint _offset;
	std::vector<double> *_q;
	robot_state_publisher::RobotStatePublisher *_rsp;
	tf::TransformBroadcaster _br;
public:
	void init(std::vector<double> &q){
		this->_q = &q;
		this->_offset=4;
		this->_Nframes = (this->_q->size()-this->_offset)/17.0; //12 joint valzues + 3 CoM (x,y,t) + 2 ZMP (x,y)
		this->_ctrFrames = 0;

	}
	TrajectoryVisualizer(){
		TrajectoryVisualizer(0,0);
	}
	TrajectoryVisualizer(double x, double y){
		ros::Rate r(10);

		KDL::Tree tree("base_link");
		urdf::Model my_model;
		std::string URDFFilename = ros::package::getPath("feasibility")+"/data/hrp2.urdf";

		if (!kdl_parser::treeFromFile(URDFFilename, tree))
		{
			ROS_ERROR("Failed to construct kdl tree");
		}
		this->_rsp = new robot_state_publisher::RobotStatePublisher(tree);

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(x,y,0.0) ); //in frame base_link
		tf::Quaternion com_rot;
		com_rot.setRPY(0,0,0);
		transform.setRotation(com_rot);

		_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_frame", "base_link"));
	}
	void rewind(){
		_ctrFrames = 0;
	}
	bool next(){
		if(_ctrFrames >= _Nframes){
			return false;
		}
		std::map<std::string, double> q;
		q["RLEG_JOINT0"] = _q->at(_offset + _ctrFrames*17 + 0);
		q["RLEG_JOINT1"] = _q->at(_offset + _ctrFrames*17 + 1);
		q["RLEG_JOINT2"] = _q->at(_offset + _ctrFrames*17 + 2);
		q["RLEG_JOINT3"] = _q->at(_offset + _ctrFrames*17 + 3);
		q["RLEG_JOINT4"] = _q->at(_offset + _ctrFrames*17 + 4);
		q["RLEG_JOINT5"] = _q->at(_offset + _ctrFrames*17 + 5);

		q["LLEG_JOINT0"] = _q->at(_offset + _ctrFrames*17 + 6);
		q["LLEG_JOINT1"] = _q->at(_offset + _ctrFrames*17 + 7);
		q["LLEG_JOINT2"] = _q->at(_offset + _ctrFrames*17 + 8);
		q["LLEG_JOINT3"] = _q->at(_offset + _ctrFrames*17 + 9);
		q["LLEG_JOINT4"] = _q->at(_offset + _ctrFrames*17 + 10);
		q["LLEG_JOINT5"] = _q->at(_offset + _ctrFrames*17 + 11);
		_rsp->publishFixedTransforms();
		_rsp->publishTransforms(q, ros::Time::now());

		double CoM[3];
		CoM[0]=_q->at(_offset + _ctrFrames*17 + 12);
		CoM[1]=_q->at(_offset + _ctrFrames*17 + 13);
		CoM[2]=_q->at(_offset + _ctrFrames*17 + 14);

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(CoM[0],CoM[1]+0.1,-0.705) ); //in frame base_link
		tf::Quaternion com_rot;
		com_rot.setRPY(0,0,CoM[2]);
		transform.setRotation(com_rot);

		_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_frame", "base_link"));
		_ctrFrames++;
		return true;
	}

};
