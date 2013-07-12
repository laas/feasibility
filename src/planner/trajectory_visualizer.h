#pragma once
#include <vector>
#include <robot_state_publisher/robot_state_publisher.h>

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
	void init(std::vector<double> &q);
	TrajectoryVisualizer(double x=0, double y=0);
	std::vector<double> getFinalCoM();
	void reset();
	void rewind();
	bool next();
};
