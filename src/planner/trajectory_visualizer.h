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
	uint Nframes_;
	uint ctrFrames_;
	uint offset_;
	double com_offset_x_;
	double com_offset_y_;
	double com_offset_t_;

	double cur_com_offset_x_;
	double cur_com_offset_y_;
	double cur_com_offset_t_;

	std::vector<double> *_q;
	robot_state_publisher::RobotStatePublisher *rsp_;
	tf::TransformBroadcaster br_;
          
        ros::Publisher trajectory_pub_;
public:
	void init(std::vector<double> &q);
	TrajectoryVisualizer(double x=0, double y=0, double t=0);
	std::vector<double> getFinalCoM();
	void setUpperBodyJointsDefault( std::map<std::string, double> &q );
	void setCoMOffset(double cur_com_x, double cur_com_y, double cur_com_t);
	void setCoMOffset(std::vector<double> com);

	void setPlanarWorldBaseTransform(double x, double y, double yaw);
	void setTranslationTransform(const char* from, const char* to, double x, double y, double z, double roll, double pitch, double yaw);
	void reset();
	void rewind();
	bool next();

#define NB_JOINT_HRP2 36
#define NB_PUBLISHED_JOINT_HRP2 12
        static const std::string JointNames[NB_JOINT_HRP2];

        void publishTrajectory();

        
};
