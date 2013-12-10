#pragma once
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <vector>
#include "rviz/geometry.h"

struct TrajectoryVisualizer;

const char *colorLeft = "red";
const char *colorRight = "green";
const std::string topic = "/feasibility/footsteps/relative";
const uint step_horizon = 3;

class FootStepTrajectory{ 
  typedef std::vector<double> FootStepState;

private:
  uint current_step_index_;
  uint last_planner_start_index_;
  uint number_of_prescripted_steps_;
  TrajectoryVisualizer *tv_;
	boost::mutex footstep_mutex_;
  ros::NodeHandle n;
  ros::Publisher pub_;

  //std::vector<FootStepState> footsteps_;
  ros::Geometry start_;
  std::vector<std::vector<double> > footsteps_;

  void checkSafety( double &xr, double &yr, double &tr);
public:
  FootStepTrajectory();
  uint size();
  void lock();
  void unlock();

  bool isFinished();
  FootStepTrajectory( const FootStepTrajectory &rhs );

  ros::Geometry& getStart();
  ros::Geometry& getStartEvart(ros::Geometry &evart_com);
  void setStart(ros::Geometry &rhs);
  ros::Geometry& getWaist();
  std::vector<std::vector<double> >& getFootSteps();

  void append( ros::Geometry &start, FootStepTrajectory &rhs );
  void execute_one_step();
  void publish();

  void push_back( FootStepState &fss );
  void pop_back();
  void add_prescripted_end_sequence(const ros::Geometry &goal);
};
//static ros::Publisher FootStepTrajectory::pub_;

