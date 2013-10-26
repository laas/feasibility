/*! Standard includes */
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

/*! Specific to UNIX */
#include <sys/time.h>

/*! ROS specific */
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trajectory_msgs/JointTrajectory.h>


/* ! Feasibility Framework */
#include "planner/trajectory_visualizer.h"
#include "util/util.h"

const std::string TrajectoryVisualizer::JointNames[NB_JOINT_HRP2]= {
          "RLEG_JOINT0",
          "RLEG_JOINT1",
          "RLEG_JOINT2",
          "RLEG_JOINT3",
          "RLEG_JOINT4",
          "RLEG_JOINT5",
          
          "LLEG_JOINT0",
          "LLEG_JOINT1",
          "LLEG_JOINT2",
          "LLEG_JOINT3",
          "LLEG_JOINT4",
          "LLEG_JOINT5",

          "COM_X",
          "COM_Y",
          "COM_THETA",
          
          "ZMP_X",
          "ZMP_Y"
};

void 
TrajectoryVisualizer::
init(std::vector<double> &q)
{
  this->q_ = &q;
  if(!this->q_->empty()){
    this->offset_=4;
    this->Nframes_ = (this->q_->size()-this->offset_)/17.0; //12 joint values + 3 CoM (x,y,t) + 2 ZMP (x,y)
    this->ctrFrames_ = 0;
    
    publishTrajectory();
  }
}

TrajectoryVisualizer::
TrajectoryVisualizer(double x, double y, double t)
{
  ROS_INFO("searching for path of package 'feasibility' ...");
  std::string URDFFilename = ros::package::getPath("feasibility")+"/data/hrp2.urdf";
  
  std::ifstream urdf_fs(URDFFilename.c_str());
  std::string urdf_content(       (std::istreambuf_iterator<char>(urdf_fs) ),
                                  (std::istreambuf_iterator<char>()      ) );
  
  ros::NodeHandle nh;
  
  ROS_INFO("Uploading robot to parameter server (%s)", URDFFilename.c_str());
  nh.setParam("/robot_description", urdf_content.c_str());
  nh.getParam("/FastRePlanner/planner_publish_robot_configurations", publish_configurations);
  
  com_offset_x_ = x;
  com_offset_y_ = y;
  com_offset_t_ = t;
  ros::Rate r(10);
  
  KDL::Tree tree("/base_link");
  urdf::Model my_model;
  
  if (!kdl_parser::treeFromFile(URDFFilename, tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }
  //if (publish_configurations)
    this->setPlanarWorldBaseTransform(x,y,0);
  
  this->rsp_ = new robot_state_publisher::RobotStatePublisher(tree);
  this->reset();
  //  if (publish_configurations)
    this->setPlanarWorldBaseTransform(x,y,0);
  
  trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/planner/trajectory",2);
  seq_id_ = 0;
}

void TrajectoryVisualizer::
setCoMOffset(std::vector<double> com)
{
  com_offset_x_ = com.at(0);
  com_offset_y_ = com.at(1);
  com_offset_t_ = com.at(2);
}

void TrajectoryVisualizer::
setCoMOffset(double cur_com_x, double cur_com_y, double cur_com_t)
{
  com_offset_x_ = cur_com_x;
  com_offset_y_ = cur_com_y;
  com_offset_t_ = cur_com_t;
}

void TrajectoryVisualizer::
reset()
{
  std::map<std::string, double> q;
  q["RLEG_JOINT0"] = 0.0;
  q["RLEG_JOINT1"] = 0.0;
  q["RLEG_JOINT2"] = -toRad(26.0);
  q["RLEG_JOINT3"] = toRad(50.0);
  q["RLEG_JOINT4"] = -toRad(24.0);
  q["RLEG_JOINT5"] = 0.0;
  
  q["LLEG_JOINT0"] = 0.0;
  q["LLEG_JOINT1"] = 0.0;
  q["LLEG_JOINT2"] = -toRad(26.0);
  q["LLEG_JOINT3"] = toRad(50.0);
  q["LLEG_JOINT4"] = -toRad(24.0);
  q["LLEG_JOINT5"] = 0.0;
  
  setUpperBodyJointsDefault(q);
  
  if(publish_configurations)
    {
      rsp_->publishFixedTransforms();
      rsp_->publishTransforms(q, ros::Time::now());
    }
}

void TrajectoryVisualizer::setUpperBodyJointsDefault( std::map<std::string, double> &q )
{
  /**
     \brief HALFSITTINGPOSITION                                         \
     legs : 0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0
     chest and head : 0.0, 0.0, 0.0, 0.0,
     right arm : 15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0,
     left arm : 15.0, 10.0, 0.0, -30.0, 0.0, 0.0, 10.0,
     right hand : -10.0, 10.0, -10.0, 10.0, -10.0,
     left hand : -10.0, 10.0, -10.0, 10.0, -10.0
  */
  q["RARM_JOINT0"] = toRad(15.0);
  q["RARM_JOINT1"] = -toRad(10.0);
  q["RARM_JOINT2"] = toRad(0.0);
  q["RARM_JOINT3"] = -toRad(30.0);
  q["RARM_JOINT4"] = toRad(0.0);
  q["RARM_JOINT5"] = toRad(0.0);
  q["RARM_JOINT6"] = toRad(10.0);

  q["LARM_JOINT0"] = toRad(15.0);
  q["LARM_JOINT1"] = toRad(10.0);
  q["LARM_JOINT2"] = toRad(0.0);
  q["LARM_JOINT3"] = -toRad(30.0);
  q["LARM_JOINT4"] = toRad(0.0);
  q["LARM_JOINT5"] = toRad(0.0);
  q["LARM_JOINT6"] = toRad(10.0);
  
  q["RHAND_JOINT0"] = -toRad(10.0);
  q["RHAND_JOINT1"] = toRad(10.0);
  q["RHAND_JOINT2"] = -toRad(10.0);
  q["RHAND_JOINT3"] = toRad(10.0);
  q["RHAND_JOINT4"] = -toRad(10.0);
  
  q["LHAND_JOINT0"] = -toRad(10.0);
  q["LHAND_JOINT1"] = toRad(10.0);
  q["LHAND_JOINT2"] = -toRad(10.0);
  q["LHAND_JOINT3"] = toRad(10.0);
  q["LHAND_JOINT4"] = -toRad(10.0);
  
  q["HEAD_JOINT0"] = 0.0;
  q["HEAD_JOINT1"] = 0.0;
  
  q["CHEST_JOINT0"] = 0.0;
  q["CHEST_JOINT1"] = 0.0;
  /*
    q["RARM_JOINT0"] = 0.0;
    q["RARM_JOINT1"] = -0.1;
    q["RARM_JOINT2"] = 0.0;
    q["RARM_JOINT3"] = -0.8;
    q["RARM_JOINT4"] = 0.0;
    q["RARM_JOINT5"] = -0.2;
    q["RARM_JOINT6"] = 0.0;
    
    q["LARM_JOINT0"] = 0.0;
    q["LARM_JOINT1"] = -0.1;
    q["LARM_JOINT2"] = 0.0;
    q["LARM_JOINT3"] = -0.8;
    q["LARM_JOINT4"] = 0.0;
    q["LARM_JOINT5"] = -0.2;
    q["LARM_JOINT6"] = 0.0;
    
    q["RHAND_JOINT0"] = 0.0;
    q["RHAND_JOINT1"] = 0.0;
    q["RHAND_JOINT2"] = 0.0;
    q["RHAND_JOINT3"] = 0.0;
    q["RHAND_JOINT4"] = 0.0;
    
    q["LHAND_JOINT0"] = 0.0;
    q["LHAND_JOINT1"] = 0.0;
    q["LHAND_JOINT2"] = 0.0;
    q["LHAND_JOINT3"] = 0.0;
    q["LHAND_JOINT4"] = 0.0;

    q["HEAD_JOINT0"] = 0.0;
    q["HEAD_JOINT1"] = 0.0;
    
    q["CHEST_JOINT0"] = 0.0;
    q["CHEST_JOINT1"] = 0.0;
  */
}

void TrajectoryVisualizer::
rewind()
{
  ctrFrames_ = 0;
}
bool TrajectoryVisualizer::next()
{
  if(this->q_->empty()){
    return false;
  }
  if(ctrFrames_ >= Nframes_){
    return false;
  }
  std::map<std::string, double> q;
  
  q["RLEG_JOINT0"] = q_->at(offset_ + ctrFrames_*17 + 0);
  q["RLEG_JOINT1"] = q_->at(offset_ + ctrFrames_*17 + 1);
  q["RLEG_JOINT2"] = q_->at(offset_ + ctrFrames_*17 + 2);
  q["RLEG_JOINT3"] = q_->at(offset_ + ctrFrames_*17 + 3);
  q["RLEG_JOINT4"] = q_->at(offset_ + ctrFrames_*17 + 4);
  q["RLEG_JOINT5"] = q_->at(offset_ + ctrFrames_*17 + 5);
  
  q["LLEG_JOINT0"] = q_->at(offset_ + ctrFrames_*17 + 6);
  q["LLEG_JOINT1"] = q_->at(offset_ + ctrFrames_*17 + 7);
  q["LLEG_JOINT2"] = q_->at(offset_ + ctrFrames_*17 + 8);
  q["LLEG_JOINT3"] = q_->at(offset_ + ctrFrames_*17 + 9);
  q["LLEG_JOINT4"] = q_->at(offset_ + ctrFrames_*17 + 10);
  q["LLEG_JOINT5"] = q_->at(offset_ + ctrFrames_*17 + 11);
  
  setUpperBodyJointsDefault(q);
  
  //publish only for simulation in rviz
  if(publish_configurations){
    rsp_->publishFixedTransforms();
    rsp_->publishTransforms(q, ros::Time::now());
    ROS_INFO("WARNING publishing Q !");
  }
  
  double CoM[3];
  CoM[0]=q_->at(offset_ + ctrFrames_*17 + 12) + com_offset_x_;
  CoM[1]=q_->at(offset_ + ctrFrames_*17 + 13) + com_offset_y_;
  CoM[2]=q_->at(offset_ + ctrFrames_*17 + 14) + com_offset_t_;
  
  cur_com_offset_x_ = CoM[0];
  cur_com_offset_y_ = CoM[1];
  cur_com_offset_t_ = CoM[2];
  
  //if (publish_configurations)
  this->setPlanarWorldBaseTransform(CoM[0], CoM[1], CoM[2]);

  ctrFrames_++;
  return true;
}

std::vector<double> 
TrajectoryVisualizer::
getFinalCoM()
{
  //if(this->q_->empty()){
  //	ROS_INFO("ATTENTION: empty com vector");
  //	std::vector<double> m;
  //	return m;
  //}
  std::vector<double> CoM(3);
  ROS_INFO("ctrFrames_: %d, Nframes: %d", ctrFrames_, Nframes_);
  CoM.at(0) = cur_com_offset_x_;
  CoM.at(1) = cur_com_offset_y_;
  CoM.at(2) = cur_com_offset_t_;
  return CoM;
}

void 
TrajectoryVisualizer::
setPlanarWorldBaseTransform(
    double x, double y, double yaw)
{
  //use constant offset of CoM to origin of robot
  setTranslationTransform("/world_frame", "/base_link", x, y-0.1, 0.650, 0, 0, yaw);
}

// deprecated transformation functions
void 
TrajectoryVisualizer::
setTranslationTransform(
    const char* from, const char* to, double x, double y, double z, 
    double roll, double pitch, double yaw)
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x,y,z) ); //in frame base_link
  tf::Quaternion com_rot;
  com_rot.setRPY(roll, pitch, yaw);
  transform.setRotation(com_rot);
  
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from, to));
}

void 
TrajectoryVisualizer::
publishTrajectory()
{
  trajectory_msgs::JointTrajectory goal;
  struct timeval tv;
  gettimeofday(&tv,0);
  
  /// Building header.
  goal.header.seq = seq_id_;
  goal.header.stamp.sec = tv.tv_sec;
  goal.header.stamp.nsec = tv.tv_usec/1000.0;
  std::ostringstream oss;
  oss<< "feasibility_" << seq_id_;
  goal.header.frame_id = oss.str();
  seq_id_++;
  
  for(unsigned int i=0;i<NB_PUBLISHED_JOINT_HRP2;i++)
    goal.joint_names.push_back(JointNames[i]);
  
  goal.points.resize(Nframes_);

  for(unsigned int idPoint=0;idPoint<Nframes_;idPoint++)
    {
      goal.points[idPoint].positions.resize(NB_PUBLISHED_JOINT_HRP2);
      for(unsigned int i=0;i<NB_PUBLISHED_JOINT_HRP2;i++)
        goal.points[idPoint].positions[i]= q_->at(offset_+idPoint*17+i);
    }
  
  // Trajectory publication through ROS
  trajectory_pub_.publish(goal);
  
}
