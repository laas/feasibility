#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <algorithm>

#include <boost/thread/mutex.hpp>
#include "std_msgs/Float64MultiArray.h" //pbulish vector<double>
#include "planner/trajectory_visualizer.h"
#include "planner/motion_generator.h"
#include "planner/contact_transition.h"
#include "rviz/visualmarker.h"

#include "planner/trajectory_footstep.hh"

#define DEBUG(x) 

using namespace std;

FootStepTrajectory::FootStepTrajectory(){
  static bool firsttime = true;
  if(firsttime){
    //tv_ = new TrajectoryVisualizer(0,0,0); //visualize q with CoM offset
    pub_ = n.advertise< std_msgs::Float64MultiArray >(topic.c_str(), 1000);
    firsttime = false;
  }
  last_planner_start_index_=0;
  current_step_index_ = 0;
  number_of_prescripted_steps_ = 0;
  setHalt(false);
}

FootStepTrajectory::FootStepTrajectory( const FootStepTrajectory &rhs ){
  this->footsteps_ = rhs.footsteps_;
  last_planner_start_index_=0;
  current_step_index_ = 0;
}

std::vector<std::vector<double> >& FootStepTrajectory::getFootSteps(){
  return footsteps_;
}
uint FootStepTrajectory::size(){
  return footsteps_.size();
}

ros::Geometry& FootStepTrajectory::getWaist(){
  return tv_->getCoM();
}
ros::Geometry& FootStepTrajectory::getStartEvart(ros::Geometry &evart_com){
  NYI();
  /*

  //CURRENT FOOTSTEP
  ros::Geometry curStep;
  curStep.setX( footsteps_.at(current_step_index_).at(4) );
  curStep.setY( footsteps_.at(current_step_index_).at(5) );
  curStep.setYawRadian( footsteps_.at(current_step_index_).at(6) );
  curStep.setFoot( footsteps_.at(current_step_index_).at(3) );

  //PREVIOUS FOOTSTEP
  ros::Geometry prevStep;
  if(current_step_index_ > 0){
    prevStep.setX( footsteps_.at(current_step_index_ - 1).at(4) );
    prevStep.setY( footsteps_.at(current_step_index_ - 1).at(5) );
    prevStep.setYawRadian( footsteps_.at(current_step_index_ - 1).at(6) );
    prevStep.setFoot( footsteps_.at(current_step_index_ - 1).at(3) );
  }else{
    // we haven't done any steps, therefore there is no drift, therefore we can
    // just return starting position which does not include evart informations
    return getStart();
  }

  // Compute the expected CoM (which we define here as the midpoint between
  // previous step and current step, i.e. cS + 0.5*(pS - cS)
  ros::Geometry expected_com;
  expected_com.setX( curStep.getX() + 0.5*(prevStep.getX() - curStep.getX()) );
  expected_com.setY( curStep.getY() + 0.5*(prevStep.getY() - curStep.getY()) );
  expected_com.setYawRadian( curStep.getYawRadian() - 0.5*(curStep.getYawRadian()-prevStep.getYawRadian()) );

  ros::ArrowMarker a(expected_com.getX(), expected_com.getY(), expected_com.getYawRadian());
  a.publish();
  */
  return this->start_;
}

ros::Geometry& FootStepTrajectory::getStart(){
  ros::Geometry newStart;
  last_planner_start_index_ = current_step_index_ + step_horizon;

  if(last_planner_start_index_ > footsteps_.size()-1){
    last_planner_start_index_ = footsteps_.size()-1;
  }
  if(footsteps_.size()==0){
    //no footsteps aquired yet, so we do not update the starting position
    return this->start_;
  }

  newStart.setX( footsteps_.at(last_planner_start_index_).at(4) );
  newStart.setY( footsteps_.at(last_planner_start_index_).at(5) );
  newStart.setYawRadian( footsteps_.at(last_planner_start_index_).at(6) );
  char flast = footsteps_.at(last_planner_start_index_).at(3);
  newStart.setFoot( flast=='R'?'L':'R');

  DEBUG(cout << "SETTING NEW START VALUE" << endl;)
  DEBUG( cout<<footsteps_.at(last_planner_start_index_) <<endl;)
  this->setStart( newStart );
  return this->start_;
}
void FootStepTrajectory::setStart( ros::Geometry &start ){
  this->start_ = start;
  this->start_.setFoot( start.getFoot() );
}
//publish a joint configuration trajectory and visualize it in RVIZ
void FootStepTrajectory::execute_one_step(){
  boost::mutex::scoped_lock lock(footstep_mutex_);

  if(footsteps_.size()<1){
    //No footsteps avaiable
    return;
  }

  if(onHalt()){
    return;
  }

  if( current_step_index_ > footsteps_.size()-1 ){
    return;
  }
  if( current_step_index_ > last_planner_start_index_ ){
    if(current_step_index_ < footsteps_.size() - number_of_prescripted_steps_){
      //outside of prescripted area, and after start position of planner (lets
      //wait for next plan)
      return;
    }
  }

  publish();

  MotionGenerator *mg;
  std::vector<double> q;

  mg = new MotionGenerator(ContactTransition::objects); //generate q

  q =  mg->generateWholeBodyMotionFromAbsoluteFootsteps(footsteps_, current_step_index_, 0, 0.19, 0, 
      footsteps_.at(current_step_index_).at(3)); //where is the right foot wrt the left foot (relative)

  if(q.size()>0){
    //ROS_INFO("configuration vector: %d", q.size());
    //Replay trajectory
    tv_->init(q);
    ros::Rate rq(400); //300
    while(tv_->next())
      {
        //ros::spinOnce();
        //rq.sleep();
      }
  }

  //boost::mutex::scoped_lock lock2(footstep_mutex_);
  current_step_index_++;
  ROS_INFO("step_index: %d -> %d (%d)", current_step_index_-1, current_step_index_, size());
  return;
}
//only call this function, if you know what you are doing
void FootStepTrajectory::execute_one_step_fast_not_thread_safe(){
  if(footsteps_.size()<1){
    return;
  }
  publish();

  MotionGenerator *mg;
  std::vector<double> q;
  mg = new MotionGenerator(ContactTransition::objects); //generate q

  q =  mg->generateWholeBodyMotionFromAbsoluteFootsteps(footsteps_, current_step_index_, 0, 0.19, 0, 
      footsteps_.at(current_step_index_).at(3)); //where is the right foot wrt the left foot (relative)

  if(q.size()>0){
    //ROS_INFO("configuration vector: %d", q.size());
    //Replay trajectory
    tv_->init(q);
    //ros::Rate rq(400); //300
    while(tv_->next())
    {
      ros::spinOnce();
    }
  }
  current_step_index_++;
  ROS_INFO("step_index: %d -> %d (%d)", current_step_index_-1, current_step_index_, size());
  return;

}
void FootStepTrajectory::lock(){
  footstep_mutex_.lock();
}
void FootStepTrajectory::unlock(){
  footstep_mutex_.unlock();
}

// append a new trajectory from the planner to the current trajectory, thereby
// making sure that they are consistent
void FootStepTrajectory::append( ros::Geometry &start, FootStepTrajectory &rhs ){
    this->setStart(start);

    if(footsteps_.size()==0){
      footsteps_ = rhs.footsteps_;
      number_of_prescripted_steps_ = rhs.number_of_prescripted_steps_;
      DEBUG( ROS_INFO("TV START %f %f %f", start.getX(), start.getY(), start.getYawRadian()); )
      tv_ = new TrajectoryVisualizer(start.getX(), start.getY(), start.getYawRadian()); //visualize q with CoM offset
      return;
    }
    if(rhs.size()==0){ 
      //nothing to append
      return;
    }
    DEBUG(ROS_INFO("SIZE: %d - %d / %d+%d",footsteps_.size(), number_of_prescripted_steps_, current_step_index_, step_horizon);)
    DEBUG(ROS_INFO("APPEND %d and %d (prescript %d and %d)", footsteps_.size(), rhs.size(), number_of_prescripted_steps_, rhs.number_of_prescripted_steps_);)
    /*
    if(footsteps_.size()-number_of_prescripted_steps_ < current_step_index_ +  1 + step_horizon){
      //in final prescript sequence, do not disturb the trajectory
      return;

    }
    */

    uint collisionStartIndex = std::max(current_step_index_, (uint)1);

    if( !ContactTransition::isInCollision(footsteps_, collisionStartIndex ,footsteps_.size()-number_of_prescripted_steps_ ) ){ //last two steps are prescripted
      if(footsteps_.size() - last_planner_start_index_ <= rhs.size()){
        // No need for updating
        return;
      }
    }


    if(current_step_index_ + step_horizon >= footsteps_.size() - last_planner_start_index_ + rhs.footsteps_.size()){
      //goal is reached in the next three steps, so we forget about the new
      //trajectory
      DEBUG(ROS_INFO("Reaching End of Trajectory in %d steps" ,step_horizon);)
      return;
    }

    //*********************************************************
    //check if start position and last step are consistent
    //*********************************************************
    double xs = this->start_.getX();
    double ys = this->start_.getY();
    double ts = this->start_.getYawRadian();
    char fs = this->start_.getFoot();
    double x = footsteps_.at(last_planner_start_index_).at(4);
    double y = footsteps_.at(last_planner_start_index_).at(5);
    double t = footsteps_.at(last_planner_start_index_).at(6);
    char f =   footsteps_.at(last_planner_start_index_).at(3);
    double d= norml2(x,xs,y,ys)+sqrtf((ts-t)*(ts-t));
    if(d<0.00001){

    }else{
      ROS_INFO("*************************************");
      ROS_INFO("[FATAL_ERROR] Could not connect replanned path to old path");
      ROS_INFO("*************************************");
      cout << footsteps_ << endl;
      cout << rhs.footsteps_ << endl;
      this->start_.print();
      ROS_INFO("FOOT: %f %f %f VS. START: %f %f %f", x,y,t,xs,ys,ts);
      cout << this->start_.getYawRadian() << endl;
      ros::FootMarker m(0,0,0);
      m.reset();
      exit(-1);
    }
      
    if(footsteps_.at(last_planner_start_index_).at(3) == rhs.footsteps_.at(0).at(3)){
      ROS_INFO("*************************************");
      ROS_INFO("[FATAL_ERROR] NOT SAME FOOT");
      ROS_INFO("*************************************");
      footsteps_.erase( footsteps_.begin()+last_planner_start_index_+1, footsteps_.end());
      cout << footsteps_ << endl;
      cout << endl;
      cout << rhs.footsteps_.at(0) << endl;
      cout << rhs.footsteps_.at(1) << endl;
      cout << rhs.footsteps_.at(2) << endl;
      this->start_.print();
      cout << this->start_.getYawRadian() << endl;
      ROS_INFO("LAST PLANNER INDEX: %d", last_planner_start_index_);
      cout << footsteps_.at(last_planner_start_index_) << endl;
      ros::FootMarker m(0,0,0);
      m.reset();
      exit(-1);
    }
    //*********************************************************
    //*********************************************************

    //everything seems fine, lets concatenate the two trajectories
    number_of_prescripted_steps_ = rhs.number_of_prescripted_steps_;
    footsteps_.erase( footsteps_.begin()+last_planner_start_index_+1, footsteps_.end());
    footsteps_.insert( footsteps_.end(), rhs.footsteps_.begin(), rhs.footsteps_.end() );

    //set START to the third step from now
}

//publish the whole trajectory, starting at the current_step_index
// additional features:
//  -- mark the next starting position for the planner as blue
//  -- publish steps as rostopic
void FootStepTrajectory::publish(){
  if(footsteps_.size()==0){
    return;
  }
  if(current_step_index_ >= footsteps_.size()){
    return;
  }
  ros::FootMarker l(0,0,0);
  l.reset();
  for(uint i=current_step_index_;i<footsteps_.size();i++){
    double x = footsteps_.at(i).at(4);
    double y = footsteps_.at(i).at(5);
    double t = footsteps_.at(i).at(6);
    char f = footsteps_.at(i).at(3);

    if(f == 'L'){
      ros::ColorFootMarker m(x,y,t,colorLeft);
      m.publish();
    }else{
      ros::ColorFootMarker m(x,y,t,colorRight);
      m.publish();
    }
  }

  if(last_planner_start_index_<footsteps_.size()){
    double x = footsteps_.at(last_planner_start_index_).at(4);
    double y = footsteps_.at(last_planner_start_index_).at(5);
    double t = footsteps_.at(last_planner_start_index_).at(6);
    double f = footsteps_.at(last_planner_start_index_).at(3);

    //ros::ColorFootMarker m(x,y,t,"blue");
    //m.g.setSX(0.28); //0.24
    //m.g.setSY(0.15); //0.14
    //m.g.setSZ(0.08); //0.03
    //m.init_marker();
    //m.publish();
  }
  //ROS_INFO("step %d/%d", current_step_index_, fsi.size());

  //publish footsteps over ros topic
  std_msgs::Float64MultiArray ros_steps;
  ros_steps.layout.dim.push_back(std_msgs::MultiArrayDimension());
  ros_steps.layout.dim.push_back(std_msgs::MultiArrayDimension());
  ros_steps.layout.dim[0].label = "step";
  ros_steps.layout.dim[0].size = footsteps_.size();
  ros_steps.layout.dim[0].stride = footsteps_.size()*footsteps_.at(0).size();
  ros_steps.layout.dim[1].label = "relative/absolute";
  ros_steps.layout.dim[1].size = footsteps_.at(0).size();
  ros_steps.layout.dim[1].stride = footsteps_.at(0).size();

  ros_steps.data.resize( footsteps_.at(0).size() * footsteps_.size() );

  for(uint i=0;i<footsteps_.size();i++){
    for(uint j=0;j<footsteps_.at(0).size();j++){
      ros_steps.data[i*footsteps_.at(0).size()+j]=( footsteps_.at(i).at(j) );
    }
  }

  pub_.publish(ros_steps);

}

void FootStepTrajectory::push_back( FootStepState &fss ){
  footsteps_.push_back(fss);
}

void FootStepTrajectory::pop_back(){
  footsteps_.pop_back();
}

uint FootStepTrajectory::getCurrentStepIndex(){
  return current_step_index_;
}

void FootStepTrajectory::setHalt(bool b){
  halt_ = b;
}

bool FootStepTrajectory::onHalt(){
  return halt_;
}
bool FootStepTrajectory::isFinished(){
  if( current_step_index_ >= footsteps_.size()-5 && current_step_index_ > 0){
    setHalt(true);
    return true;
  }else{
    setHalt(false);
    return false;
  }
}

void FootStepTrajectory::checkSafety( double &xr, double &yr, double &tr){
  while(tr>M_PI) tr-=2*M_PI;
  while(tr<-M_PI) tr+=2*M_PI;
  //security such that feet are not colliding
  double xr_real = xr;
  double yr_real = yr;

  while(abs(xr)<0.28 && abs(yr)<0.16){ //w=0.24,l=0.12
    //ROS_INFO("xr %f, yr %f || %f %f", xr, yr, xr_real, yr_real);
    xr = xr + 0.01*xr_real;
    yr = yr + 0.01*yr_real;
  }
}
//move the robot towards the exact position, thereby assuming that it is already
//in the goal region, i.e. it can make the steps which we prescript
void FootStepTrajectory::add_prescripted_end_sequence(const ros::Geometry &goal){
  if(footsteps_.size()>1){
    //footsteps_.pop_back(); //delete last element (is predefined goal positon and does not belong to the trajectory)

    const double goal_offset_x = -0.1;
    const double goal_offset_y = -0.1; //offset in L direction
    const double step_y = 0.2; //distance between feet in half-sitting
    char sf_f = footsteps_.at( footsteps_.size() -1 ).at(3);

    if(sf_f == 'R'){
      if(footsteps_.size()==1){
        //add new footstep to make L/R prescript possible
        double xl = footsteps_.at( footsteps_.size() -1 ).at(4);
        double yl = footsteps_.at(footsteps_.size() -1 ).at(5);
        double yawl = footsteps_.at(footsteps_.size() -1 ).at(6);
        double abs_x = cos(yawl)*(0) - sin(yawl)*(step_y) + xl;
        double abs_y = sin(yawl)*(0) + cos(yawl)*(step_y) + yl;
        std::vector<double> pre_script_foot0 = vecD(0, -step_y , 0, 'L', abs_x, abs_y, yawl);
        footsteps_.push_back(pre_script_foot0);
      }else{
        footsteps_.pop_back();
      }

      sf_f = footsteps_.at( footsteps_.size() -1 ).at(3);
    }
    uint N_planned_steps = footsteps_.size();

    double goal_x = goal.getX();
    double goal_y = goal.getY();
    double yawg = goal.getYawRadian();

    double xl = footsteps_.at( footsteps_.size() -1 ).at(4);
    double yl = footsteps_.at(footsteps_.size() -1 ).at(5);
    double yawl = footsteps_.at(footsteps_.size() -1 ).at(6);

    double abs_x = cos(yawl)*(0) - sin(yawl)*(step_y) + xl;
    double abs_y = sin(yawl)*(0) + cos(yawl)*(step_y) + yl;

    //*******************************************************
    // move to half sitting
    //*******************************************************
    std::vector<double> pre_script_foot0 = vecD(0, -step_y , 0, 'R', abs_x, abs_y, yawl);
    footsteps_.push_back(pre_script_foot0);

    //*******************************************************
    //rotate robot on the spot
    //*******************************************************
    double new_angle = yawl;
    const double max_rotate_angle = 10.0*M_PI/180.0;//M_PI/4; //rotate no more than radians per step

    double last_step_x = abs_x;
    double last_step_y = abs_y;
    double last_step_t = yawl;

    //make sure that the robot is walking into the shortest direction
    if(fabs(new_angle-yawg) > M_PI){
      if(new_angle > yawg){
        yawg+=2*M_PI;
      }else{
        new_angle+=2*M_PI;
      }
    }

    while( fabs(new_angle - yawg) > 0.01 ){
      int sign = (yawg - new_angle)/fabs(yawg-new_angle); //+1 or -1 rotation direction
      double angle_offset = sign*min( fabs(yawg - new_angle), max_rotate_angle );
      new_angle += angle_offset;
      DEBUG(ROS_INFO("new angle: %f (start: %f, goal: %f)", new_angle, yawl, yawg);)

      double new_step_x = cos(last_step_t)*(0) - sin(last_step_t)*(-step_y) + last_step_x;
      double new_step_y = sin(last_step_t)*(0) + cos(last_step_t)*(-step_y) + last_step_y;
      double new_step_t = new_angle;

      std::vector<double> pre_script_foot1 = vecD(0, -step_y , angle_offset, 'L', new_step_x, new_step_y, new_step_t);
      footsteps_.push_back(pre_script_foot1);

      new_step_x = cos(new_angle)*(0) - sin(new_angle)*(step_y) + new_step_x;
      new_step_y = sin(new_angle)*(0) + cos(new_angle)*(step_y) + new_step_y;
      new_step_t = new_angle;

      std::vector<double> pre_script_foot2 = vecD(0, -step_y , 0, 'R', new_step_x, new_step_y, new_step_t);
      footsteps_.push_back(pre_script_foot2);
      last_step_x = new_step_x;
      last_step_y = new_step_y;
      last_step_t = new_step_t;
    }
    //*******************************************************
    // move robot to precise position
    //*******************************************************
    const double max_step_distance = 0.09; //m of foot movement

    while( norml2(last_step_x, goal_x, last_step_y, goal_y) > 0.01 ){
      double vx = goal_x - last_step_x;
      double vy = goal_y - last_step_y;

      double lambda = norml2(vx,0,vy,0);
      if(lambda>max_step_distance){
        lambda=max_step_distance;
      }

      vx/=sqrtf(vx*vx+vy*vy); //normed
      vy/=sqrtf(vx*vx+vy*vy);
      double dx = cos(yawg)*0 - sin(yawg)*(-step_y);
      double dy = cos(yawg)*0 + cos(yawg)*(-step_y);
      double new_step_x_abs = vx*lambda + last_step_x + dx;
      double new_step_y_abs = vy*lambda + last_step_y + dy;

      double xr = cos(-yawg)*(dx + vx*lambda) - sin(-yawg)*(dy + vy*lambda);
      double yr = sin(-yawg)*(dx + vx*lambda) + cos(-yawg)*(dy + vy*lambda);

      std::vector<double> pre_script_foot1 = vecD(xr, yr, 0, 'L', new_step_x_abs, new_step_y_abs, yawg);
      footsteps_.push_back(pre_script_foot1);

      //*******************************************************
      // move second foot to half sitting with respect to L foot
      //*******************************************************
      dx = cos(yawg)*0 - sin(yawg)*(step_y);
      dy = cos(yawg)*0 + cos(yawg)*(step_y);

      new_step_x_abs = dx + new_step_x_abs;
      new_step_y_abs = dy + new_step_y_abs;

      std::vector<double> pre_script_foot2 = vecD(0, -step_y , 0, 'R', new_step_x_abs, new_step_y_abs, yawg);
      footsteps_.push_back(pre_script_foot2);


      last_step_x = new_step_x_abs;
      last_step_y = new_step_y_abs;
    }
      /*
      double goal_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(-goal_offset_y) + goal_x;
      double goal_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(-goal_offset_y) + goal_y;
      double goal_offset_in_right_foot_space_x = cos(-yawl)*(goal_offset_abs_x-xl) - sin(-yawl)*(goal_offset_abs_y-yl);
      double goal_offset_in_right_foot_space_y = sin(-yawl)*(goal_offset_abs_x-xl) + cos(-yawl)*(goal_offset_abs_y-yl);
      double xr = goal_offset_in_right_foot_space_x;
      double yr = goal_offset_in_right_foot_space_y;
      double tr = (yawl - yawg);

      checkSafety(xr, yr, tr);

      std::vector<double> pre_script_foot = vecD(xr, -yr , tr, 'R', goal_offset_abs_x, goal_offset_abs_y, yawg);
      footsteps_.push_back(pre_script_foot);

      double goal_left_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y+step_y) + goal_x;
      double goal_left_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y+step_y) + goal_y;

      std::vector<double> pre_script_foot2 = vecD(0, -step_y, 0, 'L', goal_left_offset_abs_x,goal_left_offset_abs_y-step_y, yawg);
      footsteps_.push_back(pre_script_foot2);
      */
        /*
#define DEBUG(x) 
      while( norml2(last_step_x, goal_x, last_step_y, goal_y) > 0.01 ){
        DEBUG(ROS_INFO("DIST: %f", norml2(last_step_x, goal_x, last_step_y, goal_y));)
        DEBUG(ROS_INFO("STEP %f %f", last_step_x, last_step_y);)

        double vx = goal_x - last_step_x;
        double vy = goal_y - last_step_y;

        DEBUG(ROS_INFO("DIR %f %f", vx, vy);)
        double lambda = norml2(vx,0,vy,0);

        if(lambda>max_step_distance){
          lambda=max_step_distance;
        }
        vx/=sqrtf(vx*vx+vy*vy); //normed
        vy/=sqrtf(vx*vx+vy*vy);
        DEBUG(ROS_INFO("DIR %f %f", vx, vy);)
        DEBUG(ROS_INFO("LAMBDA %f", lambda);)

        double dx = cos(yawg)*0 - sin(yawg)*(-step_y);
        double dy = cos(yawg)*0 + cos(yawg)*(-step_y);
        double new_step_x_abs = dx + last_step_x;
        double new_step_y_abs = dy + last_step_y;

        double new_step_x = last_step_x + vx*lambda;
        double new_step_y = last_step_y + vy*lambda;

        DEBUG(ROS_INFO("STEP %f %f", last_step_x, last_step_y);)
        DEBUG(ROS_INFO("DIST: %f", norml2(new_step_x, goal_x, new_step_y, goal_y));)

        double xr = cos(-yawg)*(new_step_x_abs-last_step_x) - sin(-yawg)*(new_step_y_abs-last_step_y);
        double yr = sin(-yawg)*(new_step_x_abs-last_step_x) + cos(-yawg)*(new_step_y_abs-last_step_y);
      //double goal_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(-goal_offset_y) + goal_x;
      //double goal_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(-goal_offset_y) + goal_y;
      //double goal_offset_in_right_foot_space_x = cos(-yawl)*(goal_offset_abs_x-xl) - sin(-yawl)*(goal_offset_abs_y-yl);
      //double goal_offset_in_right_foot_space_y = sin(-yawl)*(goal_offset_abs_x-xl) + cos(-yawl)*(goal_offset_abs_y-yl);

        std::vector<double> pre_script_foot1 = vecD(xr, yr, 0, 'L', new_step_x_abs, new_step_y_abs, yawg);
        footsteps_.push_back(pre_script_foot1);

        //*******************************************************
        // move second foot to half sitting with respect to L foot
        //*******************************************************

        dx = cos(yawg)*0 - sin(yawg)*(step_y);
        dy = cos(yawg)*0 + cos(yawg)*(step_y);

        new_step_x_abs = dx + new_step_x_abs;
        new_step_y_abs = dy + new_step_y_abs;

        std::vector<double> pre_script_foot2 = vecD(0, -step_y , 0, 'R', new_step_x_abs, new_step_y_abs, yawg);
        footsteps_.push_back(pre_script_foot2);


        last_step_x = new_step_x;
        last_step_y = new_step_y;
      }
      */

      /*
      while( fabs(new_angle - yawg) > 0.01 ){
        int sign = (yawg - new_angle)/fabs(yawg-new_angle); //+1 or -1 rotation direction

        double angle_offset = sign*min( fabs(yawg - new_angle), max_rotate_angle );
        new_angle += angle_offset;

        double new_step_x = cos(new_angle)*(0) - sin(-new_angle)*(step_y) + last_step_x;
        double new_step_y = sin(new_angle)*(0) + cos(-new_angle)*(step_y) + last_step_y;
        double new_step_t = new_angle;

        std::vector<double> pre_script_foot1 = vecD(0, -step_y , angle_offset, 'L', new_step_x, new_step_y, new_step_t);
        footsteps_.push_back(pre_script_foot1);

        new_step_x = cos(new_angle)*(0) - sin(-new_angle)*(step_y) + new_step_x;
        new_step_y = sin(new_angle)*(0) + cos(-new_angle)*(step_y) + new_step_y;
        new_step_t = new_angle;

        std::vector<double> pre_script_foot2 = vecD(0, -step_y , 0, 'R', new_step_x, new_step_y, new_step_t);
        footsteps_.push_back(pre_script_foot2);

        last_step_x = new_step_x;
        last_step_y = new_step_y;
        last_step_t = new_step_t;
        ROS_INFO("added prescripted FOOTSTEP %f %f %f", new_step_x, new_step_y, new_step_t);

      }

      //*/


      /*
      double goal_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(-goal_offset_y) + goal_x;
      double goal_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(-goal_offset_y) + goal_y;
      double goal_offset_in_right_foot_space_x = cos(-yawl)*(goal_offset_abs_x-xl) - sin(-yawl)*(goal_offset_abs_y-yl);
      double goal_offset_in_right_foot_space_y = sin(-yawl)*(goal_offset_abs_x-xl) + cos(-yawl)*(goal_offset_abs_y-yl);
      double xr = goal_offset_in_right_foot_space_x;
      double yr = goal_offset_in_right_foot_space_y;
      double tr = (yawl - yawg);

      checkSafety(xr, yr, tr);

      std::vector<double> pre_script_foot = vecD(xr, -yr , tr, 'R', goal_offset_abs_x, goal_offset_abs_y, yawg);
      footsteps_.push_back(pre_script_foot);

      double goal_left_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y+step_y) + goal_x;
      double goal_left_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y+step_y) + goal_y;

      std::vector<double> pre_script_foot2 = vecD(0, -step_y, 0, 'L', goal_left_offset_abs_x,goal_left_offset_abs_y-step_y, yawg);
      footsteps_.push_back(pre_script_foot2);
      */

    number_of_prescripted_steps_ = footsteps_.size() - N_planned_steps;

  }

}
