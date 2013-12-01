#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/thread/mutex.hpp>
#include "std_msgs/Float64MultiArray.h" //pbulish vector<double>
#include "planner/trajectory_visualizer.h"
#include "planner/motion_generator.h"
#include "planner/contact_transition.h"
#include "rviz/visualmarker.h"

#include "planner/trajectory_footstep.hh"

#define DEBUG(x) x

using namespace std;

FootStepTrajectory::FootStepTrajectory(){
  static bool firsttime = true;
  if(firsttime){
    tv_ = new TrajectoryVisualizer(0,0,0); //visualize q with CoM offset
    pub_ = n.advertise< std_msgs::Float64MultiArray >(topic.c_str(), 1000);
    firsttime = false;
  }
  current_step_index_ = 0;
}

FootStepTrajectory::FootStepTrajectory( const FootStepTrajectory &rhs ){
  this->footsteps_ = rhs.footsteps_;
  current_step_index_ = 0;
}

std::vector<std::vector<double> >& FootStepTrajectory::getFootSteps(){
  return footsteps_;
}

ros::Geometry& FootStepTrajectory::getStart(){
  //ros::Geometry g;
  //g.setX( this->start__.at(4) );
  //g.setY( this->start__.at(5) );
  //g.setYawRadian( this->start__.at(6) );
  //g.setFoot( this->start__.at(3) );
  //return g;
  return this->start_;
}
void FootStepTrajectory::setStart( ros::Geometry &start ){
  this->start_ = start;
}
//publish a joint configuration trajectory and visualize it in RVIZ
void FootStepTrajectory::execute_one_step(){
  boost::mutex::scoped_lock lock(footstep_mutex_);

  if(footsteps_.size()<1){
    return;
  }
  if(current_step_index_ >= footsteps_.size()-1){
    return;
  }

  MotionGenerator *mg;
  std::vector<double> q;

  if( ContactTransition::isInCollision(footsteps_, current_step_index_ ) ){
    //ROS_INFO("********************************");
    //ROS_INFO("COLLISION WARNING");
    //ROS_INFO("********************************");
    lock.unlock();
    //ros::Rate rq(1);
    //rq.sleep();
    return;
  }


  mg = new MotionGenerator(ContactTransition::objects); //generate q
  q =  mg->generateWholeBodyMotionFromAbsoluteFootsteps(footsteps_, current_step_index_, 0, 0.19, 0, 'R'); //where is the right foot wrt the left foot (relative)

  lock.unlock();
  if(q.size()>0){
    ROS_INFO("configuration vector: %d", q.size());
    //Replay trajectory
    tv_->init(q);
    ros::Rate rq(400); //300
    while(tv_->next()){
      ros::spinOnce();
      rq.sleep();
    }
  }

  current_step_index_++;
  ROS_INFO("step_index: %d -> %d", current_step_index_-1, current_step_index_);
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
      DEBUG( ROS_INFO("TV START %f %f %f", start.getX(), start.getY(), start.getYawRadian()); )
      tv_ = new TrajectoryVisualizer(start.getX(), start.getY(), start.getYawRadian()); //visualize q with CoM offset
      return;
    }

    if( !ContactTransition::isInCollision(footsteps_, current_step_index_ ) ){
      // No need for updating
      return;
    }



    uint step_horizon = 3;
    //if(!environment_changed && success()){
			//ROS_INFO("Unchanged environment -> reusing old trajectory");
			//return;
		//}

    if(current_step_index_ + step_horizon >= footsteps_.size() + rhs.footsteps_.size()){
      //goal is reached in the next three steps, 
      return;
    }

    //*********************************************************
    //check which step in fsi is the fsi start position
    //*********************************************************
    bool found = false;

    int fsi_last_element = -1;
    int fsi_new_first_element = 0;
    double xs = this->start_.getX();
    double ys = this->start_.getY();
    double ts = this->start_.getYawRadian();
    char fs = this->start_.getFoot();
    for(uint i = 0 ; i< footsteps_.size() ; i++){
      double x = footsteps_.at(i).at(4);
      double y = footsteps_.at(i).at(5);
      double t = footsteps_.at(i).at(6);
      char f = footsteps_.at(i).at(3);
      double d= norml2(x,xs,y,ys)+sqrtf((ts-t)*(ts-t));
      //ROS_INFO("FOOT: %f %f %f VS. START: %f %f %f", x,y,t,xs,ys,ts);
      if(d<0.001){
        fsi_last_element = i;
        /*
        for(uint j = 0; j< fsi_new.size() ; j++){
          double xn = fsi_new.at(j).at(4);
          double yn = fsi_new.at(j).at(5);
          double tn = fsi_new.at(j).at(6);
          char fn = fsi_new.at(j).at(3);
          double d= norml2(x,xn,y,yn)+sqrtf((tn-t)*(tn-t));
          if(d<0.001 && f==fn){
            fsi_last_element = i;
            fsi_new_first_element = j;
            found = true;
            break;
          }
        }
        */
        if(footsteps_.at(fsi_last_element).at(3) == rhs.footsteps_.at(fsi_new_first_element).at(3)){
          ROS_INFO("****************************************");
          ROS_INFO("****************************************");
          ROS_INFO("****************************************");
          ROS_INFO("****************************************");
          ROS_INFO("CHANGED FSI_NEW");
          ROS_INFO("****************************************");
          ROS_INFO("****************************************");
          ROS_INFO("****************************************");
          ROS_INFO("****************************************");
          this->start_.print();
          rhs.footsteps_.erase( rhs.footsteps_.begin() );
        }
        found = true;
        break;
      }
    }
    if(!found){
      ROS_INFO("*************************************");
      ROS_INFO("[FATAL_ERROR] Could not connect replanned path to old path");
      ROS_INFO("*************************************");
      footsteps_.erase( footsteps_.begin()+fsi_last_element+1, footsteps_.end());
      cout << footsteps_ << endl;
      cout << rhs.footsteps_ << endl;
      this->start_.print();
      cout << this->start_.getYawRadian() << endl;
      ros::FootMarker m(0,0,0);
      m.reset();
      exit(-1);
    }
      
    /*
    for(uint j = 0; j< fsi_new.size() ; j++){
      double xn = fsi_new.at(j).at(4);
      double yn = fsi_new.at(j).at(5);
      double tn = fsi_new.at(j).at(6);
      char fn = fsi_new.at(j).at(3);
      double d= norml2(x,xn,y,yn)+sqrtf((tn-t)*(tn-t));
      if(d<0.01 && f==fn){
        fsi_last_element = i;
        fsi_new_first_element = j;
        found = true;
        break;
      }
    }
      if(found) break;
    }
  */
    //for(uint i=0;i<=fsi_last_element;i++){
      //fsi.at(i)=fsi.at(i);
    //}
      ROS_INFO("*************************************");
      ROS_INFO("%d/%d --> %d/%d", fsi_last_element, footsteps_.size(), fsi_new_first_element, rhs.footsteps_.size());
      ROS_INFO("*************************************");

      if(footsteps_.at(fsi_last_element).at(3) == rhs.footsteps_.at(fsi_new_first_element).at(3)){

        ROS_INFO("*************************************");
        ROS_INFO("[FATAL_ERROR] SAME FOOT");
        ROS_INFO("*************************************");
        footsteps_.erase( footsteps_.begin()+fsi_last_element+1, footsteps_.end());
        cout << footsteps_ << endl;
        cout << rhs.footsteps_ << endl;
        this->start_.print();
        cout << this->start_.getYawRadian() << endl;
        ros::FootMarker m(0,0,0);
        m.reset();
        exit(-1);
      }

    footsteps_.erase( footsteps_.begin()+fsi_last_element+1, footsteps_.end());
    footsteps_.insert( footsteps_.end(), rhs.footsteps_.begin()+fsi_new_first_element, rhs.footsteps_.end() );
    //fsi.erase(min(fsi.begin() + current_step_index_ + step_horizon, fsi.end()), fsi.end());
    //fsi.insert( fsi.end(), fsi_new.begin(), fsi_new.end() );

    //set START to the third step from now
    //

    ros::Geometry newStart;
    uint start_index = current_step_index_ + step_horizon;
    newStart.setX( footsteps_.at(start_index).at(4) );
    newStart.setY( footsteps_.at(start_index).at(5) );
    newStart.setYawRadian( footsteps_.at(start_index).at(6) );
    newStart.setFoot( footsteps_.at(start_index).at(3) );
    this->setStart( newStart );


}

//publish the whole trajectory, starting at the current_step_index
// additional features:
//  -- mark the next starting position for the planner as blue
//  -- publish steps as rostopic
void FootStepTrajectory::publish(){
  if(current_step_index_ >= footsteps_.size()){
    //ROS_INFO("Finished trajectory");
    return;
  }
  ros::FootMarker l(0,0,0);
  l.reset();
  for(uint i=current_step_index_;i<footsteps_.size();i++){
    double x = footsteps_.at(i).at(4);
    double y = footsteps_.at(i).at(5);
    double t = footsteps_.at(i).at(6);
    double f = footsteps_.at(i).at(3);

    if(f == 'L'){
      ros::ColorFootMarker m(x,y,t,colorLeft);
      m.publish();
    }else{
      ros::ColorFootMarker m(x,y,t,colorRight);
      m.publish();
    }
  }
  //update start pos for further replanning
  uint i=current_step_index_ + 3;

  if(i<footsteps_.size()){
    double x = footsteps_.at(i).at(4);
    double y = footsteps_.at(i).at(5);
    double t = footsteps_.at(i).at(6);
    char f = footsteps_.at(i).at(3);
    this->start_.setX(x);
    this->start_.setY(y);
    this->start_.setRPYRadian(0,0,t);
    //ROS_INFO("SET START: %f %f %f", x,y,t);
    this->start_.setFoot( (f=='R'?'L':'R') ); //starting pos is omitted
    ros::ColorFootMarker m(x,y,t,"blue");
    m.g.setSX(0.28); //0.24
    m.g.setSY(0.15); //0.14
    m.g.setSZ(0.08); //0.03
    m.init_marker();
    m.publish();
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
    footsteps_.pop_back(); //delete last element (is predefined goal positon and does not belong to the trajectory)

    char sf_f = footsteps_.at( footsteps_.size() -1 ).at(3);

    double goal_x = goal.getX();
    double goal_y = goal.getY();
    double yawg = goal.getYawRadian();

    double xl = footsteps_.at( footsteps_.size() -1 ).at(4);
    double yl = footsteps_.at(footsteps_.size() -1 ).at(5);
    double yawl = footsteps_.at(footsteps_.size() -1 ).at(6);

    double goal_offset_x = -0.1;
    double goal_offset_y = -0.1; //offset in L direction
    double step_y = 0.2; //distance between feet in half-sitting

    if(sf_f == 'L'){
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

      std::vector<double> pre_script_foot2 = vecD(0, -step_y, 0, 'L', goal_left_offset_abs_x,goal_left_offset_abs_y, yawg);
      footsteps_.push_back(pre_script_foot2);
    }else{
      double goal_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y) + goal_x;
      double goal_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y) + goal_y;
      double goal_offset_in_left_foot_space_x = cos(-yawl)*(goal_offset_abs_x-xl) - sin(-yawl)*(goal_offset_abs_y-yl);
      double goal_offset_in_left_foot_space_y = sin(-yawl)*(goal_offset_abs_x-xl) + cos(-yawl)*(goal_offset_abs_y-yl);

      double xr = goal_offset_in_left_foot_space_x;
      double yr = goal_offset_in_left_foot_space_y;
      double tr = yawg-yawl;

      checkSafety(xr, yr, tr);

      std::vector<double> pre_script_foot = vecD(xr, yr , tr, 'L', goal_offset_abs_x, goal_offset_abs_y, yawg);
      footsteps_.push_back(pre_script_foot);
      double goal_left_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y+step_y) + goal_x;
      double goal_left_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y+step_y) + goal_y;

      std::vector<double> pre_script_foot2 = vecD(0, -step_y, 0, 'R', goal_left_offset_abs_x,goal_left_offset_abs_y, yawg);
      footsteps_.push_back(pre_script_foot2);
    }


  }

}