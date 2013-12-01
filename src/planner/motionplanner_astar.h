#pragma once
#include "planner/motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "planner/contact_transition.h"
#include "util/util_timer.h"
#include "planner/trajectory_footstep.hh"

#define DEBUG(x) x

struct MotionPlannerAStar: public MotionPlanner{

  static ContactTransition goal;
  static ContactTransition start;
  static AStarSearch<ContactTransition> *astarsearch;

  //static double sf_x, sf_y, sf_t;
  //static double init_sf_x, init_sf_y, init_sf_t;
  //static char sf_f;
  //static uint current_step_index_;

  //to publish footsteps on ros topic
        
 MotionPlannerAStar(Environment *env, int &argc, char** &argv): MotionPlanner(env){
    
    DEBUG(ROS_INFO("***** START A_STAR ********");)
                  
    ContactTransition::cleanStatic();
    astarsearch = new AStarSearch<ContactTransition>(10000);
    if(ContactTransition::timer!=NULL){
      delete ContactTransition::timer;
    }
    ContactTransition::timer = new Timer();
    ContactTransition::timer->register_stopper("prepare", "prepare objects");
    ContactTransition::timer->register_stopper("loader", "Loading prerequisite structures");
    ContactTransition::timer->register_stopper("actionExpansion", "compute actionExpansion");
    ContactTransition::timer->register_stopper("ff", "compute ff transformation");
    ContactTransition::timer->register_stopper("a*", "a* algorithm");
    ContactTransition::feasibilityChecks=0;
    //current_step_index_=0;
    results.success=false;

  }
  ~MotionPlannerAStar(){
    DEBUG(ROS_INFO("***** DELETE A_STAR ********");)
    astarsearch->EnsureMemoryFreed();
    if(astarsearch!=NULL) delete astarsearch;
    astarsearch=NULL;
  }
  void setConstraintsChecker( ConstraintsChecker *constraints ){
    ContactTransition::setConstraintsChecker( constraints );
  }
  ros::Geometry& getGoal(){
    return this->goal.g;
  }
  ros::Geometry& getStart(){
    return this->start.g;
  }
  void setGoal( ros::Geometry &goal ){
    this->goal.g = goal;
  }
  void setStart( ros::Geometry &start ){
    this->start.g = start;
    this->start.rel_x_parent = 0;
    this->start.rel_y_parent = 0;
    this->start.rel_yaw_parent = 0;//start.getYawRadian();
    this->start.L_or_R = 'L';
    this->start.g.setFoot(start.getFoot());
  }
  void setGoal( double x, double y, double yaw_rad, char foot){
    this->goal.g.setX(x);
    this->goal.g.setY(y);
    this->goal.g.setYawRadian(yaw_rad);
    this->goal.g.setFoot(foot);
  }
  void setStart( double x, double y, double yaw_rad, char foot){
    this->start.g.setX(x);
    this->start.g.setY(y);
    this->start.g.setYawRadian(yaw_rad);
    this->start.g.setFoot(foot);
  }

  void feasibilityChecker(){
    ros::FootMarker m(0,0,0);
    m.reset();
    this->start.feasibilityVisualizer();
  }
  virtual void publish(){
    NYI();
  }

  void start_planner(){
    astarsearch->SetStartAndGoalStates( start, goal );

    ContactTransition::feasibilityChecks = 0;
    unsigned int SearchSteps = 0;
    uint SearchState;
    ContactTransition::timer->begin("a*");
    do
      {
        SearchState = astarsearch->SearchStep();
        SearchSteps++;
        if(SearchSteps > 1000){
          astarsearch->CancelSearch();
          break;
        }
      }
    while( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SEARCHING );

    results.success=false;
    results.iterations = SearchSteps;
    results.steps = astarsearch->GetStepCount();
    if(SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED){
      cout << "A* search successful after " << SearchSteps << " iterations." <<endl;
      results.success=true;
    }else{ 
      if(SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_FAILED){
        cout << "Search terminated. Did not find goal" << endl;
      }else{
        cout << "Search terminated. status code of search (see extern/astar/stlastar.h) " << SearchState << endl;
      }
    }
    ContactTransition::timer->end("a*");
    ContactTransition::timer->print_summary();

    results.feasibilityChecks = ContactTransition::feasibilityChecks;
    results.time = ContactTransition::timer->getFinalTime("a*");
    ContactTransition::timer->reset();
  }

  bool success(){
    uint SearchState = astarsearch->SearchStep();
    if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
      return true;
    else
      return false;
  }

  FootStepTrajectory get_footstep_trajectory(){

    FootStepTrajectory fs_trajectory;
    uint SearchState = astarsearch->SearchStep();
    if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
    {
      ContactTransition *node = astarsearch->GetSolutionStart();
      for( ;; ){
          node = astarsearch->GetSolutionNext();
          if( !node ) break;
            
          std::vector<double> fs_tmp = 
            vecD(node->rel_x, node->rel_y, node->rel_yaw, node->L_or_R=='L'?'R':'L', node->g.getX(), node->g.getY(), node->g.getYawRadian());

          //FootStepState fs_state(fs_tmp);
          fs_trajectory.push_back(fs_tmp);
      };
      astarsearch->FreeSolutionNodes();
    }
    fs_trajectory.add_prescripted_end_sequence(this->goal.g);

    DEBUG(
    		Logger footlogger("footsteps.dat");
    		footlogger( fs_trajectory.getFootSteps() );
		)
    astarsearch->EnsureMemoryFreed();
    return fs_trajectory;
  }



  void clean(){
    ContactTransition::cleanStatic();
    astarsearch->EnsureMemoryFreed();
  }

  virtual void addObjectToPlanner(ros::RVIZVisualMarker *m){
    ContactTransition::objects.push_back(m);
  }
  virtual void cleanObjects(){
    ContactTransition::objects.clear();
  }
};
ContactTransition MotionPlannerAStar::goal;
ContactTransition MotionPlannerAStar::start;
AStarSearch<ContactTransition> *MotionPlannerAStar::astarsearch;
//std::vector<std::vector<double> > MotionPlannerAStar::fsi;
//uint MotionPlannerAStar::current_step_index_;
//TrajectoryVisualizer *MotionPlannerAStar::tv;
//double MotionPlannerAStar::sf_x, MotionPlannerAStar::sf_y, MotionPlannerAStar::sf_t;
//double MotionPlannerAStar::init_sf_x, MotionPlannerAStar::init_sf_y, MotionPlannerAStar::init_sf_t;
//char MotionPlannerAStar::sf_f;
