#include "planner/trajectory_visualizer.h"
#include "planner/motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "planner/contact_transition.h"
#include "util/util_timer.h"
#include "planner/motion_generator.h"
#include "std_msgs/Float64MultiArray.h" //pbulish vector<double>

#define DEBUG(x) x
const char *colorLeft = "red";
const char *colorRight = "green";
struct MotionPlannerAStar: public MotionPlanner{

  static ContactTransition goal;
  static ContactTransition start;
  static AStarSearch<ContactTransition> *astarsearch;
  static std::vector<std::vector<double> > fsi;
	static boost::mutex footstep_mutex;

  static double sf_x, sf_y, sf_t;
  static double init_sf_x, init_sf_y, init_sf_t;
  static char sf_f;
  static uint current_step_index_;
  static TrajectoryVisualizer *tv;

  //to publish footsteps on ros topic
  ros::NodeHandle n;
  ros::Publisher pub;
        
 MotionPlannerAStar(Environment *env, int &argc, char** &argv): MotionPlanner(env){
    
    std::string topic = "/feasibility/footsteps/relative";
    pub = n.advertise< std_msgs::Float64MultiArray >(topic.c_str(), 1000);
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
    current_step_index_=0;
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
  void setGoal( ros::Geometry &goal ){
    this->goal.g = goal;
  }
  void setStart( ros::Geometry &start ){
    static bool firsttime = true;
    if(firsttime){
      firsttime=false;
      this->start.g = start;
      this->start.rel_x_parent = 0;
      this->start.rel_y_parent = 0;
      this->start.rel_yaw_parent = 0;//start.getYawRadian();
      this->start.L_or_R = 'L';
      init_sf_x = this->start.g.getX();
      init_sf_y = this->start.g.getY();
      init_sf_t = this->start.g.getYawRadian();
      //ROS_INFO("%f %f %f %f %f %f", this->start.g.x, this->start.g.y, this->start.g.getYawRadian(), start.x, start.y, start.getYawRadian());
      tv = new TrajectoryVisualizer(start.getX(), start.getY(), start.getYawRadian()); //visualize q with CoM offset
    }
  }

  void feasibilityChecker(){
    ros::FootMarker m(0,0,0);
    m.reset();
    this->start.feasibilityVisualizer();
  }

  void start_planner(){
    footstep_mutex.lock();
    astarsearch->SetStartAndGoalStates( start, goal );
    footstep_mutex.unlock();

    ContactTransition::feasibilityChecks = 0;
    unsigned int SearchSteps = 0;
    uint SearchState;
    ContactTransition::timer->begin("a*");
    do
      {
        SearchState = astarsearch->SearchStep();
        SearchSteps++;
        if(SearchSteps > 10000){
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
		/*
		boost::mutex::scoped_lock lock(footstep_mutex);
    if( ContactTransition::isInCollision(fsi, current_step_index_ ) ){
      return false;
    }else{
      return true;
    }
    */
    uint SearchState = astarsearch->SearchStep();
    if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
      return true;
    else
      return false;
  }

  void clean_publish(){
    ros::FootMarker m(0,0,0);
    m.reset();
  }

  std::vector<std::vector<double> > get_footstep_vector(){

    std::vector<std::vector<double> > fs_vector;
    uint SearchState = astarsearch->SearchStep();
    if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
      {
        ContactTransition *node = astarsearch->GetSolutionStart();
        //double offset_t = node->g.getYawRadian();
				//node = astarsearch->GetSolutionNext();
				//std::vector<double> tmp_fsi = 
					//vecD(node->rel_x, node->rel_y, node->rel_yaw, node->L_or_R=='L'?'R':'L', node->g.x, node->g.y, node->g.getYawRadian());
				//fs_vector.push_back(tmp_fsi);
        for( ;; )
          {
            node = astarsearch->GetSolutionNext();
            if( !node ) break;

            std::vector<double> tmp_fsi = 
              vecD(node->rel_x, node->rel_y, node->rel_yaw, node->L_or_R=='L'?'R':'L', node->g.getX(), node->g.getY(), node->g.getYawRadian());
            fs_vector.push_back(tmp_fsi);
          };
        astarsearch->FreeSolutionNodes();

      }
    if(!fs_vector.empty()){
      fs_vector.pop_back(); //delete last element (is predefined goal positon and does not belong to the trajectory)

      char sf_f = fs_vector.at( fs_vector.size() -1 ).at(3);

      double goal_x = this->goal.g.getX();
      double goal_y = this->goal.g.getY();
      double yawg = this->goal.g.getYawRadian();

      double xl = fs_vector.at( fs_vector.size() -1 ).at(4);
      double yl = fs_vector.at(fs_vector.size() -1 ).at(5);
      double yawl = fs_vector.at(fs_vector.size() -1 ).at(6);

      double goal_offset_x = 0.0;
      double goal_offset_y = -0.1; //offset in L direction
      double step_y = 0.2; //distance between feet in half-sitting

      if(sf_f == 'L'){
        double goal_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(-goal_offset_y) + goal_x;
        double goal_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(-goal_offset_y) + goal_y;
        double goal_offset_in_right_foot_space_x = cos(-yawl)*(goal_offset_abs_x-xl) - sin(-yawl)*(goal_offset_abs_y-yl);
        double goal_offset_in_right_foot_space_y = sin(-yawl)*(goal_offset_abs_x-xl) + cos(-yawl)*(goal_offset_abs_y-yl);
        double xr = goal_offset_in_right_foot_space_x;
        double yr = goal_offset_in_right_foot_space_y;
        double tr = - yawg + yawl;

        //security such that feet are not colliding
        while(sqrt(xr*xr+yr*yr)<0.3){
          xr+=0.01;yr+0.02;
        }

        std::vector<double> pre_script_foot = vecD(xr, -yr , tr, 'R', goal_offset_abs_x, goal_offset_abs_y, yawg);
        fs_vector.push_back(pre_script_foot);

        double goal_left_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y+step_y) + goal_x;
        double goal_left_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y+step_y) + goal_y;

        std::vector<double> pre_script_foot2 = vecD(0, -step_y, 0, 'L', goal_left_offset_abs_x,goal_left_offset_abs_y, yawg);
        fs_vector.push_back(pre_script_foot2);
      }else{
        double goal_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y) + goal_x;
        double goal_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y) + goal_y;
        double goal_offset_in_left_foot_space_x = cos(-yawl)*(goal_offset_abs_x-xl) - sin(-yawl)*(goal_offset_abs_y-yl);
        double goal_offset_in_left_foot_space_y = sin(-yawl)*(goal_offset_abs_x-xl) + cos(-yawl)*(goal_offset_abs_y-yl);
        double xr = goal_offset_in_left_foot_space_x;
        double yr = goal_offset_in_left_foot_space_y;
        double tr = yawg - yawl;

        //security such that feet are not colliding
        while(sqrt(xr*xr+yr*yr)<0.3){
          xr+=0.01;yr+0.02;
        }
        std::vector<double> pre_script_foot = vecD(xr, yr , tr, 'L', goal_offset_abs_x, goal_offset_abs_y, yawg);
        fs_vector.push_back(pre_script_foot);
        double goal_left_offset_abs_x = cos(yawg)*(goal_offset_x) - sin(yawg)*(goal_offset_y+step_y) + goal_x;
        double goal_left_offset_abs_y = sin(yawg)*(goal_offset_x) + cos(yawg)*(goal_offset_y+step_y) + goal_y;

        std::vector<double> pre_script_foot2 = vecD(0, -step_y, 0, 'R', goal_left_offset_abs_x,goal_left_offset_abs_y, yawg);
        fs_vector.push_back(pre_script_foot2);
      }


    }

    DEBUG(
    		Logger footlogger("footsteps.dat");
    		footlogger( fs_vector );
		)
    astarsearch->EnsureMemoryFreed();
    return fs_vector;
  }

  void update_planner(){
    uint step_horizon = 4;

		boost::mutex::scoped_lock lock(footstep_mutex);

    if(fsi.size()==0){
      fsi = get_footstep_vector();
      return;
    }
    if(!environment_changed && success()){
			//ROS_INFO("Unchanged environment -> reusing old trajectory");
			return;
		}


    std::vector<std::vector<double> > fsi_new;
    fsi_new = get_footstep_vector();

    if(current_step_index_ + step_horizon >= fsi.size() + fsi_new.size()){
      //goal is reached in the next three steps, 
      return;
    }

    //check which step in fsi is the fsi start position
    bool found = false;
    int fsi_last_element = -1;
    int fsi_new_first_element = -1;
    for(uint i = current_step_index_ ; i< fsi.size() ; i++){
      double x = fsi.at(i).at(4);
      double y = fsi.at(i).at(5);
      double t = fsi.at(i).at(6);
      char f = fsi.at(i).at(3);
      for(uint j = 0; j< fsi_new.size() ; j++){
        double xn = fsi_new.at(j).at(4);
        double yn = fsi_new.at(j).at(5);
        double tn = fsi_new.at(j).at(6);
        char fn = fsi_new.at(j).at(3);
        double d= norml2(x,xn,y,yn);
        if(d<0.0001 && f==fn){
          fsi_last_element = i;
          fsi_new_first_element = j;
          found = true;
          break;
        }
      }
      if(found) break;
    }
    for(uint i=0;i<=fsi_last_element;i++){
      fsi.at(i)=fsi.at(i);
    }
    if(!found){
      ROS_INFO("*************************************");
      ROS_INFO("[FATAL_ERROR] Could not connect replanned path to old path");
      ROS_INFO("*************************************");
      cout << fsi.size() << endl;
      cout << fsi_new.size() << endl;
      cout << fsi << endl;
      cout << fsi_new << endl;
      ros::FootMarker m(0,0,0);
      m.reset();
      exit(-1);
    }

    fsi.erase( fsi.begin()+fsi_last_element, fsi.end());
    fsi.insert( fsi.end(), fsi_new.begin()+fsi_new_first_element, fsi_new.end() );
    //fsi.erase(min(fsi.begin() + current_step_index_ + step_horizon, fsi.end()), fsi.end());
    //fsi.insert( fsi.end(), fsi_new.begin(), fsi_new.end() );

  }
  void steps_to_results(){
    results.step_vector = &fsi;
  }
  void publish_footstep_vector(){
    if(current_step_index_ >= fsi.size()){
      ROS_INFO("Finished trajectory");
      return;
    }
    ros::FootMarker l(0,0,0);
    l.reset();
    for(uint i=current_step_index_;i<fsi.size();i++){
      double x = fsi.at(i).at(4);
      double y = fsi.at(i).at(5);
      double t = fsi.at(i).at(6);
      double f = fsi.at(i).at(3);

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

    if(i<fsi.size()){
      double x = fsi.at(i).at(4);
      double y = fsi.at(i).at(5);
      double t = fsi.at(i).at(6);
      char f = fsi.at(i).at(3);
      this->start.g.setX(x);
      this->start.g.setY(y);
      this->start.g.setRPYRadian(0,0,t);
      this->start.L_or_R = (f=='R'?'L':'R'); //starting pos is omitted
      ros::ColorFootMarker m(x,y,t,"blue");
      m.g.setSX(0.33); //0.24
      m.g.setSY(0.19); //0.14
      m.g.setSZ(0.1); //0.03
      m.init_marker();
      m.publish();
    }
    //ROS_INFO("step %d/%d", current_step_index_, fsi.size());

    //publish footsteps over ros topic
    std_msgs::Float64MultiArray ros_steps;
    ros_steps.layout.dim.push_back(std_msgs::MultiArrayDimension());
    ros_steps.layout.dim.push_back(std_msgs::MultiArrayDimension());
    ros_steps.layout.dim[0].label = "step";
    ros_steps.layout.dim[0].size = fsi.size();
    ros_steps.layout.dim[0].stride = fsi.size()*fsi.at(0).size();
    ros_steps.layout.dim[1].label = "relative/absolute";
    ros_steps.layout.dim[1].size = fsi.at(0).size();
    ros_steps.layout.dim[1].stride = fsi.at(0).size();

    ros_steps.data.resize( fsi.at(0).size() * fsi.size() );

    for(uint i=0;i<fsi.size();i++){
      for(uint j=0;j<fsi.at(0).size();j++){
        ros_steps.data[i*fsi.at(0).size()+j]=( fsi.at(i).at(j) );
      }
    }

    pub.publish(ros_steps);

  }

  bool publish_onestep_next(){

		MotionGenerator *mg;
    std::vector<double> q;
		{
      boost::mutex::scoped_lock lock(footstep_mutex);
			if(current_step_index_ >= fsi.size()){
        results.success=false;
				return false;
			}
			publish_footstep_vector();
			if( ContactTransition::isInCollision(fsi, current_step_index_ ) ){
        //ROS_INFO("********************************");
        //ROS_INFO("COLLISION WARNING");
        //ROS_INFO("********************************");
        lock.unlock();
        //ros::Rate rq(1);
        //rq.sleep();
			  return false;
      }
			mg = new MotionGenerator(ContactTransition::objects); //generate q
			q =  mg->generateWholeBodyMotionFromAbsoluteFootsteps(fsi, current_step_index_, 0, 0.19, 0, 'R'); //where is the right foot wrt the left foot (relative)
			lock.unlock();
		}

    if(q.size()>0){
      ROS_INFO("configuration vector: %d", q.size());
      //Replay trajectory
      tv->init(q);
      ros::Rate rq(400); //300
      while(tv->next()){
        ros::spinOnce();
        rq.sleep();
      }
    }

    current_step_index_++;
    ROS_INFO("step_index: %d -> %d", current_step_index_-1, current_step_index_);
    return true;
  }


  void draw_swept_volume(){
    if(fsi.size()>0){
      draw_swept_volume(fsi.at(0).at(0), fsi.at(0).at(1), fsi.at(0).at(2), fsi.at(0).at(3));
    }
  }
  void draw_swept_volume(double x, double y, double yaw, bool L_or_R){

    uint hash = hashit<double>(vecD(x,y,yaw));

    std::string robot_file = "/model/fullBodyApprox/";
    robot_file += ContactTransition::get_swept_volume_file_name( hash );
    robot_file = get_robot_str(robot_file.c_str());

    ROS_INFO("step[%d] %f %f %f", 0, x, y, yaw);
    ros::Geometry sv_pos;
    sv_pos.setX(x);
    sv_pos.setY(y);
    sv_pos.setRPYRadian( 0,0, yaw );

    ROS_INFO("loading swept volume %s",robot_file.c_str());

    ros::TrisTriangleObject *robot;
    if(L_or_R == 'L'){
      robot = new ros::TrisTriangleObject(robot_file.c_str(), sv_pos, false);
    }else{
      robot = new ros::TrisTriangleObject(robot_file.c_str(), sv_pos, true);
    }
    robot->set_color(0.6, 0.0, 0.6, 0.3);
    robot->publish();
    delete robot;

  }
  void publish(){
    NYI();
  }
  void clean(){
    ContactTransition::cleanStatic();
    astarsearch->EnsureMemoryFreed();
  }

  virtual void addObjectToPlanner(ros::RVIZVisualMarker *m){
    ContactTransition::objects.push_back(m);
    //ContactTransition::objects = environment->getObjects();
  }
  virtual void cleanObjects(){
    ContactTransition::objects.clear();
  }
};
ContactTransition MotionPlannerAStar::goal;
ContactTransition MotionPlannerAStar::start;
AStarSearch<ContactTransition> *MotionPlannerAStar::astarsearch;
std::vector<std::vector<double> > MotionPlannerAStar::fsi;
boost::mutex MotionPlannerAStar::footstep_mutex;
uint MotionPlannerAStar::current_step_index_;
TrajectoryVisualizer *MotionPlannerAStar::tv;
double MotionPlannerAStar::sf_x, MotionPlannerAStar::sf_y, MotionPlannerAStar::sf_t;
double MotionPlannerAStar::init_sf_x, MotionPlannerAStar::init_sf_y, MotionPlannerAStar::init_sf_t;
char MotionPlannerAStar::sf_f;
