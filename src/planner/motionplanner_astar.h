#include "planner/motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "planner/contact_transition.h"
#include "util/util_timer.h"
#include "planner/motion_generator.h"

#define DEBUG(x) x
const char *colorLeft = "red";
const char *colorRight = "green";
struct MotionPlannerAStar: public MotionPlanner{

	ContactTransition goal;
	ContactTransition start;
	AStarSearch<ContactTransition> *astarsearch;
	std::vector<std::vector<double> > fsi;
	double sf_x, sf_y, sf_t;
	char sf_f;
	uint _current_step_index;
	TrajectoryVisualizer *tv;

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
		_current_step_index=0;
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
			this->start.rel_yaw_parent = 0;
			this->start.L_or_R = 'L';
			tv = new TrajectoryVisualizer(start.x, start.y, start.getYawRadian()); //visualize q with CoM offset
		}
	}

	void feasibilityChecker(){
		ros::FootMarker m(0,0,0);
		m.reset();
		this->start.feasibilityVisualizer();
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
			if(SearchSteps > 200){
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

	//plot footsteps into rviz
	void publish(){
		publish("green", "red");
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
			for( ;; )
			{
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				std::vector<double> tmp_fsi = 
						vecD(node->rel_x, node->rel_y, node->rel_yaw, node->L_or_R=='L'?'R':'L', node->g.x, node->g.y, node->g.getYawRadian());
				fs_vector.push_back(tmp_fsi);
			};
			astarsearch->FreeSolutionNodes();

		}
		if(!fs_vector.empty()){
			fs_vector.pop_back(); //delete last element (is predefined goal positon and does not belong to the trajectory)
		}
		astarsearch->EnsureMemoryFreed();
		return fs_vector;
	}

	void update_planner(){
		if(fsi.size()==0){
			fsi = get_footstep_vector();
			return;
		}

		for(uint i=0;i<=_current_step_index+3 && i<fsi.size();i++){
			fsi.at(i)=fsi.at(i);
		}

		std::vector<std::vector<double> > fsi_new;
		fsi_new = get_footstep_vector();

		if(_current_step_index+3 >= fsi.size()+fsi_new.size()){
			//goal is reached in the next three steps, 
			return;
		}

		fsi.erase(min(fsi.begin()+_current_step_index+3, fsi.end()), fsi.end());
		fsi.insert( fsi.end(), fsi_new.begin(), fsi_new.end() );
	}
	void publish_footstep_vector(){
		if(_current_step_index >= fsi.size()){
			ROS_INFO("Finished trajectory");
			return;
		}
		ros::FootMarker l(0,0,0);
		l.reset();
		for(uint i=_current_step_index;i<fsi.size();i++){
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
		uint i=_current_step_index+3;

		if(i<fsi.size()){
			double x = fsi.at(i).at(4);
			double y = fsi.at(i).at(5);
			double t = fsi.at(i).at(6);
			char f = fsi.at(i).at(3);
			this->start.g.x=x;
			this->start.g.y=y;
			this->start.g.setRPYRadian(0,0,t);
			this->start.L_or_R = f=='R'?'L':'R'; //starting pos is omitted
			ros::ColorFootMarker m(x,y,t,"blue");
			m.publish();
		}
		ROS_INFO("step %d/%d", _current_step_index, fsi.size());
	}
	bool publish_onestep_next(){
			if(_current_step_index >= fsi.size()){
				return false;
			}

			publish_footstep_vector();
			MotionGenerator *mg = new MotionGenerator(); //generate q
			std::vector<double> q = 
				mg->generateWholeBodyMotionFromAbsoluteFootsteps(fsi, _current_step_index);
			if(q.size()>0){
				ROS_INFO("configuration vector: %d", q.size());
				//Replay trajectory
				tv->init(q);
				ros::Rate rq(200);
				while(tv->next()){
					ros::spinOnce();
					rq.sleep();
				}
			}

			_current_step_index++;
			return true;
	}
	void publish_run(){
		const char *colorLeft = "red";
		const char *colorRight = "green";
		uint SearchState = astarsearch->SearchStep();

		std::vector<std::vector<double> > fsi;
		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
		{
			ContactTransition *node = astarsearch->GetSolutionStart();
			int steps = 0;
			double oldX = node->g.x;
			double oldY = node->g.y;
			double oldT = node->g.getYawRadian();

			ros::ColorFootMarker l(oldX, oldY, node->g.getYawRadian(), colorRight);
			l.publish();
			ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorLeft);
			m.publish();

			for( ;; )
			{
				DEBUG(ROS_INFO("step[%d] %f %f %f", steps, node->g.x, node->g.y, toDeg(node->g.getYawRadian()));)
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				std::vector<double> tmp_fsi = 
						vecD(node->rel_x, node->rel_y, node->rel_yaw, node->L_or_R=='L'?'R':'L');
				ROS_INFO("X: %f, Y: %f, T: %f, F: %f", node->rel_x, node->rel_y, toDeg(node->rel_yaw), node->L_or_R);
				fsi.push_back(tmp_fsi);

				if(node->L_or_R == 'L'){
					ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorLeft);
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;//oldT = node->g.getYawRadian();
				}else{
					ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorRight);
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;//oldT = node->g.getYawRadian();
				}

				steps++;
			};
			cout << "Solution steps " << steps << endl;
			astarsearch->FreeSolutionNodes();
			results.steps = steps;

			//Show Trajectory
			TrajectoryVisualizer *tv = new TrajectoryVisualizer(start.g.x, start.g.y);
			MotionGenerator *mg = new MotionGenerator();
			for(uint i=0;i<fsi.size() && ros::ok();i++){
				std::vector<double> q = mg->generateWholeBodyMotionFromAbsoluteFootsteps(fsi, i);
				if(q.size()>0){
					ROS_INFO("configuration vector: %d", q.size());
					//Replay trajectory
					tv->init(q);
					ros::Rate rq(500);
					while(tv->next()){
						ros::spinOnce();
						rq.sleep();
					}
				}
			}



		}
		else if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_FAILED ) 
		{
			cout << "No contact transitions are published" << endl;
		}

		astarsearch->EnsureMemoryFreed();


	}
	void publish(const char *colorLeft, const char *colorRight, bool sv=false){
		uint SearchState = astarsearch->SearchStep();
		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
		{
			ContactTransition *node = astarsearch->GetSolutionStart();
			int steps = 0;
			double oldX = node->g.x;
			double oldY = node->g.y;
			double oldT = node->g.getYawRadian();

			ros::ColorFootMarker l(oldX, oldY, node->g.getYawRadian(), colorRight);
			l.publish();

			ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorLeft);
			m.publish();

			for( ;; )
			{
				DEBUG(ROS_INFO("step[%d] %f %f %f", steps, node->g.x, node->g.y, toDeg(node->g.getYawRadian()));)
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				if(node->L_or_R == 'L'){
					ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorLeft);
					m.publish();
					m.drawLine(oldX, oldY);



					oldX = node->g.x;oldY = node->g.y;//oldT = node->g.getYawRadian();
				}else{
					ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorRight);
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;//oldT = node->g.getYawRadian();
				}

				if(sv){
					//ros::Geometry rel = node->computeRelFFfromAbsFF( oldX, oldY, oldT,
							//node->g.x, node->g.y, node->g.getYawRadian(), node->L_or_R);
					//uint hash = hashit<double>(vecD(rel.x, rel.y, rel.getYawRadian()));
					uint hash = hashit<double>(vecD(node->rel_x, node->rel_y, node->rel_yaw));

					//std::string robot_file = node->get_swept_volume_file_name( hash );
					//robot_file += node->get_swept_volume_file_name( hash );
					std::string robot_file = "fullBodyApprox/";
					robot_file += node->get_swept_volume_file_name( hash );
					robot_file = get_robot_str(robot_file.c_str());

					ROS_INFO("step[%d] %f %f %f", steps, node->rel_x, node->rel_y, node->rel_yaw);
					ros::Geometry sv_pos;
					sv_pos.x = node->g.x;
					sv_pos.y = node->g.y;
					sv_pos.setRPYRadian( 0,0,node->g.getYawRadian() );
					if(node->L_or_R == 'R'){
						sv_pos.setRPYRadian( 0,0,node->g.getYawRadian()+M_PI );
					}

					ROS_INFO("loading swept vlume %s",robot_file.c_str());

					//ros::SweptVolumeObject *robot = static_cast<ConstraintsCheckerSweptVolume*>(ContactTransition::constraints)->get_sv_from_hash(hash);
					ros::TrisTriangleObject *robot = new ros::TrisTriangleObject(robot_file.c_str(), sv_pos);
					robot->set_color(0.6, 0.0, 0.6, 0.3);
					robot->publish();
					delete robot;
				}
				steps++;
			};
			cout << "Solution steps " << steps << endl;
			astarsearch->FreeSolutionNodes();
			results.steps = steps;
		}
		else if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_FAILED ) 
		{
			cout << "No contact transitions are published" << endl;
		}

		astarsearch->EnsureMemoryFreed();
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
