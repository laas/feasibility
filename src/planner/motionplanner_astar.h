#include "planner/motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "planner/contact_transition.h"
#include "util_timer.h"

#define DEBUG(x) x
struct MotionPlannerAStar: public MotionPlanner{

	ContactTransition goal;
	ContactTransition start;
	AStarSearch<ContactTransition> *astarsearch;

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
		this->goal = goal;
	}
	void setStart( ros::Geometry &start ){
		this->start = start;
		this->start.rel_x_parent = 0;
		this->start.rel_y_parent = 0;
		this->start.rel_yaw_parent = 0;
		this->start.L_or_R = 'L'; //start foot
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

	//plot footsteps into rviz
	void publish(){
		publish("green", "red");
	}
	void clean_publish(){
		ros::FootMarker m(0,0,0);
		m.reset();
	}
	void publish(const char *colorLeft, const char *colorRight){

		uint SearchState = astarsearch->SearchStep();
		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
		{
			ContactTransition *node = astarsearch->GetSolutionStart();
			int steps = 0;
			double oldX = node->g.x;
			double oldY = node->g.y-0.2;

			ros::ColorFootMarker l(oldX, oldY, node->g.getYawRadian(), colorRight);
			l.publish();

			ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorLeft);
			m.publish();

			for( ;; )
			{
				DEBUG(ROS_INFO("step[%d] %f %f %f", steps, node->g.x, node->g.y, node->g.getYawRadian());)
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				if(node->L_or_R == 'L'){
					ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorLeft);
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;
				}else{
					ros::ColorFootMarker m(node->g.x, node->g.y, node->g.getYawRadian(), colorRight);
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;
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
