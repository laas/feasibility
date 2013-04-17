#include "motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "contact_transition.h"
#include "util_timer.h"

struct MotionPlannerHyperPlanar: public MotionPlanner{
	static Environment *environment;

	ContactTransition goal;
	ContactTransition start;
	AStarSearch<ContactTransition> *astarsearch;

	MotionPlannerHyperPlanar(Environment &env, int &argc, char** &argv): MotionPlanner(env){
		this->environment = &env;
		astarsearch = new AStarSearch<ContactTransition>(10000);
		ContactTransition::timer = new Timer();
		ContactTransition::timer->register_stopper("prepare", "prepare objects");
		ContactTransition::timer->register_stopper("hyperplanar", "compute hyperplanar");
		ContactTransition::timer->register_stopper("ff", "compute ff transformation");
		ContactTransition::timer->register_stopper("loader", "load hyperplane params");
		ContactTransition::timer->register_stopper("a*", "a* algorithm");

		ContactTransition::timer->begin("loader");
		ContactTransition::loadHyperPlaneParameters("data/planeparams.dat");
		ContactTransition::timer->end("loader");
	}
	void setGoal( ros::Geometry &goal ){
		//this->goal = new ContactTransition( goal );
		this->goal = goal;
	}
	void setStart( ros::Geometry &start ){
		//this->start = new ContactTransition( start );
		this->start = start;
		this->start.rel_x_parent = 0;
		this->start.rel_y_parent = 0;
		this->start.rel_yaw_parent = 0;
		this->start.L_or_R = 'L'; //start foot
	}

	void start_planner(){
		//goal.print();
		//goal.showSuccessors(1,-2,0,'L');
		//goal.showSuccessors(1,-1,M_PI/2,'L');
		//goal.showSuccessors(1,0,M_PI,'L');
		//goal.showSuccessors(1,1,3*M_PI/2,'L');
		//goal.showSuccessors(1,2,2*M_PI,'L');
		//goal.showSuccessors(1,-2,0,'R');
		//goal.showSuccessors(1,-1,M_PI/2,'R');
		//goal.showSuccessors(1,0,M_PI,'R');
		//goal.showSuccessors(0.5,0,M_PI/6,'R');
		//goal.showSuccessors(1,2,2*M_PI,'R');
	//goal.showSuccessors(0.982171, 0.502283, 1.012291, 'L');
		//return;

		//std::vector<double> g(3);
		//g.at(0)=0.0;
		//g.at(1)=-0.16;
		//g.at(2)=0.0;



		//std::vector<double> obj(3);
		//obj.at(0)=0;
		//obj.at(1)=0;
		//obj.at(2)=M_PI;
		//
		//for(double v=0.0;v<5;v+=0.1){
		//	obj.at(0)=v;
		//	std::vector<std::vector<double> > kob;
		//	kob.push_back(obj);
		//	double d = goal.computeHyperPlaneDistance(g, kob);
		//	ROS_INFO("hyperplane dist for obj x = %f >> %f", v, d);
		//}
		//return;
		//ros::LeftFootMarker m(0,1,0);
		//m.addText("PLANNER");
		//m.publish();
		//goal.getDistanceToHyperPlane();
		//start.loadObjectPositions(environment);

		astarsearch->SetStartAndGoalStates( start, goal );
		unsigned int SearchSteps = 0;
		uint SearchState;
		ContactTransition::timer->begin("a*");
		do
		{
			SearchState = astarsearch->SearchStep();
			SearchSteps++;
			if(SearchSteps > 1000){
				break;
			}
		}
		while( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SEARCHING );

		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "A* search successful after " << SearchSteps << " iterations." <<endl;
		}
		else if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_FAILED ) 
		{
			cout << "Search terminated. Did not find goal" << endl;
		}
		ContactTransition::timer->end("a*");
		ContactTransition::timer->print_summary();
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

		uint SearchState = astarsearch->SearchStep();
		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
		{
			ContactTransition *node = astarsearch->GetSolutionStart();
			int steps = 0;

			ros::LeftFootMarker m(node->g.x, node->g.y, node->g.getYawRadian());
			m.reset();
			m.publish();
			double oldX = node->g.x;
			double oldY = node->g.y;
			for( ;; )
			{
				ROS_INFO("step[%d] %f %f %f", steps, node->g.x, node->g.y, node->g.getYawRadian());
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				if(node->L_or_R == 'L'){
					ros::LeftFootMarker m(node->g.x, node->g.y, node->g.getYawRadian());
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;
				}else{
					ros::RightFootMarker m(node->g.x, node->g.y, node->g.getYawRadian());
					m.publish();
					m.drawLine(oldX, oldY);
					oldX = node->g.x;oldY = node->g.y;
				}
				steps++;
			};
			cout << "Solution steps " << steps << endl;
			astarsearch->FreeSolutionNodes();
		}
		else if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_FAILED ) 
		{
			cout << "No contact transitions are published" << endl;
		}

		astarsearch->EnsureMemoryFreed();
	}
	virtual void addObjectToPlanner(ros::RVIZVisualMarker *m){
		ContactTransition::objects = environment->getObjects();
	}
};
Environment *MotionPlannerHyperPlanar::environment;
