#include "motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "contact_transition.h"

struct MotionPlannerHyperPlanar: public MotionPlanner{
	static Environment *environment;

	ContactTransition goal;
	ContactTransition start;
	AStarSearch<ContactTransition> *astarsearch;

	MotionPlannerHyperPlanar(Environment &env, int &argc, char** &argv): MotionPlanner(env){
		this->environment = &env;
		astarsearch = new AStarSearch<ContactTransition>(2000);
		ContactTransition::loadHyperPlaneParameters("data/planeparams.dat");
	}
	void setGoal( ros::Geometry &goal ){
		//this->goal = new ContactTransition( goal );
		this->goal = goal;
	}
	void setStart( ros::Geometry &start ){
		//this->start = new ContactTransition( start );
		this->start = start;
		this->start.L_or_R = 'L';
	}

	void start_planner(){
		start.print();
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
		//return;
		ROS_INFO("start planner maintenant");
		//ros::LeftFootMarker m(0,1,0);
		//m.addText("PLANNER");
		//m.publish();
		//goal.getDistanceToHyperPlane();
		//start.loadObjectPositions(environment);

		astarsearch->SetStartAndGoalStates( start, goal );
		unsigned int SearchSteps = 0;
		uint SearchState;
		do
		{
			SearchState = astarsearch->SearchStep();
			SearchSteps++;
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
	}
	bool success(){
		uint SearchState = astarsearch->SearchStep();
		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
			return true;
		else
			return false;
	}
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
				ROS_INFO("step[%d] %f %f", steps, node->g.x, node->g.y);
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
		//ROS_INFO("adding objects to planner NOT YET IMPLEMENTED!");
		ContactTransition::objects = environment->getObjects();
	}
};
Environment *MotionPlannerHyperPlanar::environment;
