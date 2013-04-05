#include "motionplanner.h"
#include "../extern/astar/stlastar.h"
#include "contact_transition.h"

struct MotionPlannerHyperPlanar: public MotionPlanner{
	Environment *environment;


	ContactTransition goal;
	ContactTransition start;
	AStarSearch<ContactTransition> *astarsearch;

	MotionPlannerHyperPlanar(Environment &env, int &argc, char** &argv): MotionPlanner(env){
		this->environment = &env;
		astarsearch = new AStarSearch<ContactTransition>(100000);
		ContactTransition::loadHyperPlaneParameters("data/planeparams.dat");
	}
	void setGoal( ros::Geometry &goal ){
		//this->goal = new ContactTransition( goal );
		this->goal = goal;
	}
	void setStart( ros::Geometry &start ){
		//this->start = new ContactTransition( start );
		this->start = start;
	}

	void start_planner(){
		start.print();
		goal.print();
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

			ros::LeftFootMarker m(node->g.x, node->g.y, node->g.tz);
			m.reset();
			m.publish();
			for( ;; )
			{
				ROS_INFO("step[%d] %f %f", steps, node->g.x, node->g.y);
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				ros::LeftFootMarker m(node->g.x, node->g.y, node->g.tz);
				m.publish();
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
		ROS_INFO("adding objects to planner NOT YET IMPLEMENTED!");
	}
};
