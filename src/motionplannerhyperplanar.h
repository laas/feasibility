#include "motionplanner.h"
#include "../extern/astar/stlastar.h"


struct MotionPlannerHyperPlanar: public MotionPlanner{
	struct ContactTransition
	{
		ros::Geometry g;
		double cost_so_far;
		ContactTransition(){
		}
		ContactTransition( ros::Geometry &g){
			this->g = g;
		}
		float GoalDistanceEstimate( ContactTransition &nodeGoal ){
			double xg = nodeGoal.g.x;
			double yg = nodeGoal.g.y;
			double x = this->g.x;
			double y = this->g.y;
			return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg)); 
		}
		bool IsGoal( ContactTransition &nodeGoal ){
			return this->GoalDistanceEstimate(nodeGoal) < 0.15;
		}
		bool GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node ){
			for (double valX = -0.35; valX < 0.351; valX+=0.05) {
				for (double valY = -0.37; valY < -0.019; valY+=0.05) {
					for (double valT = -0.52; valT < 0.521; valT += 0.26) {
						double abs_x = this->g.x;
						double abs_y = this->g.y;
						double abs_t = this->g.tz;

						double newX = abs_x + cos(abs_t)*valX-sin(abs_t)*valY;
						double newY = abs_y + sin(abs_t)*valX+cos(abs_t)*valY;
						double newT = abs_t + valT;

						ros::Geometry ng;
						ng.x = newX;
						ng.y = newY;
						ng.tz = newT;
						ContactTransition next(ng);
						next.cost_so_far = this->GoalDistanceEstimate( next );
						astarsearch->AddSuccessor(next);
					}
				}
			}
			return true;
		}

		float GetCost( ContactTransition &successor ){
			//return successor.cost_so_far;
			return 0;
			
		}
		bool IsSameState( ContactTransition &rhs ){
			double xg = rhs.g.x;
			double yg = rhs.g.y;
			double tg = rhs.g.tz;
			double x = this->g.x;
			double y = this->g.y;
			double t = this->g.tz;
			return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg) + (t-tg)*(t-tg))<0.01;
		}

	};//contactTransition
	ContactTransition goal;
	ContactTransition start;
	AStarSearch<ContactTransition> *astarsearch;

	MotionPlannerHyperPlanar(Environment &env, int &argc, char** &argv): MotionPlanner(env){
		astarsearch = new AStarSearch<ContactTransition>(100000);
	}
	void setGoal( ros::Geometry &goal ){
		//this->goal = new ContactTransition( goal );
		this->goal = goal;
		goal.print();
	}
	void setStart( ros::Geometry &start ){
		//this->start = new ContactTransition( start );
		this->start = start;
		start.print();
		this->start.cost_so_far = 0.0;
	}

	void start_planner(){
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
			cout << "A* search successful!" << SearchSteps << " steps" <<endl;
		}
		else if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_FAILED ) 
		{
			cout << "Search terminated. Did not find goal state" << endl;
		}
	}
	void publish(){
		uint SearchState = astarsearch->SearchStep();
		if( SearchState == AStarSearch<ContactTransition>::SEARCH_STATE_SUCCEEDED )
		{
			ContactTransition *node = astarsearch->GetSolutionStart();
			int steps = 0;

			ros::LeftFootMarker m(node->g.x, node->g.y, node->g.tz);
			m.publish();
			for( ;; )
			{
				node = astarsearch->GetSolutionNext();
				if( !node ) break;

				ros::LeftFootMarker m(node->g.x, node->g.y, node->g.tz);
				m.publish();
				steps ++;
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
