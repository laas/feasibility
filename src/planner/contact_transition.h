//#include <map>
#include <unordered_map>
#include <vector>
#include "../extern/astar/stlastar.h"
#include "rviz/rviz_visualmarker.h"
#include "util_timer.h"
#include "planner/constraints_checker.h"

//unordered_map query time O(1) -- map query time O(log(n))

struct ContactTransition
{
	ros::Geometry g;
	double rel_x_parent;
	double rel_y_parent;
	double rel_yaw_parent;
	char L_or_R;
	static Timer* timer;

	static uint feasibilityChecks;

	static ConstraintsChecker *constraints;
	static std::vector<ros::RVIZVisualMarker*> objects;
	static bool loaded;
	double cost_so_far;

	ContactTransition();
	void print();
	static void cleanStatic();

	/// package of functions, neccessary for A* star algorithm --> see
	//stlastar.h
	ContactTransition( ros::Geometry &g);
	double GoalDistanceEstimate( ContactTransition &nodeGoal );
	bool IsGoal( ContactTransition &nodeGoal );
	bool GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node );
	double GetCost( ContactTransition &successor );
	bool IsSameState( ContactTransition &rhs );
	void showSuccessors( double x, double y, double t, char L_or_R);
	static void setConstraintsChecker( ConstraintsChecker *c );

	ros::Geometry computeAbsFFfromRelFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_rel_x, double ff_rel_y, double ff_rel_t, 
		char sf_foot);

};//contactTransition
ConstraintsChecker *ContactTransition::constraints;
std::vector<ros::RVIZVisualMarker*> ContactTransition::objects;
Timer* ContactTransition::timer;
uint ContactTransition::feasibilityChecks;
