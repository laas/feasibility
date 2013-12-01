#pragma once
//#include <map>
#include <unordered_map>
#include <vector>
#include "../extern/astar/stlastar.h"
#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"
#include "util/util_timer.h"
#include "planner/constraints_checker.h"

//unordered_map query time O(1) -- map query time O(log(n))

struct ContactTransition
{
	ros::Geometry g;
	double rel_x_parent;
	double rel_y_parent;
	double rel_yaw_parent;

	double rel_x;
	double rel_y;
	double rel_yaw;

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
	static std::string get_swept_volume_file_name( uint action_hash );

	/// package of functions, neccessary for A* star algorithm --> see
	//stlastar.h
	ContactTransition( ros::Geometry &g);
	double GoalDistanceEstimate( ContactTransition &nodeGoal );
	bool IsGoal( ContactTransition &nodeGoal );
	bool GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node );
	double GetCost( ContactTransition &successor );
	bool IsSameState( ContactTransition &rhs );
	void showSuccessors( double x, double y, double t, char L_or_R);
	void feasibilityVisualizer();
	static bool isInCollision( std::vector< std::vector<double> > &fsi, uint current_step_index );
	static void setConstraintsChecker( ConstraintsChecker *c );

	ros::Geometry computeAbsFFfromRelFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_rel_x, double ff_rel_y, double ff_rel_t, 
		char sf_foot);
	ros::Geometry computeRelFFfromAbsFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_abs_x, double ff_abs_y, double ff_abs_yaw,
		char sf_foot);

};//contactTransition
