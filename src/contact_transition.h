#include <map>
#include <vector>
#include "../extern/astar/stlastar.h"
#include "rviz/rviz_visualmarker.h"
#include "util_timer.h"

struct ContactTransition;
struct Environment;

struct ContactTransition
{
	ros::Geometry g;
	double rel_x_parent;
	double rel_y_parent;
	double rel_yaw_parent;
	char L_or_R;
	static Timer* timer;
	static std::map< int, std::vector<double> > hyperplane;
	static std::vector<ros::RVIZVisualMarker*> objects;
	static bool loaded;
	double cost_so_far;

	ContactTransition();
	void print();

	/// package of functions, neccessary for A* star algorithm --> see
	//stlastar.h
	ContactTransition( ros::Geometry &g);
	double GoalDistanceEstimate( ContactTransition &nodeGoal );
	bool IsGoal( ContactTransition &nodeGoal );
	bool GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node );
	double GetCost( ContactTransition &successor );
	bool IsSameState( ContactTransition &rhs );
	void showSuccessors( double x, double y, double t, char L_or_R);

	/// everything related to hyperplane implementation
	void loadObjectPosition(Environment &env);
	std::vector< std::vector<double> > prepareObjectPosition(double sf_x, double sf_y, double sf_yaw, char foot);
	static void loadHyperPlaneParameters(const char *file);

	double computeHyperPlaneDistance( const std::vector<double> &p, std::vector< std::vector<double> > &obj);

	ros::Geometry computeAbsFFfromRelFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_rel_x, double ff_rel_y, double ff_rel_t, 
		char sf_foot);

};//contactTransition
std::map< int, std::vector<double> > ContactTransition::hyperplane;
std::vector<ros::RVIZVisualMarker*> ContactTransition::objects;
Timer* ContactTransition::timer;
