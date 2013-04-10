#include <map>
#include <vector>
#include "../extern/astar/stlastar.h"
#include "rviz/rviz_visualmarker.h"

struct ContactTransition;
struct Environment;

struct ContactTransition
{
	ros::Geometry g;
	double rel_x_parent;
	double rel_y_parent;
	double rel_t_parent;
	char L_or_R;
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
	double computeHyperPlaneDistance(std::vector<double> &g, std::vector< std::vector<double> > &obj);

};//contactTransition
std::map< int, std::vector<double> > ContactTransition::hyperplane;
std::vector<ros::RVIZVisualMarker*> ContactTransition::objects;
