#include <map>
#include <vector>
#include "../extern/astar/stlastar.h"
#include "rviz/rviz_visualmarker.h"

struct ContactTransition;
struct Environment;

struct ContactTransition
{
	ros::Geometry g;
	char L_or_R;
	static std::map< int, std::vector<double> > hyperplane;
	static bool loaded;
	double cost_so_far;

	ContactTransition();
	void print();
	ContactTransition( ros::Geometry &g);
	double GoalDistanceEstimate( ContactTransition &nodeGoal );
	bool IsGoal( ContactTransition &nodeGoal );
	bool GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node );
	double GetCost( ContactTransition &successor );
	bool IsSameState( ContactTransition &rhs );
	void showSuccessors( double x, double y, double t, char L_or_R);

	void loadObjectPosition(Environment &env);
	static void loadHyperPlaneParameters(const char *file);
	double getDistanceToHyperPlane();
	void nearestDiscreteGeometry(std::vector<int> &in, double x, double y, double t);

};//contactTransition
std::map< int, std::vector<double> > ContactTransition::hyperplane;
