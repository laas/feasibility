//#include <map>
#include <unordered_map>
#include <vector>
#include "../extern/astar/stlastar.h"
#include "rviz/rviz_visualmarker.h"
#include "util_timer.h"

//unordered_map query time O(1) -- map query time O(log(n))
typedef std::unordered_map< int, std::vector<double> > HashMap; 

#include <fann.h>
typedef std::unordered_map< int, struct fann*> NeuralHashMap; 

struct ContactTransition
{
	ros::Geometry g;
	double rel_x_parent;
	double rel_y_parent;
	double rel_yaw_parent;
	char L_or_R;
	static Timer* timer;

	static HashMap hyperplane;
	static NeuralHashMap neuralMap;
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
	std::vector< std::vector<double> > prepareObjectPosition(double sf_x, double sf_y, double sf_yaw, char foot);
	static void loadHyperPlaneParameters(const char *file);
	bool isInCollision(  const std::vector<double> &p, const std::vector< std::vector<double> > &obj);

	static void loadNNParameters(const char *path);

	double computeHyperPlaneDistance( const std::vector<double> &p, const std::vector< std::vector<double> > &obj);

	ros::Geometry computeAbsFFfromRelFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_rel_x, double ff_rel_y, double ff_rel_t, 
		char sf_foot);

};//contactTransition
HashMap ContactTransition::hyperplane;
NeuralHashMap ContactTransition::neuralMap;
std::vector<ros::RVIZVisualMarker*> ContactTransition::objects;
Timer* ContactTransition::timer;
