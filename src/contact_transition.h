#include "../extern/astar/stlastar.h"
struct ContactTransition
{
	ros::Geometry g;
	static std::map< int, std::vector<double> > hyperplane;
	static bool loaded;
	double cost_so_far;

	ContactTransition(){
	}
	void print(){
		g.print();
	}
	ContactTransition( ros::Geometry &g){
		this->g = g;
	}
	double GoalDistanceEstimate( ContactTransition &nodeGoal ){
		double xg = nodeGoal.g.x;
		double yg = nodeGoal.g.y;
		double x = this->g.x;
		double y = this->g.y;
		return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg)); 
	}
	bool IsGoal( ContactTransition &nodeGoal ){
		return this->GoalDistanceEstimate(nodeGoal) < 0.2;
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
					//next.cost_so_far = this->getPlanarDistance(ng) + this->GoalDistanceEstimate( next );
					next.cost_so_far = this->GoalDistanceEstimate( next );
					astarsearch->AddSuccessor(next);
				}
			}
		}
		return true;
	}

	double GetCost( ContactTransition &successor ){
		//return successor.cost_so_far;
		return 0.0;
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

	/*
	double getPlanarDistance( ros::Geometry &g ){
		std::vector<RVIZVisualMarker*> objects = environment->getObjects();

		std::vector<ros::RVIZVisualMarker*>::iterator obj;
		for(obj = objects.begin();obj!=objects.end();obj++){
			double d = (*obj)->getDistanceToHyperPlane(g);
		}


	}
	//hyperparameter realization
	int computeHash(ros::Geometry &g){

	}

	double getDistanceToHyperPlane(ros::Geometry &g){
		std::vector<double> params = hyperplane[ computeHash(g) ];



	}
	*/
	void loadObjectPosition(Environment &env){

	}

	static void loadHyperPlaneParameters(const char *file);

};//contactTransition
std::map< int, std::vector<double> > ContactTransition::hyperplane;
