#include <algorithm>//lowerbound, sort
#include "contact_transition.h"
#include "util.h"
#include "rviz/rviz_visualmarker.h"
#include "environment.h"

ContactTransition::ContactTransition(){
}
void ContactTransition::print(){
	g.print();
}
ContactTransition::ContactTransition( ros::Geometry &g){
	this->g = g;
}
double ContactTransition::GoalDistanceEstimate( ContactTransition &nodeGoal ){
	double xg = nodeGoal.g.x;
	double yg = nodeGoal.g.y;
	double x = this->g.x;
	double y = this->g.y;
	return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg)); 
}
bool ContactTransition::IsGoal( ContactTransition &nodeGoal ){
	return this->GoalDistanceEstimate(nodeGoal) < 0.2;
}
bool ContactTransition::GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node ){
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

double ContactTransition::GetCost( ContactTransition &successor ){
	//return successor.cost_so_far;
	return 0.0;
}
bool ContactTransition::IsSameState( ContactTransition &rhs ){
	double xg = rhs.g.x;
	double yg = rhs.g.y;
	double tg = rhs.g.tz;
	double x = this->g.x;
	double y = this->g.y;
	double t = this->g.tz;
	return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg) + (t-tg)*(t-tg))<0.01;
}

/*
double ContactTransition::getPlanarDistance( ros::Geometry &g ){
	std::vector<RVIZVisualMarker*> objects = environment->getObjects();

	std::vector<ros::RVIZVisualMarker*>::iterator obj;
	for(obj = objects.begin();obj!=objects.end();obj++){
		double d = (*obj)->getDistanceToHyperPlane(g);
	}


}
*/
void ContactTransition::nearestDiscreteGeometry(std::vector<int> &in, double x, double y, double t){


	std::vector<int> tpos = vecI(-29,-14,0,14,29);
	std::vector<int> xpos = vecI(-34, -29, -24, -19, -14, -9, -4, 0, 5, 10, 15, 20, 25, 30, 35);
	std::vector<int> ypos = vecI(-36, -31, -26, -21, -16, -11, -6, -1);
	cout << xpos << endl;
	cout << ypos << endl;
	cout << tpos << endl;

	std::sort(xpos.begin(), xpos.end());
	std::sort(ypos.begin(), ypos.end());
	std::sort(tpos.begin(), tpos.end());

	cout << x << " " << y << " " <<  t << endl;
	int rx = round2(x);
	int ry = round2(y);
	int rt = round2(t);
	cout << rx << " " << ry << " " << rt << endl;
	int nx = nearest(xpos, rx);
	int ny = nearest(ypos, ry);
	int nt = nearest(tpos, rt);
	cout << nx << " " << ny << " " << nt << endl;

	in.push_back( nearest(xpos, rx) );
	in.push_back( nearest(ypos, ry) );
	in.push_back( nearest(tpos, rt) );
}

double ContactTransition::getDistanceToHyperPlane(){
	std::vector<int> pos;
	nearestDiscreteGeometry(pos, g.x, g.y, g.z);
	uint hash = hashit<int>(pos);

	if(hyperplane.find(hash)==hyperplane.end()){
		ROS_INFO("could not find hyperplane hash for geometry %d %d %d (transformed from %f %f %f) (hash: %d)",pos.at(0), pos.at(1), pos.at(2), g.x,g.y,g.tz,hash);
		throw "hash not found";
		exit(-1);
	}

	std::vector<double> params = hyperplane[hash];
}
void ContactTransition::loadObjectPosition(Environment &env){

}
void ContactTransition::loadHyperPlaneParameters(const char *file){

	//FILE *fp = fopen_s(file,'r');
	ROS_INFO("loading parameter from %s", file);
	CSVReader f(file);
	std::vector< std::vector<double> > vv = f.getVV(7);
	cout << vv.at(0) << endl;
	cout << vv.at(1) << endl;
	cout << vv.at(2) << endl;

	bool collision=false;
	for(uint k=0;k<vv.size();k++){
		std::vector<double> params(4);
		for(uint i=0;i<4;i++) params.push_back(vv.at(k).at(i));

		std::vector<int> pos;
		for(uint i=4;i<7;i++) pos.push_back(vv.at(k).at(i));

		uint hash = hashit<int>(pos);
		ROS_INFO("hash: %d (%d %d %d)", hash, pos.at(0), pos.at(1), pos.at(2));
		if(hyperplane.find(hash)!=hyperplane.end()){
			ROS_INFO("hash collision: %d", hash);
			collision=true;
		}
		hyperplane[hash] = params;
	}
	if(collision){
		ROS_INFO("WARNING: collision in hyperplane");
		throw "collision in hyperplane error";
		exit(-1);
	}
	ROS_INFO("successfully loaded hyperplane hash map without collisions and %d entries!", hyperplane.size());
}
