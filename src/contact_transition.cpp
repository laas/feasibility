#include <algorithm>//lowerbound, sort
#include <map>
#include "contact_transition.h"
#include "util.h"
#include "rviz/rviz_visualmarker.h"
#include "environment.h"


#define DEBUG(x) 
ContactTransition::ContactTransition(){
}
void ContactTransition::print(){
	g.print();
}
ContactTransition::ContactTransition( ros::Geometry &g){
	this->g = g;
	this->g.setYawRadian( g.getYawRadian() );
}
double ContactTransition::GoalDistanceEstimate( ContactTransition &nodeGoal ){
	//heuristic = distance to goal + distance to obstacles
	double xg = nodeGoal.g.x;
	double yg = nodeGoal.g.y;
	double x = this->g.x;
	double y = this->g.y;
	double rx = this->rel_x_parent;
	double ry = this->rel_y_parent;
	//negative reward for stepping back:
	//double stepback = max( 10*(norml2(x,xg,y,yg) - norml2(rx,xg,ry,yg)), 0.0);
	return norml2(x,xg,y,yg);
}
bool ContactTransition::IsGoal( ContactTransition &nodeGoal ){
	return norml2(this->g.x, nodeGoal.g.x, this->g.y, nodeGoal.g.y) < 0.3;
}

void ContactTransition::showSuccessors( double x, double y, double t, char L_or_R){
	std::map<int, std::vector<double> >::const_iterator it;
	char foot = 'L';
	if(L_or_R == 'L'){
		foot='R';
		ros::LeftFootMarker m(x,y,t);
		m.publish();
	}else{
		foot='L';
		ros::RightFootMarker m(x,y,t);
		m.publish();
	}

	double abs_x = x;
	double abs_y = y;
	double abs_t = t;

	uint counter=0;
	//CHECK( (abs_t <= M_PI && abs_t >= -M_PI), "angle not in right interval");

	for(  it = hyperplane.begin(); it != hyperplane.end(); ++it ){

		double valX = it->second.at(4);
		double valY = it->second.at(5);
		double valT = toRad(it->second.at(6)*100);

		double newX = abs_x + cos(abs_t)*valX - sin(abs_t)*valY;
		double newY = abs_y + sin(abs_t)*valX + cos(abs_t)*valY;
		double newT = valT + abs_t;

		if(L_or_R == 'R'){
			//reflection at x-axis [1 0,0 -1], following rotation around z and
			//translation to x,y
			newX = abs_x + cos(abs_t)*valX + sin(abs_t)*valY;
			newY = abs_y + sin(abs_t)*valX - cos(abs_t)*valY;
			newT = -(valT - abs_t);
		}
		while(newT>M_PI) newT-=2*M_PI;
		while(newT<-M_PI) newT+=2*M_PI;

		ros::Geometry ng;
		ng.x = newX;
		ng.y = newY;
		ng.setYawRadian(newT);

		if(rand()>0.0){
			if(foot=='L'){
				ros::LeftFootMarker rp(ng.x, ng.y, ng.getYawRadian());
				rp.publish();
			}else{
				ros::RightFootMarker rp(ng.x, ng.y, ng.getYawRadian());
				rp.publish();
			}
		}
		//}
	}
}
bool ContactTransition::GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node ){
	std::map<int, std::vector<double> >::const_iterator it;
	char foot;

	ContactTransition *parent = this;
	if(parent->L_or_R == 'L'){
		foot='R';
	}else{
		foot='L';
	}

	//get absolute position of support foot (SF)
	double abs_x = parent->g.x;
	double abs_y = parent->g.y;
	double abs_t = parent->g.getYawRadian();

	//get the relative position of the free foot (FF)
	double ff_x = parent->rel_x_parent;
	double ff_y = parent->rel_y_parent;
	double ff_t = parent->rel_t_parent;

	std::vector<double> ff(3);
	ff.at(0)=ff_x;ff.at(1)=ff_y;ff.at(2)=ff_t;

	static bool firstFF=true;

	std::vector< std::vector<double> > obj = this->prepareObjectPosition(abs_x, abs_y, abs_t, L_or_R);

	uint counter=0;
	for(  it = hyperplane.begin(); it != hyperplane.end(); ++it ){

		if(counter++ % 7 ==0) continue;
		//relative position of next pos of FF
		double valX = it->second.at(4);
		double valY = it->second.at(5);
		double valT = toRad(it->second.at(6)*100);

		std::vector<double> ff_next(3);
		ff_next.at(0)=it->second.at(4); ff_next.at(1)=it->second.at(5); ff_next.at(2)=it->second.at(6);

		//absolute position of next pos of FF
		double ff_nextX = abs_x + cos(abs_t)*valX - sin(abs_t)*valY;
		double ff_nextY = abs_y + sin(abs_t)*valX + cos(abs_t)*valY;
		double ff_nextT = valT + abs_t;

		if(L_or_R == 'R'){
			//reflection at x-axis [1 0,0 -1], following rotation around z and
			//translation to x,y
			ff_nextX = abs_x + cos(abs_t)*valX + sin(abs_t)*valY;
			ff_nextY = abs_y + sin(abs_t)*valX - cos(abs_t)*valY;
			ff_nextT = -(valT - abs_t);
		}
		//while(ff_nextT>M_PI) ff_nextT-=2*M_PI;
		//while(ff_nextT<-M_PI) ff_nextT+=2*M_PI;

		//build new contact transition from absolute next FF
		ros::Geometry ng;
		ng.x = ff_nextX;
		ng.y = ff_nextY;
		ng.setYawRadian(ff_nextT);

		ContactTransition next(ng);
		next.L_or_R = foot;
		next.rel_x_parent = it->second.at(4);
		next.rel_y_parent = it->second.at(5);
		next.rel_t_parent = it->second.at(6);

		double hyper = this->computeHyperPlaneDistance(ff_next, obj) - 1.0;
		next.cost_so_far = (hyper>0.0?hyper:0.0);

		if(!(hyper>0.0)){
			astarsearch->AddSuccessor(next);
		}
		//plot if neccessary
		DEBUG(
			if(rand(0,1)>0.95){
				if(foot=='L'){
					ros::LeftFootMarker rp(ng.x, ng.y, ng.getYawRadian());
					rp.publish();
				}else{
					ros::RightFootMarker rp(ng.x, ng.y, ng.getYawRadian());
					rp.publish();
				}
			}
		);
	}
	firstFF=false;
	return true;
}
// support foot (SF)
std::vector< std::vector<double> > ContactTransition::prepareObjectPosition(double sf_x, double sf_y, double sf_yaw, char foot){
	std::vector< std::vector<double> > v;
	std::vector< std::vector<double> >::const_iterator vit;
	std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		double x = (*oit)->g.x;
		double y = (*oit)->g.y;
		double yaw = (*oit)->g.getYawRadian();

		//translate object, so that origin and sf origin conincide
		double tx = x - sf_x;
		double ty = y - sf_y;
		//rotate object around origin, such that object is aligned with
		//sf
		double rx = cos(sf_yaw)*tx - sin(sf_yaw)*ty;
		double ry = sin(sf_yaw)*tx + cos(sf_yaw)*ty;
		double ryaw = yaw - sf_yaw;

		//swept vlumes are always tested on the right side
/*
		if(foot == 'R'){
			rx = cos(sf_yaw)*tx + sin(sf_yaw)*ty;
			ry = sin(sf_yaw)*tx - cos(sf_yaw)*ty;
			ryaw = -(yaw - sf_yaw);
		}
*/
		while(ryaw>M_PI) ryaw-=2*M_PI;
		while(ryaw<-M_PI) ryaw+=2*M_PI;

		//--> cylinder coordinates
		std::vector<double> cyl(3);
		cyl.at(0)=sqrtf(rx*rx + ry*ry); cyl.at(1)=atan2(rx,ry); cyl.at(2)=yaw;

		v.push_back(cyl);
		//DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

	}
	return v;
}

//input: obj in cylinder coordinates (r,phi, yaw)| relative footstep of free
//foot (FF), in X,Y,T, whereby T is degree divided by 100
double ContactTransition::computeHyperPlaneDistance(std::vector<double> &g, std::vector< std::vector<double> > &obj){
	uint hash = hashit<double>(g);
	//ROS_INFO("%d objects", obj.size());

	if(hyperplane.find(hash)==hyperplane.end()){
		ROS_INFO("could not find hyperplane hash for geometry %f %f %f (hash: %d)",g.at(0),g.at(1),g.at(2),hash);
		throw "hash not found";
		exit(-1);
	}
	std::vector<double> params = hyperplane[hash];

	double cost = 0.0;

	double a = params.at(0);
	double b = params.at(1);
	double c = params.at(2);
	double d = params.at(3);

	//ROS_INFO("hyperplane %f %f %f %f", a,b,c,d);

	std::vector< std::vector<double> >::const_iterator oit;
	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		//compute distance from hyperplane (TODO: port to eigen to make
		//it accesible in N dimensions)
		double ox = (*oit).at(0);
		double oy = (*oit).at(1);
		double ot = (*oit).at(2);

		double dist = (a*ox+b*oy+c*ot+d) / sqrtf(a*a+b*b+c*c);
		cost+=dist;
		//ROS_INFO("dist %f cost %f", dist, cost);

	}

	return cost;
}

double ContactTransition::GetCost( ContactTransition &successor ){
	//return 0.0;
	return successor.cost_so_far;
}
bool ContactTransition::IsSameState( ContactTransition &rhs ){
	double xg = rhs.g.x;
	double yg = rhs.g.y;
	double yawg = rhs.g.getYawRadian();
	double x = this->g.x;
	double y = this->g.y;
	double yaw = this->g.getYawRadian();
	return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg) + (yaw-yawg)*(yaw-yawg))<0.01;
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

		std::vector<double> pos;

		for(uint i=4;i<7;i++) pos.push_back(vv.at(k).at(i)/100.0);

		uint hash = hashit<double>(pos);

		ROS_INFO("hash: %d (%f %f %f)", hash, pos.at(0), pos.at(1), pos.at(2));
		if(hyperplane.find(hash)!=hyperplane.end()){
			ROS_INFO("hash collision: %d", hash);
			collision=true;
		}
		for(uint i=4;i<7;i++) vv.at(k).at(i)=pos.at(i-4);
		hyperplane[hash] = vv.at(k);
	}
	if(collision){
		ROS_INFO("WARNING: collision in hyperplane");
		throw "collision in hyperplane error";
		exit(-1);
	}
	ROS_INFO("successfully loaded hyperplane hash map without collisions and %d entries!", hyperplane.size());
}
