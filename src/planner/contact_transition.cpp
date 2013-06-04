#include <algorithm>//lowerbound, sort
//#include <map>
#include <unordered_map>
#include <algorithm> //max_element
#include "contact_transition.h"
#include "util.h"
#include "rviz/rviz_visualmarker.h"
#include <Eigen/Geometry>
#include <Eigen/Core>

#define DEBUG(x)
#define TIMER_DEBUG(x) x

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
	return norml2(x,xg,y,yg);
}
bool ContactTransition::IsGoal( ContactTransition &nodeGoal ){
	return norml2(this->g.x, nodeGoal.g.x, this->g.y, nodeGoal.g.y) < 0.3;
}

ros::Geometry ContactTransition::computeAbsFFfromRelFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_rel_x, double ff_rel_y, double ff_rel_yaw,
		char sf_foot){
	//absolute position of next pos of FF
	double ff_abs_x = sf_abs_x + cos(sf_abs_yaw)*ff_rel_x - sin(sf_abs_yaw)*ff_rel_y;
	double ff_abs_y = sf_abs_y + sin(sf_abs_yaw)*ff_rel_x + cos(sf_abs_yaw)*ff_rel_y;
	double ff_abs_yaw = ff_rel_yaw + sf_abs_yaw;

	if(sf_foot == 'R'){
		//reflection at x-axis [1 0,0 -1], following rotation around z and
		//translation to x,y
		ff_abs_x = sf_abs_x + cos(sf_abs_yaw)*ff_rel_x + sin(sf_abs_yaw)*ff_rel_y;
		ff_abs_y = sf_abs_y + sin(sf_abs_yaw)*ff_rel_x - cos(sf_abs_yaw)*ff_rel_y;
		ff_abs_yaw = -(ff_rel_yaw - sf_abs_yaw);
	}
	while(ff_abs_yaw>M_PI) ff_abs_yaw-=2*M_PI;
	while(ff_abs_yaw<-M_PI) ff_abs_yaw+=2*M_PI;

	ros::Geometry ff_abs;
	ff_abs.x = ff_abs_x;
	ff_abs.y = ff_abs_y;
	ff_abs.setYawRadian(ff_abs_yaw);

	return ff_abs;
}
	
/*
void ContactTransition::showSuccessors( double x, double y, double t, char L_or_R){
	HashMap::const_iterator it;
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

		double valX = it->second.at(5);
		double valY = it->second.at(6);
		double valT = toRad(it->second.at(7)*100.0);

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
*/
bool ContactTransition::GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node ){
	char foot;
	ContactTransition *parent = this;

	(parent->L_or_R == 'L')? foot='R':foot='L';

	//NOTE:
	//relative means always with respect to the support foot
	//absolute means the position in the global world coordinate frame

	//get absolute position of support foot (SF)
	double sf_abs_x = parent->g.x;
	double sf_abs_y = parent->g.y;
	double sf_abs_yaw = parent->g.getYawRadian();

	TIMER_DEBUG(timer->begin("prepare"));
	//project all objects into the reference frame of the SF
	std::vector< std::vector<double> > obj = 
		constraints->prepareObjectPosition(objects, sf_abs_x, sf_abs_y, sf_abs_yaw, parent->L_or_R);
	TIMER_DEBUG(timer->end("prepare"));

	//get the relative position of the starting free foot (FF)
	double ff_rel_x = parent->rel_x_parent;
	double ff_rel_y = parent->rel_y_parent;
	double ff_rel_yaw = parent->rel_yaw_parent;

	std::vector<double> ff_rel(3);
	ff_rel.at(0)=ff_rel_x; ff_rel.at(1)=ff_rel_y; ff_rel.at(2)=ff_rel_yaw;

	//TODO: include first half-step!
	//ROS_INFO("%f %f %f\n", sf.at(0),sf.at(1),sf.at(2));
	//std::vector<double> sf_abs = constraints->actionSpace.find(hashit<double>(sf))->second;

	std::vector<double> sf = vecD(ff_rel_x, ff_rel_y, ff_rel_yaw);
	double sf_hash = hashit<double>(sf);

	bool sf_feasible = true;
	if(constraints->actionSpace.find(sf_hash)!=constraints->actionSpace.end()){
		//support foot exists in actionSpace, which means that it is not
		//the first footstep and we can check its feasibility
		
		//std::vector<double> sf_abs = constraints->actionSpace.find(sf_hash)->second;
		//sf_feasible = constraints->isFeasible(sf_abs, obj);
	}

	if(sf_feasible){
		uint counter=0;
		TIMER_DEBUG( timer->begin("actionExpansion") );

		ActionSpace::const_iterator it;
		for(  it = constraints->actionSpace.begin(); it != constraints->actionSpace.end(); ++it ){

			if(constraints->isFeasible( it->second, obj)){

				//relative position of the next proposed pos of FF
				TIMER_DEBUG( timer->begin("ff") );

				double next_ff_rel_x = it->second.at(0);
				double next_ff_rel_y = it->second.at(1);
				double next_ff_rel_yaw = toRad(it->second.at(2)/100.0);

				ros::Geometry ng = computeAbsFFfromRelFF(sf_abs_x, sf_abs_y, sf_abs_yaw, next_ff_rel_x, next_ff_rel_y, next_ff_rel_yaw, L_or_R);

				ContactTransition next(ng);
				next.L_or_R = foot;
				next.rel_x_parent = it->second.at(0);
				next.rel_y_parent = it->second.at(1);
				next.rel_yaw_parent = it->second.at(2);

				next.cost_so_far = 0.0;
				astarsearch->AddSuccessor(next);

				//DEBUG(
					if(rand(0,1)>0.9){
						if(foot=='L'){
							ros::LeftFootMarker rp(ng.x, ng.y, ng.getYawRadian());
							rp.publish();
						}else{
							ros::RightFootMarker rp(ng.x, ng.y, ng.getYawRadian());
							rp.publish();
						}
					}
				//);
				TIMER_DEBUG( timer->end("ff") );
				continue;
			}
		}//foreach action
		TIMER_DEBUG( timer->end("actionExpansion") );
	}

	return true;
}

void ContactTransition::setConstraintsChecker( ConstraintsChecker *c ){
	ContactTransition::constraints = c;
}

double ContactTransition::GetCost( ContactTransition &successor ){
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

/*
//############################################################################
//## Fann functions
//############################################################################
//input: obj in euclidean coordinates (x,y,r)| relative to footstep of free
//foot (FF), in X,Y,T, whereby T is degree divided by 100
double ContactTransition::computeNNDistance( const std::vector<double> &p, const std::vector< std::vector<double> > &obj){


*/

