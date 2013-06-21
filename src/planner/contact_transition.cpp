#include <unordered_map>
#include <algorithm> //lowerbound, sort, max_element
#include <vector> //max_element
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
	this->g.setRPYRadian(0,0, g.getYawRadian() );
}
double ContactTransition::GoalDistanceEstimate( ContactTransition &nodeGoal ){
	//heuristic = distance to goal (classical l2 norm)
	double xg = nodeGoal.g.x;
	double yg = nodeGoal.g.y;
	double x = this->g.x;
	double y = this->g.y;
	//return norml1(x,xg,y,yg);
	return norml2(x,xg,y,yg);
}
void ContactTransition::cleanStatic(){
	std::vector<ros::RVIZVisualMarker*>::iterator oit;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		if(*oit!=NULL) *oit=NULL;
	}
	objects.clear();
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
	ff_abs.setRPYRadian(0,0,ff_abs_yaw);

	return ff_abs;
}
	
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


	std::vector<double> ff_rel = vecD(ff_rel_x, ff_rel_y, ff_rel_yaw);
	double ff_hash = hashit<double>(ff_rel);

	bool ff_start_feasible = true;
	//Check first foot first -> if it is infeasible, we do not need to test
	//the rest 
	if(constraints->actionSpace.find(ff_hash)!=constraints->actionSpace.end()){
		//support foot exists in actionSpace, which means that it is not
		//the first footstep and we can check its feasibility
		std::vector<double> ff_rel_table = constraints->actionSpace.find(ff_hash)->second;
		ff_start_feasible = constraints->isFeasible(ff_rel_table, obj);
		feasibilityChecks++;

		//position constraints
		if(sf_abs_x < -0.3 || sf_abs_x > 3.3 || sf_abs_y < -0.76 || sf_abs_y > 0.76){
			ff_start_feasible = false;
		}
	}

	if(ff_start_feasible){
		DEBUG(
			//sleep(1);
			if(foot=='L'){
				ros::ColorFootMarker rp(parent->g.x, parent->g.y, parent->g.getYawRadian(),"white");
				rp.reset();
				rp.publish();
			}

		)
		TIMER_DEBUG( timer->begin("actionExpansion") );
		ActionSpace::const_iterator it;
		for(  it = constraints->actionSpace.begin(); it != constraints->actionSpace.end(); ++it ){

			feasibilityChecks++;
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

				DEBUG(
					if(foot=='L'){
						ros::ColorFootMarker rp(ng.x, ng.y, ng.getYawRadian(), "green");
						rp.publish();
					}else{
						ros::ColorFootMarker rp(ng.x, ng.y, ng.getYawRadian(), "red");
						rp.publish();
					}
				)
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

