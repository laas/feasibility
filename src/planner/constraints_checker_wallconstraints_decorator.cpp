#include "constraints_checker_wallconstraints_decorator.h"

ConstraintsCheckerWallConstraints::ConstraintsCheckerWallConstraints( 
		ConstraintsChecker *cc, double xlow, double xhigh, double ylow, double yhigh){
	this->cc_ = cc;
	this->xlow_ = xlow;
	this->xhigh_ = xhigh;
	this->ylow_ = ylow;
	this->yhigh_ = yhigh;
}
bool ConstraintsCheckerWallConstraints::isFeasible(  const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	//if(p.at(0) < xlow_ || p.at(0) > xhigh_) return false;
	//if(p.at(1) < ylow_ || p.at(1) > yhigh_) return false;
	return cc_->isFeasible(p,obj);
}

std::vector< std::vector<double> > 
ConstraintsCheckerWallConstraints::prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &objects, 
		double sf_x, double sf_y, double sf_yaw, char foot){
	return cc_->prepareObjectPosition( objects, sf_x, sf_y, sf_yaw, foot);
}
