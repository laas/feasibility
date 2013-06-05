#include "constraints_checker_wallconstraints_decorator.h"

ConstraintsCheckerWallConstraints::ConstraintsCheckerWallConstraints( 
		ConstraintsChecker *cc, double xlow, double xhigh, double ylow, double yhigh){
	this->_cc = cc;
	this->_xlow = xlow;
	this->_xhigh = xhigh;
	this->_ylow = ylow;
	this->_yhigh = yhigh;
}
bool ConstraintsCheckerWallConstraints::isFeasible(  const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	//if(p.at(0) < _xlow || p.at(0) > _xhigh) return false;
	//if(p.at(1) < _ylow || p.at(1) > _yhigh) return false;
	return _cc->isFeasible(p,obj);
}

std::vector< std::vector<double> > 
ConstraintsCheckerWallConstraints::prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &objects, 
		double sf_x, double sf_y, double sf_yaw, char foot){
	return _cc->prepareObjectPosition( objects, sf_x, sf_y, sf_yaw, foot);
}
