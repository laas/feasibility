#pragma once
#include <vector>
#include "planner/constraints_checker.h"

//adding rectangular wall constraints to the feasibility function (if the
//footstep is outside the rectangular it will be ignored)

class ConstraintsCheckerWallConstraints: public ConstraintsChecker{
public:
	ConstraintsCheckerWallConstraints( ConstraintsChecker *cc, double xlow, double xhigh, double ylow, double yhigh);

	virtual 
	bool isFeasible(  const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj);
	virtual 
	std::vector< std::vector<double> > 
	prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
			double sf_x, double sf_y, double sf_yaw, char foot);
private:
	ConstraintsChecker *cc_;
	double xlow_;
	double xhigh_;
	double ylow_;
	double yhigh_;
};

