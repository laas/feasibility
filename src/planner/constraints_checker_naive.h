#pragma once
#include "planner/constraints_checker.h"

class ConstraintsCheckerNaive: public ConstraintsChecker{
private:
	void loadActionSpace(const char *path);

public:
	ConstraintsCheckerNaive();
	~ConstraintsCheckerNaive();

	virtual bool 
	isFeasible(  
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj
	);

	virtual std::vector< std::vector<double> > 
	prepareObjectPosition(
		std::vector<ros::RVIZVisualMarker*> &obj, 
		double sf_x, double sf_y, double sf_yaw, char foot
	);
  virtual bool 
  isInCollision( 
      std::vector<ros::RVIZVisualMarker*> &objects_absolute, 
      std::vector< std::vector<double> > &fsi, 
      uint current_step_index );

};

