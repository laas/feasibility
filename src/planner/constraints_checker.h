#pragma once
#include <vector>
#include "rviz/rviz_visualmarker.h"

class ConstraintsChecker{
public:
	ConstraintsChecker(){};
	virtual 
	bool isFeasible(  const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj)=0;

	virtual 
	std::vector< std::vector<double> > 
	prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
			double sf_x, double sf_y, double sf_yaw, char foot)=0;
private:

};

