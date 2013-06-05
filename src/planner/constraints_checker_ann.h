#pragma once
#include <fann.h>
#include <unordered_map>
#include "planner/constraints_checker.h"

class ConstraintsCheckerANN: public ConstraintsChecker{
private:
	NeuralHashMap neuralMap;
public:
	ConstraintsCheckerANN();
	~ConstraintsCheckerANN();

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

private:
	double 
	computeNNOutput( 
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj
	);
	void loadNNParameters(const char *path);
};
