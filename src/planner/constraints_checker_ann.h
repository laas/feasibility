#pragma once
#include <fann.h>
#include <unordered_map>
#include <dirent.h>
#include "constraints_checker.h"

class ConstraintsCheckerANN: public ConstraintsChecker{
private:
	typedef std::unordered_map< int, struct fann*> NeuralHashMap; 
	NeuralHashMap neuralMap;
public:
	ConstraintsCheckerANN();
	virtual bool isFeasible(  const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj);

// support foot (SF)
//get object position in the coordinate system of the SF -- and prune objects
//which are too far away
	virtual 
	std::vector< std::vector<double> > 
	prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
			double sf_x, double sf_y, double sf_yaw, char foot);
private:
	void loadNNParameters(const char *path);

};


