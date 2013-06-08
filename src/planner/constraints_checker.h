#pragma once
#include <unordered_map>
#include <vector>
#include "rviz/rviz_visualmarker.h"

typedef std::unordered_map< int, struct fann*> NeuralHashMap; 
typedef std::unordered_map< int, std::vector<double>> ActionSpace; 
typedef std::unordered_map< int, ros::SweptVolumeObject*> SweptVolumeHashMap; 

//imagine a cylinder around all swept volumes (including the symmetrical ones) 
// checking feasibility outside this cylinder does not make snese, so we prune everything outside
// MAX_SWEPT_VOLUME_LIMIT is the radius of this imaginary cylinder
const double MAX_SWEPT_VOLUME_LIMIT = 1.1; 

class ConstraintsChecker{
public:
	ActionSpace actionSpace;
	ConstraintsChecker(){};
	~ConstraintsChecker(){
		actionSpace.clear();
	}
	virtual 
	bool isFeasible(  const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj)=0;

	virtual 
	std::vector< std::vector<double> > 
	prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
			double sf_x, double sf_y, double sf_yaw, char foot)=0;

};
