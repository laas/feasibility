#pragma once
#include <vector>
#include "constraints_checker.h"

class ConstraintsCheckerSweptVolume: public ConstraintsChecker{
	SweptVolumeHashMap sweptvolumeMap;
	std::vector<ros::TriangleObject*> objects_;

public:

	ros::SweptVolumeObject* get_sv_from_hash( uint hash );
	ConstraintsCheckerSweptVolume();

	virtual bool isFeasible(  
			const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj);
	virtual 
	std::vector< std::vector<double> > 
	prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
			double sf_x, double sf_y, double sf_yaw, char foot);

private:

	double computeSVOutput( 
			const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj);


	void loadSweptVolumesToHashMap(const char *path);


};
