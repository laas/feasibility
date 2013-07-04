#include <Eigen/Core>
#include "util.h"

Eigen::VectorXd randn_vec(double m, double stddev, uint size){
	Eigen::VectorXd v(size);
	for(uint i=0;i<size;i++){
		v[i]=randn(m,stddev);
	}
	return v;
}

