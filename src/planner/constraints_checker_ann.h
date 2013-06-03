#pragma once
#include "constraints_checker.h"
class ConstraintsCheckerANN{
public:
	virtual bool isFeasible(  const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj);
private:
	ConstraintsCheckerANN();

};


