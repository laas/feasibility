#pragma once
#include <vector>
class ConstraintsChecker{
public:
	virtual bool isFeasible(  const std::vector<double> &p, 
			const std::vector< std::vector<double> > &obj);
private:
	ConstraintsChecker();

};

