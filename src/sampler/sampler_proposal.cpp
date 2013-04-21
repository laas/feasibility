#include "sampler_abstract_problem.h"
#include "util.h"

Eigen::VectorXd Proposal::operator()( Eigen::VectorXd &x ){
	Eigen::VectorXd q(x.size());
	for(uint i=0;i<x.size();i++){
		q(i) = randn(x(i), q_stddev(i));

		//lets assume that limits are rarely visited, such that
		//the stationary distribution will not be affected by
		//reseting samples under the limits
		if(q(i)<q_constraints_low(i)) q(i)=q_constraints_low(i);
		if(q(i)>q_constraints_high(i)) q(i)=q_constraints_high(i);
	}
	return q;
}
Eigen::VectorXd Proposal::init(){
	Eigen::VectorXd x(q_stddev.size());
	for(uint i=0;i<q_stddev.size();i++){
		x(i) = rand( q_constraints_low(i), q_constraints_high(i) );
	}
	return x;
}
