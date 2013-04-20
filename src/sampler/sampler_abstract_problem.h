#pragma once
#include <Eigen/Core>

struct Proposal{
	Eigen::VectorXd q_stddev;
	Eigen::VectorXd q_constraints_low;
	Eigen::VectorXd q_constraints_high;
	Eigen::VectorXd operator()( Eigen::VectorXd &x );
	Eigen::VectorXd init();
};
struct ProbabilityDistribution{
	virtual double operator()(double d) = 0;
};
struct ObjectiveFunction{
	virtual double operator()(Eigen::VectorXd &x) = 0; //evaluate E(x)
	virtual void update( Eigen::VectorXd &x ) = 0; //update internal structures according to x, if necesary
};
struct AbstractSamplingProblem{
	ObjectiveFunction *E;
	ProbabilityDistribution *p;
	Proposal *q;

	virtual ObjectiveFunction* getObjectiveFunction() = 0;
	virtual ProbabilityDistribution* getProbabilityDistribution() = 0;
	virtual Proposal* getProposal() = 0;
};
struct SamplingCTOCylinder: public AbstractSamplingProblem{
	char *robot;
	SamplingCTOCylinder(char *argv); 
	ObjectiveFunction* getObjectiveFunction();
	ProbabilityDistribution* getProbabilityDistribution();
	Proposal* getProposal();
};
