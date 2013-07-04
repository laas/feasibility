#pragma once
#include <Eigen/Core>
#include "rviz/rviz_visualmarker.h"
#include "util/util.h"
#include "sampler_problem_abstract.h"

//doing mcmc or hmc for your convenience
class SamplingInterface{
	AbstractSamplingProblem *S;
	uint accepted_samples;
	uint rejected_samples;
	uint samples;

	Eigen::VectorXd x_cand; //store samples from mcmc steps 
	Eigen::VectorXd x_old;
	double p_cand; //probability of current sample candidate
	double p_old; //probability of last sample

	Logger logger;

	void logging( Eigen::VectorXd &v, double d);
	void leap_frog_constraints_handler(Eigen::VectorXd &x, Eigen::VectorXd &p, uint pos, double l, double u);
	void hmc_step(); //Hamiltonian Monte Carlo (as described in ITILA, Ch. 30, David MacKay)
	void hmc_multi_step( uint Nsamples, double rmax);
	void mcmc_step(); //Metropolis-Hastings MCMC (as described in PRML, Bishop)
	void mcmc_multi_step( uint Nsamples );
	void accept( Eigen::VectorXd &x);
	void loop(Eigen::VectorXd &x, Eigen::VectorXd &ql, Eigen::VectorXd &qh, Eigen::VectorXd &stepsize, uint d);
	void print();
public:
	SamplingInterface(Logger &l);
	void init( AbstractSamplingProblem *p );
	void uniform(uint Nsamples);
	void uniform_normalized(uint Nsamples);
	void mcmc(uint Nsamples);
	void hmc(uint Nsamples, double rmax);

};

