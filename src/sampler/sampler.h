#pragma once
#include <Eigen/Core>
#include "rviz/rviz_visualmarker.h"
#include "util.h"
#include "sampler_abstract_problem.h"

//doing mcmc or hmc for your convenience
struct SamplingInterface{
	AbstractSamplingProblem *S;
	uint accepted_samples;
	uint rejected_samples;
	uint samples;
	Eigen::VectorXd x_cand;
	Eigen::VectorXd x_old;
	double p_cand; //probability of current sample candidate
	double p_old; //probability of last sample
	Logger logger;

	SamplingInterface(Logger &l);
	void init( AbstractSamplingProblem *p );
	//void init( ros::TriangleObject *obj_a, ros::TriangleObject *obj_b);

	void log( Eigen::VectorXd &v, double d);
	void mcmc_step();
	void mcmc_multi_step( uint Nsamples );
	void accept( Eigen::VectorXd &x);
	void print();

	void mcmc(uint Nsamples);
	void hmc(uint Nsamples);

};

