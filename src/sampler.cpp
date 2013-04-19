#include "sampler.h"

#include <Eigen/Core>
#define DEBUG(x) x
SamplingInterface::SamplingInterface(Logger &l){
	accepted_samples=0;
	rejected_samples=0;
	this->logger= l;
}

void SamplingInterface::mcmc( uint Nsamples){
	mcmc_multi_step( Nsamples );
	print();
}
void SamplingInterface::init( AbstractSamplingProblem *p ){
	S = p;
	x_cand = S->q->init(); //next candidate
	x_old = x_cand; //last candidate
}
void SamplingInterface::log( Eigen::VectorXd &v, double d){
	for(uint i=0;i<v.size();i++){
		logger("%f ", v(i));
	}
	logger("%f\n", d);
}

void SamplingInterface::mcmc_step(){
	assert(S!=NULL);
	Eigen::VectorXd x_cand = (*S->q)(x_old);

	p_cand = (*S->p)( (*S->E)(x_cand) );
	double E_old = (*S->E)(x_old);
	p_old = (*S->p)( E_old );

	double a = p_cand / p_old;

	if(a>=1){
		x_old = x_cand;
		accept(x_old);
		log(x_old, E_old);
	}else{
		double u = rand(0,1);
		if(u<a){
			x_old = x_cand;
			accept(x_old);
			log( x_old, E_old);
		}else{
			rejected_samples++;
		}
	}
	p_old = p_cand;
	samples++;
}
void SamplingInterface::mcmc_multi_step( uint Nsamples ){
	for(uint i=0;i<Nsamples;i++){
		mcmc_step();
	}
}

void SamplingInterface::accept( Eigen::VectorXd &x){
	accepted_samples++;
	S->E->update(x);
}
void SamplingInterface::print(){
	ROS_INFO("accepted samples %d/%d -- %f", accepted_samples, samples, accepted_samples/(double)samples);
}
