#include "sampler.h"

#include <Eigen/Core>
#define DEBUG(x)
SamplingInterface::SamplingInterface(Logger &l){
	accepted_samples=0;
	rejected_samples=0;
	samples=0;
	this->logger= l;
}

void SamplingInterface::loop(Eigen::VectorXd &x, Eigen::VectorXd &ql, Eigen::VectorXd &qh, Eigen::VectorXd &stepsize, uint d){
	if(d==x.size()-1){
		while(x(d)<=qh(d)){
			double E = (*S->E)(x);
			double p = (*S->p)(E);
			accept(x);
			log(x, E);
			x(d)+=stepsize(d)+0.00001;//get rid of 0.0 stepsize, caused by std deviation of zero
			ROS_INFO("point %f %f %f %f", x(0), x(1), x(2), x(3));
		}
		return;
	}

	while(x(d)<=qh(d)){
		x(d+1)=ql(d+1);
		loop(x, ql, qh, stepsize, d+1);
		x(d)+=stepsize(d)+0.00001;//get rid of 0.0 stepsize, caused by std deviation of zero
	}
	return;
}

void SamplingInterface::uniform( uint Nsamples){
	//try to sample Nsamples uniformally in all dimensions
	assert(S!=NULL);

	uint Ndim = (*S->q).q_constraints_low.size(); //number of dimensions
	Eigen::VectorXd ql = (*S->q).q_constraints_low;
	Eigen::VectorXd qh = (*S->q).q_constraints_high;
	Eigen::VectorXd x = ql;
	Eigen::VectorXd stepsize = ql;
	Eigen::VectorXd length = ql;

	double sumLogLength=0.0;
	//equal spacing of points, such that Nsamples=N_0* ... *N_Ndim;
	//we set k*log(Nsamples) = log( l_0*...*l_Ndim), whereby l is the
	//length of the dimension, which is proportional to its samples
	//now we can compute k= log(l_0*...*l_Ndim)/log(Nsamples) and the
	//samples per dimensions as N_i = 2^( log(l_i) / k)
	for(uint i=0;i<Ndim;i++){
		length(i) = abs(qh(i)-ql(i))+1; //get rid of 0-1 interval
		sumLogLength+=log2(length(i));
	}

	double k = (double)sumLogLength/log2((double)Nsamples);
	for(uint i=0;i<Ndim;i++){
		uint samplesInThisDim = ceil(pow(2,log2(length(i))/(double)k));
		stepsize(i) = (length(i)-1)/(double)samplesInThisDim; //account for length+1 hack
		ROS_INFO("dim: %d, stepsize: %f, length: %f, samples: %d", i, stepsize(i), length(i), samplesInThisDim);
	}

	loop(x,ql,qh,stepsize,0);
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

	DEBUG(ROS_INFO("mcmc step %d: %f %f", samples, x_cand(0), x_cand(1));)
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
	DEBUG(ROS_INFO("accepted sample %f %f", x(0), x(1));)
	accepted_samples++;
	S->E->update(x);
}
void SamplingInterface::print(){
	ROS_INFO("accepted samples %d/%d -- %f", accepted_samples, samples, accepted_samples/(double)samples);
}
