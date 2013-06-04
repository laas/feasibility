#include "sampler.h"
#include <math.h>
#include <Eigen/Core>
#define DEBUG(x) x

//helper function for uniform sampling, which samples uniformly in [ql,qh] with
//a certain stepsize
void SamplingInterface::loop(Eigen::VectorXd &x, Eigen::VectorXd &ql, Eigen::VectorXd &qh, Eigen::VectorXd &stepsize, uint d){
	if(d==x.size()-1){
		while(x(d)<=qh(d)){
			double E = (*S->E)(x);
			double p = (*S->p)(E);
			accept(x);
			logging(x, E);
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
//sample uniformly with equal number of samples per dimension (independent of
//length)
void SamplingInterface::uniform_normalized( uint Nsamples){
	//try to sample Nsamples uniformally in all dimensions
	assert(S!=NULL);

	uint Ndim = (*S->q).q_constraints_low.size(); //number of dimensions
	Eigen::VectorXd ql = (*S->q).q_constraints_low;
	Eigen::VectorXd qh = (*S->q).q_constraints_high;
	Eigen::VectorXd x = ql;
	Eigen::VectorXd stepsize = ql;
	Eigen::VectorXd length = ql;

	uint Ndim_norm = 0;
	for(uint i=0;i<Ndim;i++){
		length(i) = fabs(qh(i)-ql(i));
		if(length(i)>0.001){
			Ndim_norm++;
		}
	}
	double n=exp(log((double)Nsamples)/(double)Ndim_norm);
	ROS_INFO("log: %f\n", log(Nsamples));
	for(uint i=0;i<Ndim;i++){
		if(length(i)>0.001){
			stepsize(i) = (length(i))/(double)n; 
			ROS_INFO("dim: %d, stepsize: %f, length: %f, samples: %f", i, stepsize(i), length(i), n);
		}else{
			stepsize(i) = 0.001;
			ROS_INFO("dim: %d, stepsize: %f, length: %f, samples: %f", i, stepsize(i), length(i), 0);
		}
	}

	loop(x,ql,qh,stepsize,0);
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
