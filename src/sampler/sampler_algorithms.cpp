#include "sampler.h"
#include "util_eigen.h"
#include <math.h>
#include <Eigen/Core>
#define DEBUG(x) x

SamplingInterface::SamplingInterface(Logger &l){
	accepted_samples=0;
	rejected_samples=0;
	samples=0;
	this->logger= l;
}
void SamplingInterface::init( AbstractSamplingProblem *p ){
	S = p;
	x_cand = S->q->init(); //next candidate
	x_old = x_cand; //last candidate
}
void SamplingInterface::logging( Eigen::VectorXd &v, double d){
	for(uint i=0;i<v.size();i++){
		logger("%f ", v(i));
	}
	logger("%f\n", d);
}

void SamplingInterface::mcmc( uint Nsamples){
	mcmc_multi_step( Nsamples );
	print();
}

void SamplingInterface::mcmc_step(){
	assert(S!=NULL);
	Eigen::VectorXd x_cand = (*S->q)(x_old);

	//DEBUG3(ROS_INFO("mcmc step %d: %f %f", samples, x_cand(0), x_cand(1));)
	DEBUG(print();)
	ObjectiveFunction *E = S->E;
	ProbabilityDistribution *p = S->p;
	Proposal *q = S->q;

	p_cand = (*p)( (*E)(x_cand) );
	double E_old = (*E)(x_old);
	p_old = (*p)( E_old );

	double a = p_cand / p_old;

	if(a>=1){
		x_old = x_cand;
		accept(x_old);
		logging(x_old, E_old);
	}else{
		double u = rand(0,1);
		if(u<a){
			x_old = x_cand;
			accept(x_old);
			logging( x_old, E_old);
		}else{
			rejected_samples++;
		}
	}
	p_old = p_cand;
	samples++;
}
void SamplingInterface::hmc( uint Nsamples ){
	hmc_multi_step( Nsamples );
}
void SamplingInterface::hmc_step(){
}
void SamplingInterface::hmc_multi_step( uint Nsamples ){
	assert(S!=NULL);

	//parameters
	double tau = 7; //steps of leapfrog
	double epsilon = 0.07; //stepsize of leapfrog

	double lambda=1e3; //modifying the p dist
	ObjectiveFunction *E = S->E;
	//ProbabilityDistribution *p = S->p;
	//Proposal *q = S->q;

	Eigen::VectorXd g_old = E->grad(x_old);
	double E_old_dist = (*E)(x_old);
	double E_old = lambda*E_old_dist*E_old_dist;


	for(uint l=0;l<Nsamples;l++){
		DEBUG(print();)
		Eigen::VectorXd p = randn_vec(0,0.08,x_old.size());
		p[2]=randn(0,0.03);
		p[3]=0.0;
		//Eigen::VectorXd p = (*q)(x_old);

		std::cout << p << std::endl;
		double H_old = 0.5*p.dot(p) + E_old;

		Eigen::VectorXd x_new = x_old;
		Eigen::VectorXd g_new = g_old;

		//do tau leapfrog steps to simulate dynamics of the system
		for(uint t=0;t<tau;t++){
			p = p - 0.5*epsilon*g_new; //half step in p direction
			x_new = x_new + epsilon*p;
			g_new = E->grad(x_new);
			if(x_new[2]>0.1){
				g_new[2]=1; //~ -0.01
			}else{
				if(x_new[2]<0.01){
					g_new[2]=-1;
				}
			}
			p = p - epsilon*g_new*0.5; //another half step in p direction
		}

		double E_new_dist = (*E)(x_new);
		double E_new= lambda*E_new_dist*E_new_dist;
		
		double H_new = 0.5*p.dot(p) + E_new;
		double dH = H_new - H_old;


		if( dH < 0.0){
			accept(x_new);
			logging( x_new, E_new_dist);
			g_old = g_new;
			x_old = x_new;
			E_old = E_new;

		}else{
			if(rand(0,1) < exp(-dH)){
				accept(x_new);
				logging( x_new, E_new_dist);
				g_old = g_new;
				x_old = x_new;
				E_old = E_new;
			}else{
				rejected_samples++;
			}
		}
		samples++;
	}
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
