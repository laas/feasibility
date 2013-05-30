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
void SamplingInterface::hmc( uint Nsamples, double rmax=0.3 ){
	accepted_samples=0;
	rejected_samples=0;
	hmc_multi_step( Nsamples, rmax );
}
void SamplingInterface::hmc_step(){
}
void toObjectiveFunctionSpace(Eigen::VectorXd &x){
	x[2] = exp(x[2])/100.0;
}
void toSamplingSpace(Eigen::VectorXd &x){
	x[2] = log(x[2]*100.0);
}
//##########################################
//billiard modification of leapfrog, to account for
//constraints
//see MCMC using Hamiltonian dynamics, 2012, Neal,
//in Handbook of MCMC, page 37
//
// INPUT: x: current state, p: current momentum, pos: which variable in x has to be
// modified, l: lower constraint limit, u: upper constraint limit
void SamplingInterface::leap_frog_constraints_handler(Eigen::VectorXd &x, Eigen::VectorXd &p, uint pos, double l, double u){
	if(x[pos]>u){
		//upper constraint
		x[pos] = u - (x[pos]-u);
		p[pos] = -p[pos];
	}else{
		if(x[pos]<l){
			x[pos] = l + (l-x[pos]);
			p[pos] = -p[pos];
		}
	}
	//##########################################
}
		
void SamplingInterface::hmc_multi_step( uint Nsamples, double rmax){
	assert(S!=NULL);

	ROS_INFO("RMAX %f", rmax);
	ROS_INFO("RMAX LOG %f", log(100*rmax));

	//parameters
	double tau = 13; //steps of leapfrog
	double epsilon = 0.30; //stepsize of leapfrog

	double lambda=1e2; //modifying the p dist
	ObjectiveFunction *E = S->E;
	//ProbabilityDistribution *p = S->p;
	Proposal *q = S->q;

	toObjectiveFunctionSpace( x_old );
	Eigen::VectorXd g_old = E->grad(x_old);
	double E_old_dist = (*E)(x_old);
	toSamplingSpace( x_old );

	double E_old = lambda*E_old_dist*E_old_dist;

	for(uint l=0;l<Nsamples;l++){
		DEBUG(print();)
		Eigen::VectorXd p = randn_vec(0,0.09,x_old.size());
		p[2]=randn(0,0.09);
		//p[3]=0.0; //deactivate height variable
		//Eigen::VectorXd p = (*q)(x_old);

		double H_old = 0.5*p.dot(p) + E_old;

		Eigen::VectorXd x_new = x_old;
		Eigen::VectorXd g_new = g_old;

		//do tau leapfrog steps to simulate dynamics of the system
		for(uint t=0;t<tau;t++){
			p = p - 0.5*epsilon*g_new; //half step in p direction
			x_new = x_new + epsilon*p;

			//handle constraints by modifying momentum and state
			leap_frog_constraints_handler(x_new, p, 2, q->q_constraints_low[2], q->q_constraints_high[2]);
			leap_frog_constraints_handler(x_new, p, 3, q->q_constraints_low[3], q->q_constraints_high[3]); //use same upper limit

			toObjectiveFunctionSpace( x_new );
			g_new = E->grad(x_new);
			toSamplingSpace( x_new );
			p = p - epsilon*g_new*0.5; //another half step in p direction
		}

		toObjectiveFunctionSpace( x_new );
		double E_new_dist = (*E)(x_new);
		toSamplingSpace( x_new );

		double E_new= lambda*E_new_dist*E_new_dist;
		
		double H_new = 0.5*p.dot(p) + E_new;
		double dH = H_new - H_old;


		if( dH < 0.0){
			toObjectiveFunctionSpace( x_new );
			accept(x_new);
			logging( x_new, E_new_dist);
			toSamplingSpace( x_new );

			g_old = g_new;
			x_old = x_new;
			E_old = E_new;

		}else{
			if(rand(0,1) < exp(-dH)){
				toObjectiveFunctionSpace( x_new );
				accept(x_new);
				logging( x_new, E_new_dist);
				toSamplingSpace( x_new );
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
