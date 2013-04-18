#pragma once
#include "rviz/rviz_visualmarker.h"
#include "util.h"

//doing mcmc or hmc for your convenience
class SampleGenerator{
	int accepted_samples;
	int rejected_samples;
	const static double mu = 0.0;
	const static double theta = 0.17;


	Logger log;

public:
	SampleGenerator(Logger &l){
		accepted_samples=0;
		rejected_samples=0;
		this->log = l;
	}

	double draw_gaussian_proposal_dist(double x){
		double stddev = 1.2;
		return randn(x, stddev);
	}
	void accept_sample(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b){
		obj_b.setXYT(x,y,t);
		accepted_samples++;
		//a.rviz_publish();
	}

	double dp(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b){
		obj_b.setXYT(x,y,t);
		//return unnormalized distribution
		double d=obj_b.distance_to(obj_a);

		double p = normpdf(d, mu, theta);
		ROS_INFO("POS: [accept rate %f] %f %f %f -- distance %f -> %f", (double)accepted_samples/(double)(accepted_samples+rejected_samples+1.0), x, y, t, d, p);

		return p;
	}
	double p(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b){
		obj_b.setXYT(x,y,t);
		//return unnormalized distribution
		double d=obj_b.distance_to(obj_a);

		double p = normpdf(d, mu, theta);
		ROS_INFO("POS: [accept rate %f] %f %f %f -- distance %f -> %f", (double)accepted_samples/(double)(accepted_samples+rejected_samples+1.0), x, y, t, d, p);

		return p;
	}

	void mcmc( ros::TriangleObject &obj_a, ros::TriangleObject &obj_b, uint Nsamples){
		int lInt = 1.0;
		int hInt = 1.4;

		double x= rand(lInt, hInt);
		double y= rand(lInt, hInt);
		double t= rand(lInt, hInt);
		//double x= 0.0;
		//double y= 0.0;

		//proposal distribution
		double x_old = x;
		double y_old = y;
		double t_old = t;
		double p_old = p(x_old, y_old, t_old, obj_a, obj_b);
		for(uint i=0;i<Nsamples;i++){
			double x_cand = draw_gaussian_proposal_dist(x);
			double y_cand = draw_gaussian_proposal_dist(y);
			double t_cand = draw_gaussian_proposal_dist(t);
			double p_cur = p(x_cand, y_cand, t_cand, obj_a, obj_b);

			double a = p_cur / p_old;

			if(a>=1){
				x_old = x_cand;
				y_old = y_cand;
				t_old = t_cand;
				accept_sample(x_old, y_old, t_old, obj_a, obj_b);
				log("%f %f %f %f", x_old, y_old, t_old, obj_b.distance_to(obj_a));
			}else{
				double u = rand(0,1);
				if(u<a){
					x_old = x_cand;
					y_old = y_cand;
					t_old = t_cand;
					accept_sample(x_old, y_old, t_old, obj_a, obj_b);
					log("%f %f %f %f", x_old, y_old, t_old, obj_b.distance_to(obj_a));
				}else{
					x_old = x_old;
					y_old = y_old;
					t_old = t_old;
					rejected_samples++;
				}
			}
			p_old = p_cur;

		}
		ROS_INFO("accepted samples %d/%d -- %f", accepted_samples, Nsamples, accepted_samples/(double)Nsamples);
	}



};
