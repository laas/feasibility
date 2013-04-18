#include "sampler.h"

#define DEBUG(x) x
SampleGenerator::SampleGenerator(Logger &l){
	accepted_samples=0;
	rejected_samples=0;
	this->log = l;
}

double SampleGenerator::draw_gaussian_proposal_dist_positive(double x){
	double stddev = 0.1;
	if(x<0){
		x=0;
	}
	double res=-1;

	while( res <= 0){
		res = randn(x, stddev);
	}
	return res;
}
double SampleGenerator::draw_gaussian_proposal_dist(double x){
	double stddev = 1.2;
	return randn(x, stddev);
}
void SampleGenerator::accept_sample(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b){
	obj_b.setXYT(x,y,t);
	accepted_samples++;
}
void SampleGenerator::accept_sample_cyl(double x, double y, double r, double z, ros::TriangleObject *obj_a, ros::TriangleObject *obj_b){
	update_cyl_pos(obj_b, x, y, r, z);
	accepted_samples++;
}
void SampleGenerator::object_update_pos(ros::TriangleObject &obj, double x, double y){
	obj.g.x = x;
	obj.g.y = y;
}
double SampleGenerator::update_cyl_pos(ros::TriangleObject *obj_b, double x, double y, double r, double z){
	char command[100];
	sprintf(command, "octave -q scripts/create_tris_cylinder.m %f %f", r, z);
	system(command);
	//*ros::TriangleObjectFloor *tmp = dynamic_cast<ros::TriangleObjectFloor*>(&obj_b);
	//delete obj_b;
	obj_b->reloadBVH();
	obj_b->setXYT(x,y,0);
		//new ros::TriangleObjectFloor(x,y,"part.tris");
	//obj_b = *tmp;
	//DEBUG( obj_b->publish());
	//object_update_pos(obj_b, x, y);
}
double SampleGenerator::dp(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b){
	obj_b.setXYT(x,y,t);
	//return unnormalized distribution
	double d=obj_b.distance_to(obj_a);

	double p = normpdf(d, mu, theta);
	ROS_INFO("POS: [accept rate %f] %f %f %f -- distance %f -> %f", (double)accepted_samples/(double)(accepted_samples+rejected_samples+1.0), x, y, t, d, p);

	return p;
}
double SampleGenerator::p(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b){
	obj_b.setXYT(x,y,t);
	//return unnormalized distribution
	double d=obj_b.distance_to(obj_a);

	double p = normpdf(d, mu, theta);
	ROS_INFO("POS: [accept rate %f] %f %f %f -- distance %f -> %f", (double)accepted_samples/(double)(accepted_samples+rejected_samples+1.0), x, y, t, d, p);

	return p;
}
double SampleGenerator::p_cyl(double x, double y, double r, double z, ros::TriangleObject *obj_a, ros::TriangleObject *obj_b){

	update_cyl_pos(obj_b, x, y, r, z);

	//return unnormalized distribution
	double d=obj_b->distance_to(*obj_a);

	double p = normpdf(d, mu, theta);
	ROS_INFO("POS: [accept rate %f] %f %f %f %f -- distance %f -> %f", (double)accepted_samples/(double)(accepted_samples+rejected_samples+1.0), x, y, r,z, d, p);

	return p;
}

void SampleGenerator::mcmc( ros::TriangleObject *obj_a, ros::TriangleObject *obj_b, uint Nsamples){
	int lInt = 1.0;
	int hInt = 1.4;

	double x= rand(lInt, hInt);
	double y= rand(lInt, hInt);
	double z= rand(0.1, hInt);
	double r= rand(0.1, 0.3);
	//double x= 0.0;
	//double y= 0.0;
	//

	//proposal distribution
	double x_old = x;
	double y_old = y;
	double r_old = r;
	double z_old = z;
	double p_old = p_cyl(x_old, y_old, r_old, z_old, obj_a, obj_b);
	for(uint i=0;i<Nsamples;i++){
		double x_cand = draw_gaussian_proposal_dist(x);
		double y_cand = draw_gaussian_proposal_dist(y);
		double r_cand = draw_gaussian_proposal_dist_positive(r);
		double z_cand = 0.1;//draw_gaussian_proposal_dist_positive(z);
		double p_cur = p_cyl(x_cand, y_cand, r_cand, z_cand, obj_a, obj_b);

		double a = p_cur / p_old;

		if(a>=1){
			x_old = x_cand;
			y_old = y_cand;
			r_old = r_cand;
			z_old = z_cand;
			accept_sample_cyl(x_old, y_old, r_old, z_old, obj_a, obj_b);
			log("%f %f %f %f %f", x_old, y_old, r_old, z_old, obj_b->distance_to(*obj_a));
		}else{
			double u = rand(0,1);
			if(u<a){
				x_old = x_cand;
				y_old = y_cand;
				r_old = r_cand;
				z_old = z_cand;
				accept_sample_cyl(x_old, y_old, r_old, z_old, obj_a, obj_b);
				log("%f %f %f %f %f", x_old, y_old, r_old, z_old, obj_b->distance_to(*obj_a));
			}else{
				x_old = x_old;
				y_old = y_old;
				r_old = r_cand;
				z_old = z_cand;
				rejected_samples++;
			}
		}
		p_old = p_cur;

	}
	ROS_INFO("accepted samples %d/%d -- %f", accepted_samples, Nsamples, accepted_samples/(double)Nsamples);
}
