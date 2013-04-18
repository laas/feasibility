#pragma once
#include "rviz/rviz_visualmarker.h"
#include "util.h"

//doing mcmc or hmc for your convenience
class SampleGenerator{
	int accepted_samples;
	int rejected_samples;
	const static double mu = 1.0;
	const static double theta = 0.17;

	Logger log;

public:
	SampleGenerator(Logger &l);
	double draw_gaussian_proposal_dist_positive(double x);
	double draw_gaussian_proposal_dist(double x);
	void accept_sample(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b);
	void accept_sample_cyl(double x, double y, double r, double z, ros::TriangleObject *obj_a, ros::TriangleObject *obj_b);
	void object_update_pos(ros::TriangleObject &obj, double x, double y);
	double update_cyl_pos(ros::TriangleObject *obj_b, double x, double y, double r, double z);
	double dp(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b);
	double p(double x, double y, double t, ros::TriangleObject &obj_a, ros::TriangleObject &obj_b);
	double p_cyl(double x, double y, double r, double z, ros::TriangleObject *obj_a, ros::TriangleObject *obj_b);
	void mcmc( ros::TriangleObject *obj_a, ros::TriangleObject *obj_b, uint Nsamples);


};
