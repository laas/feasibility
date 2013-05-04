#include "sampler_abstract_problem.h"
#include "util.h"
#include "rviz/rviz_visualmarker.h"

using namespace ros;
struct ProposalCylinder: public Proposal{
	ProposalCylinder(double h){
		// X,Y,R,H
		Eigen::VectorXd m(4);
		m << 0.8,0.8,0.1,0.0; //0.0 means keep fixed
		q_stddev=m;

		Eigen::VectorXd ql(4);
		ql << -2, -2, 0.01, h;
		q_constraints_low = ql;

		Eigen::VectorXd qh(4);
		qh << 2, 2, 5, h;
		q_constraints_high = qh;
	}
};

struct ProbabilityDistributionCylinder: public ProbabilityDistribution{
	ProbabilityDistributionCylinder(double m): ProbabilityDistribution(){
		this->mean = m;
	}
	double operator()(double d){
		return normpdf(d, mean, 0.17);
	}
private:
	double mean;
};
struct ObjectiveFunctionCylinder: public ObjectiveFunction{
	ros::TriangleObject *a;
	ros::TriangleObject *b;
	ObjectiveFunctionCylinder(  ros::TriangleObject *obj_a, ros::TriangleObject *obj_b ){
		a = obj_a;
		b = obj_b;
	}
	double operator()(Eigen::VectorXd &x){
		update(x);
		double d = b->distance_to(*a);
		return d;
	}
	void update( Eigen::VectorXd &x ){
		double x1 = x(0);
		double y1 = x(1);
		double r = x(2);
		double h = x(3);
		char command[100];
		sprintf(command, "octave -q scripts/create_tris_cylinderXYRH.m %f %f", r, h);
		ROS_INFO("%s",command);
		system(command);
		b->reloadBVH();
		b->setXYT(x1,y1,0); //keep on floor
	}
};

SamplingCTOCylinder::SamplingCTOCylinder(char *argv, double h, double m){
	robot = argv;
	getProbabilityDistribution(m);
	getObjectiveFunction();
	getProposal(h);
}
ObjectiveFunction* SamplingCTOCylinder::getObjectiveFunction(){
	std::string chair_file = get_chair_str();
	std::string robot_file = get_tris_str(this->robot);

	Geometry robot_pos;
	robot_pos.x = 0;
	robot_pos.y = 0;

	char command[100];
	sprintf(command, "octave -q scripts/create_tris_cylinderXYRH.m %f %f", 1.0,1.0);
	system(command);

	TriangleObject *robot = new TriangleObject(robot_file.c_str(), robot_pos);
	TriangleObjectFloor *cylinder = new TriangleObjectFloor(0.8, 0.5, "partXYRH.tris" );

	E = new ObjectiveFunctionCylinder(robot, cylinder);
	return E;
}
ProbabilityDistribution* SamplingCTOCylinder::getProbabilityDistribution(){
	return getProbabilityDistribution(0.6);
}

ProbabilityDistribution* SamplingCTOCylinder::getProbabilityDistribution(double m){
	p = new ProbabilityDistributionCylinder(m);
	return p;
}
Proposal* SamplingCTOCylinder::getProposal(){
	return getProposal(0.10);
}
Proposal* SamplingCTOCylinder::getProposal(double h){
	q = new ProposalCylinder(h);
	return q;
}
