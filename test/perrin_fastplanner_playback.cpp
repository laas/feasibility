#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>

#include <fast-replanning/fast-replanning-interface.hh>

#include "util/util.h"
#include "planner/trajectory_visualizer.h"
#include "rviz/visualmarker.h"
#include "environment/environment.h"

#include "genFullBodyTrajectory.h"
#include <analyticalPG/newPGstepStudy.h>                                                                                                                                   

//###############################################################################
//###############################################################################
//definitions
//###############################################################################
//###############################################################################


#define MARGIN 0.01
#define COEF_MAX_SLIDE_UP 0.8
#define COEF_MAX_SLIDE_DOWN 0.8
#define T_STOP_DICHO 0.1

#define STEP_LENGTH 1.5

#define HALF_FOOT_FRONT 0.140
#define HALF_FOOT_BACK 0.095
#define HALF_FOOT_SIDE 0.0675

//CFunEval *evalFunction;
//CFunPick *pickFunction;
CnewPGstepStudy *NPSS = new CnewPGstepStudy(STEP_LENGTH); 
//checkCollisionsPQP *CC;
CgenFullBodyTrajectory * CGFBT = new CgenFullBodyTrajectory();
//###############################################################################
//###############################################################################

struct step
{
    //UNITS are meters and radians

    //Step parameters
    double x;
    double y;                                                                                                                                                              
    double theta;
    double f;
    char left_or_right;

    //Position before move
    double abs_x;
    double abs_y;
    double abs_theta;

    //Smoothing
    StepFeatures stepFeaturesUP;
    StepFeatures stepFeaturesDOWN;
    double slideUP;
    double slideDOWN;
    bool smoothed;
};
#include "helperFunctions.h"


std::vector<double> generateWholeBodyMotionFromFootsteps(
		std::vector<std::vector<double> > fsi, int lastStepSmoothed)
{
	std::vector<double> q;
	//###############################################################################
	//###############################################################################
	//fsi to step format
	//###############################################################################
	//###############################################################################

	if(lastStepSmoothed > fsi.size()-1){
		return q;
	}
	vector<step> vectStep;
	halfFootStepToStep(fsi, vectStep);

	//###############################################################################
	//###############################################################################
	// NO smooth STEPS
	//computeStepFeaturesWithoutSmoothing
	//###############################################################################
	//###############################################################################

	recomputeZMP(vectStep, 'L');

	double defaultSlide = -0.1;

	StepFeatures stepF, stepUP, stepDOWN;
	stepUP = vectStep.at(0).stepFeaturesUP;
	stepDOWN = vectStep.at(0).stepFeaturesDOWN;

	stepF = stepUP;
	vectStep[0].slideUP = 0.0;
	vectStep[0].slideDOWN = defaultSlide;
	NPSS->addStepFeaturesWithSlide(stepF, stepDOWN ,defaultSlide);

	for(unsigned int i=1;i<vectStep.size();i++){
		stepUP = vectStep.at(i).stepFeaturesUP;
		stepDOWN = vectStep.at(i).stepFeaturesDOWN;
		NPSS->addStepFeaturesWithSlide(stepF, stepUP ,defaultSlide);
		NPSS->addStepFeaturesWithSlide(stepF, stepDOWN , defaultSlide);
		vectStep[i].slideUP = defaultSlide;
		vectStep[i].slideDOWN = defaultSlide;
	}


//###############################################################################
//###############################################################################
//Generate full body trajectory
//###############################################################################
//###############################################################################
	int size;
	if(lastStepSmoothed == 0){
		size = 
		  (int)(vectStep[lastStepSmoothed].stepFeaturesUP.size
			+ vectStep[lastStepSmoothed].stepFeaturesDOWN.size
			+ 0.001);
	}else{
		size = 
		  (int)(vectStep[lastStepSmoothed].stepFeaturesUP.size
			+ vectStep[lastStepSmoothed].stepFeaturesDOWN.size
			//+ 200.0*(vectStep [lastStepSmoothed-1].slideUP
				 //+ vectStep [lastStepSmoothed-1].slideDOWN) 
			+ 0.001);
	}
	int firstIndex = getFirstIndex( vectStep, lastStepSmoothed);                                                                                                   
	ROS_INFO("stepLength %d, firstIndex %d, size %d", vectStep.size(), firstIndex, size);

	if(size>0){

		vector<vector<double> > trajTimedRadQ;
		CGFBT->generateTrajectory( trajTimedRadQ, stepF, firstIndex, size);
		q = createArticularValuesVector(trajTimedRadQ, stepF, firstIndex, 0, size);
	}

	return q;

}
int main( int argc, char** argv )
{
	using namespace ros;
	ros::init(argc, argv, "footstep_visualizer");
	ros::NodeHandle n;
	ros::Rate r(1);

	std::string prefix = get_data_path();
	std::string chair_file = get_chair_str();
	std::string robot_file = get_robot_str();

	//######################################################
	// set robot start
	//######################################################
	std::vector<double> start;
	start.push_back(0.0);
	start.push_back(0.0);

	//######################################################
	// set goal 
	//######################################################
	std::vector<double> goal;
	goal.push_back(2.5); goal.push_back(0.0); goal.push_back(0.0);
	TrajectoryVisualizer *tv = new TrajectoryVisualizer(0,0);

	CSVReader data_q("playback_q.dat");
	//std::vector<double> q = data_q.getV();
	//ROS_INFO("q size: %d", q.size());


	r.sleep();
	FootMarker marker(0,0,0);

	Environment* environment = Environment::getSalleBauzil();


	CSVReader data_steps("playback_steps.dat");
	
	std::vector<std::vector<double> > fsi;
	fsi = data_steps.getVV(11);
	if(ros::ok()){
		marker.reset();

		double last_xL = 0;
		double last_yL = 0;
		double last_tL = 0;
		double last_xR = 0;
		double last_yR = 0;
		double last_tR = 0;

		double xold = 0.0;//fsi.at(0).at(0);
		double yold = 0.0;//fsi.at(0).at(1);
		double told = 0.0;//fsi.at(0).at(2);
		uint i=0;

		for(i=0;i<fsi.size();i++){
			//half-foot-step-format v.3.0:
			// 1: x
			// 2: y
			// 3: theta
			// 4: ascii code for L or R
			// 5-11: absolute x,y,theta?
			double newX = fsi.at(i).at(5);
			double newY = fsi.at(i).at(6);
			double newT = fsi.at(i).at(7);
			char foot = fsi.at(i).at(3);
			FootMarker f(newX,newY,newT);


			if(foot == 'R'){
				f.set_color(1,0,0);
				f.drawLine(last_xR,last_yR);
				last_xR = newX;
				last_yR = newY;
			}else{
				f.set_color(0,1,0);
				f.drawLine(last_xR,last_yR);
				last_xR = newX;
				last_yR = newY;

			}
			f.publish();
			//ROS_INFO("published footstep [%d] at x=%f, y=%f, theta=%f", i,newX,newY,newT);

		}
	}
	uint stepCounter = 0;
	while (ros::ok())
	{
		//######################################################
		std::vector<double> q = generateWholeBodyMotionFromFootsteps(fsi, stepCounter++);
		if(q.size()>0){
			ROS_INFO("configuration vector: %d", q.size());
			//Replay trajectory
			tv->init(q);
			ros::Rate rq(500);
			while(tv->next()){
				ros::spinOnce();
				rq.sleep();
			}
		}
		r.sleep();

	}
}
