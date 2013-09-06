#pragma once
//refactoring of Nicolas Perrins work
//std::vector<double> q = generateWholeBodyMotionFromFootsteps(fsi, stepCounter++);
//
//
#include <analyticalPG/newPGstepStudy.h>                                                                                                                                   
#include "genFullBodyTrajectory.h"
#include "util/util.h"

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
class MotionGenerator{

public:
	CnewPGstepStudy *NPSS;
	CgenFullBodyTrajectory *CGFBT;

	MotionGenerator(){
		NPSS = new CnewPGstepStudy(STEP_LENGTH); 
		CGFBT = new CgenFullBodyTrajectory();
	}
	std::vector<double> generateWholeBodyMotionFromAbsoluteFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed);
	std::vector<double> generateWholeBodyMotionFromFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed);
private:
	std::vector<double> generateWholeBodyMotionFromStepVector(
				std::vector<step> &vectStep, int lastStepSmoothed);

	void convertAbsoluteHalfFootStepToStepVector(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep);
	void convertHalfFootStepToStepVector(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep);

	int getFirstIndex(vector<step> &stepVect, int nStep);
	std::vector<double> createArticularValuesVector(vector<vector<double> >& trajTimedRadQ, 
                                                           StepFeatures& stepF, int time_start, int start, int nbPosToSend);

	void createFeatures( step& s0, step& s1 );
	void recomputeZMP(vector<step>& vectStep, char ,double, double, double);

};

