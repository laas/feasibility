#pragma once
//refactoring of Nicolas Perrins work
//std::vector<double> q = generateWholeBodyMotionFromFootsteps(fsi, stepCounter++);
//
#include <analyticalPG/newPGstepStudy.h>
#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl/dynamics/dynamicsfactory.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>
#include "genFullBodyTrajectory.h"
#include "util/util.h"
#include "environment/environment.h"

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

struct PQPobject {

  PQP_Model* model;
  PQP_REAL T[3], R[3][3];
  int id;

};
class MotionGenerator{
private: 
	CnewPGstepStudy *NPSS;
	CgenFullBodyTrajectory *CGFBT;

	int nbObs;
	vector<PQPobject> mp_Obstacles;
	std::vector<CjrlJoint *> mp_aVecOfJoints;
	std::vector<PQPobject> mp_Objects;
	Chrp2OptHumanoidDynamicRobot * mp_aHDR;
  // Path to robot specific files: geometric description, and semantic.
  static const std::string mp_robot_path;

  uint last_step_smoothed_;

public:

  MotionGenerator(std::vector<ros::RVIZVisualMarker*> objects);
	std::vector<double> generateWholeBodyMotionFromAbsoluteFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed,
			double sf_x, double sf_y, double sf_t, char sf_f);

	std::vector<double> generateWholeBodyMotionFromAbsoluteFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed);

	std::vector<double> generateWholeBodyMotionFromFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed);

	std::vector<double> generateWholeBodyMotionFromStepVector(
				std::vector<step> &vectStep, int lastStepSmoothed, 
				double sf_x, double sf_y, double sf_t, char sf_f);
  bool
  isInCollision(std::vector<std::vector<double> > &fsi);
private:
  std::vector<double> generateWholeBodyMotionFromStepVector(
      std::vector<step> &vectStep, int lastStepSmoothed);

  StepFeatures computeFeaturesWithoutSmoothing(std::vector<step> &vectStep);
  StepFeatures computeFeaturesWithSmoothing(vector<step>& stepsVect, int startFrom, int numberOfSteps);
  double findMultiple(double x, double mul);

  void convertAbsoluteHalfFootStepToStepVector(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep);
  void convertHalfFootStepToStepVector(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep);

  int getFirstIndex(vector<step> &stepVect, int nStep);
  std::vector<double> createArticularValuesVector(vector<vector<double> >& trajTimedRadQ, 
                                                  StepFeatures& stepF, int time_start, int start, int nbPosToSend);

  void init_checkCollisionsPQP(std::string path, int ratio=110);
  int checkCollisions(vector<vector<double> >& trajTimedRadQ, StepFeatures& stepF, int start, int offset);
  void createFeatures( step& s0, step& s1 );
  void recomputeZMP(vector<step>& vectStep, char, double, double, double);

};
