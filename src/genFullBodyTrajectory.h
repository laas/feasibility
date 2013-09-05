/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#ifndef genFullBodyTrajectory_H
#define genFullBodyTrajectory_H

#include <analyticalPG/newPGstepStudy.h>
#include <jrl/mal/matrixabstractlayer.hh>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>


class CgenFullBodyTrajectory
{
	public:

        CgenFullBodyTrajectory();
	
        ~CgenFullBodyTrajectory();

        void generateTrajectoryBAK(vector<vector<double> > & trajTimedRadQ, vector<vector<double> > & trajTimedZMP, vector<vector<double> > & trajTimedWaist, StepFeatures & stepF);
        void generateTrajectory(vector<vector<double> > & trajTimedRadQ, vector<vector<double> > & trajTimedZMP, vector<vector<double> > & trajTimedWaist, StepFeatures & stepF, int from = 0);
        void generateTrajectory(vector<vector<double> > & trajTimedRadQ, StepFeatures & stepF, int from = 0, int size = -1);

        /*private:
	int ComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX(,double) &Body_R,
						MAL_S3_VECTOR( ,double) &Body_P,
						MAL_S3_VECTOR( ,double) &Dt,
						MAL_S3x3_MATRIX( ,double) &Foot_R,
						MAL_S3_VECTOR( ,double) &Foot_P,
						MAL_VECTOR( ,double)&q);
        */

	void getWaistFootKinematics(const matrix4d & jointRootPosition,
				const matrix4d & jointEndPosition,
							  vectorN &q,
							  vector3d Dt);

};

#endif
