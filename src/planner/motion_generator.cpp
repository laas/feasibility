#include "planner/motion_generator.h"
int MotionGenerator::getFirstIndex(vector<step> &stepVect, int nStep)                                                                                                      
{
  if(nStep==0)
    return 0;

  int index = 0;
  for(int i=0; i<nStep; i++){
    index += stepVect.at(i).stepFeaturesUP.size +
      stepVect.at(i).stepFeaturesDOWN.size +
      (int)(200.0*(stepVect.at(i).slideUP +
                   stepVect.at(i).slideDOWN) + 0.0001);
  }
  return index + 1;
}

std::vector<double> MotionGenerator::createArticularValuesVector(vector<vector<double> >& trajTimedRadQ, 
                                                           StepFeatures& stepF, int time_start, int start, int nbPosToSend)
{

  if( start + nbPosToSend > (int)trajTimedRadQ.size() )
    nbPosToSend = trajTimedRadQ.size() - start;

  if(start > (int)trajTimedRadQ.size())
    nbPosToSend = 0;

  vector<double> vect(nbPosToSend*17 + 4);


  vect[0] = 42;                      //ID TODO
  vect[1] = stepF.size*0.005;                   //End of trajectory
  vect[2] = time_start*0.005;                   //Time of the begginnig of the modification
  vect[3] = (time_start + nbPosToSend)*0.005;   //Time of the end of the modification

  int offset = 4;
  for(int i=0;i<nbPosToSend;i++){
    for(int j=0;j<12;j++){
      vect[offset + i*17 + j] = trajTimedRadQ[i+start][j+1];
    }
    vect[offset + i*17 + 12 + 0] = stepF.comTrajX[i+time_start];
    vect[offset + i*17 + 12 + 1] = stepF.comTrajY[i+time_start];
    vect[offset + i*17 + 12 + 2] = toRad(stepF.waistOrient[i+time_start]);
    vect[offset + i*17 + 12 + 3] = stepF.zmpTrajX[i+time_start];
    vect[offset + i*17 + 12 + 4] = stepF.zmpTrajY[i+time_start];
  }
  return vect;
}


void MotionGenerator::convertAbsoluteHalfFootStepToStepVector(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep){
	for(uint i=0;i<fsi.size();i++){
		step t;
		t.x = fsi.at(i).at(0);
		t.y = fsi.at(i).at(1);
		t.theta = fsi.at(i).at(2);
		t.left_or_right = fsi.at(i).at(3);

		if(t.left_or_right=='R'){
			t.y = -t.y;
			t.theta = -t.theta;
		}
		//double halfYLengthHalfSitting=0.095; 
		double halfYLengthHalfSitting=-0.000; 

		if (t.left_or_right=='L'){
			halfYLengthHalfSitting = -halfYLengthHalfSitting;
		}
		double theta = t.theta;

		double ct = cos(theta); double st = sin(theta);
		t.x = t.x - (st) * halfYLengthHalfSitting;
		t.y = t.y + (ct + 1) * halfYLengthHalfSitting;

		//ROS_INFO("X: %f, Y: %f, T: %f, F: %f", t.x, t.y, toDeg(t.theta), t.left_or_right);


		vectStep.push_back(t);
	}
}
void MotionGenerator::convertHalfFootStepToStepVector(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep){

	for(uint i=0;i<fsi.size();i++){
		//half-foot-step-format v.3.0:
		// 1: x
		// 2: y
		// 3: theta
		// 4: ascii code for L or R
		// 5-11: absolute x,y,theta?
		step t;
		t.x = fsi.at(i).at(0);
		t.y = fsi.at(i).at(1);
		t.theta = fsi.at(i).at(2);
		t.left_or_right = fsi.at(i).at(3);

		double halfYLengthHalfSitting=0.095; 
		if (t.left_or_right=='L'){
			halfYLengthHalfSitting = -halfYLengthHalfSitting;
		}

		double theta = t.theta;
		double ct = cos(theta); double st = sin(theta);

		t.x = t.x - (st) * halfYLengthHalfSitting;
		t.y = t.y + (ct + 1) * halfYLengthHalfSitting;

		vectStep.push_back(t);
	}
}
void MotionGenerator::createFeatures( step& s0, step& s1 )
{
	double zc = 0.814;
	double t1 = STEP_LENGTH/2.0 - 0.05;
	double t2 = STEP_LENGTH/2.0 + 0.05;
	double t3 = STEP_LENGTH;
	double incrTime = 0.005;
	double g = 9.81;
	char foot = s1.left_or_right;

	vector<double> tmpPart1;
	tmpPart1.resize(8,0.0);

	vector<double> tmpPart2;
	tmpPart2.resize(5,0.0);

	tmpPart1[0] = (s0.x/2)*cos(-s0.theta) - (s0.y/2)*sin(-s0.theta);
	tmpPart1[1] = (s0.x/2)*sin(-s0.theta) + (s0.y/2)*cos(-s0.theta);
	tmpPart1[2] = 0;
	tmpPart1[3] = -tmpPart1[0];
	tmpPart1[4] = -tmpPart1[1];
	tmpPart1[5] = -s0.theta * 180.0/PI;
	tmpPart1[6] = 0.24;
	tmpPart1[7] = 0.25;

	tmpPart2[0] = 0.24;
	tmpPart2[1] = 0.25;
	tmpPart2[2] = s1.x;
	tmpPart2[3] = s1.y;
	tmpPart2[4] = s1.theta * 180.0/PI;

	s1.slideUP = 0.0;
	s1.slideDOWN = 0.0;

	NPSS->produceOneUPHalfStepFeatures(s1.stepFeaturesUP, 
				     incrTime, zc, g, t1, t2, t3, 
				     tmpPart1, foot);
	NPSS->produceOneDOWNHalfStepFeatures(s1.stepFeaturesDOWN, 
				       incrTime, zc, g, t1, t2, t3, 
				       tmpPart2, foot);
} 
// adding stepFeaturesUP/DOWN to footsteps
void MotionGenerator::recomputeZMP(vector<step>& vectStep, char foot='R', 
                                 double x_init=0.0, double y_init=0.19, double t_init=0.0)
{
	if(vectStep.at(0).stepFeaturesUP.size == 0){
		step initialPosition; 
		initialPosition.x = x_init; 
		initialPosition.y = y_init; 
		initialPosition.theta = t_init; 
		initialPosition.left_or_right=foot;

		createFeatures( initialPosition, vectStep.at(0));
	}
    
	for(unsigned int i=1; i<vectStep.size();i++){
		if(vectStep.at(i).stepFeaturesUP.size == 0 || 
		   vectStep.at(i).stepFeaturesDOWN.size == 0){
			createFeatures( vectStep.at(i-1), vectStep.at(i));
		}
	}

}


//###############################################################################
//###############################################################################
std::vector<double> MotionGenerator::generateWholeBodyMotionFromAbsoluteFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed){
	std::vector<step> vectStep;
	vectStep.clear();
	convertAbsoluteHalfFootStepToStepVector(fsi, vectStep);
	return generateWholeBodyMotionFromStepVector( vectStep, lastStepSmoothed);
}
std::vector<double> MotionGenerator::generateWholeBodyMotionFromAbsoluteFootsteps(
			std::vector<std::vector<double> > &fsi, int lastStepSmoothed,
			double sf_x, double sf_y, double sf_t, char sf_f){
	std::vector<step> vectStep;
	vectStep.clear();
	convertAbsoluteHalfFootStepToStepVector(fsi, vectStep);
	return generateWholeBodyMotionFromStepVector( vectStep, lastStepSmoothed, sf_x, sf_y, sf_t, sf_f);
}
std::vector<double> MotionGenerator::generateWholeBodyMotionFromFootsteps(
		std::vector<std::vector<double> > &fsi, int lastStepSmoothed)
{
	std::vector<step> vectStep;
	vectStep.clear();
	convertHalfFootStepToStepVector(fsi, vectStep);
	return generateWholeBodyMotionFromStepVector(vectStep, lastStepSmoothed);
}


std::vector<double> MotionGenerator::generateWholeBodyMotionFromStepVector(
				std::vector<step> &vectStep, int lastStepSmoothed){

	return generateWholeBodyMotionFromStepVector( vectStep, lastStepSmoothed, 0,0.19,0,'R');
}
StepFeatures MotionGenerator::computeFeaturesWithoutSmoothing(
			std::vector<step> &vectStep){
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
	return stepF;
}
double MotionGenerator::findMultiple(double x, double mul)
{
	double temp = 0.0;
	if(x>=0.0){
		while(temp<x){
			temp += mul;
		}
	}else{
		while(temp>x){
			temp -= mul;
		}
	}
	return temp;
}
StepFeatures MotionGenerator::computeFeaturesWithSmoothing(vector<step>& stepsVect, int startFrom, int numberOfSteps){

	StepFeatures stepF1, stepUP, stepDOWN;
	vector<vector<double> > trajTimedRadQ;
	//FIXME: useful?
	//CC->updateObstacle(evalFunction);

	if(startFrom<0) startFrom=0;
	if(numberOfSteps==-1) numberOfSteps = stepsVect.size() - startFrom;

	double t1 = STEP_LENGTH/2.0 - 0.05*STEP_LENGTH/2.0; //0.95;
	double t2 = STEP_LENGTH/2.0 + 0.05*STEP_LENGTH/2.0; //1.05;
	double t3 = STEP_LENGTH;

	//FIXME : test multiple of 0.005
	double maxSlideUp = COEF_MAX_SLIDE_UP * (t1 + t3 - t2);
	double maxSlideDown =  COEF_MAX_SLIDE_DOWN * min(t3-t2, t1);

	double minZero = 0.0;
	double maxOne = 0.0;
	double currentAttempt = 0.0;
	double slideMin = 0.1;

	int NB_TEST = 0;

	unsigned int STEP_FEATURE_LENGTH_UP = stepsVect.at(0).stepFeaturesUP.size;
	unsigned int STEP_FEATURE_LENGTH_DOWN = stepsVect.at(0).stepFeaturesDOWN.size;
	//FIXME : collision test length ?
	int TEST_LENGTH = 3*200;

	//Create StepFeatures
	stepUP = stepsVect.at(0).stepFeaturesUP;
	stepF1 = stepUP;

	for(int i=0; i<startFrom; i++){
		if(i==0){
			if(stepsVect.at(i).stepFeaturesUP.size!=STEP_FEATURE_LENGTH_UP) 
			{
				printf("\n[%d] stepFeaturesDOWN is not good (%d!=%d).\n",i,
					 stepsVect.at(i).stepFeaturesUP.size,STEP_FEATURE_LENGTH_UP); 
				exit(1);
			}
			stepDOWN = stepsVect.at(i).stepFeaturesDOWN;
			NPSS->addStepFeaturesWithSlide(stepF1, stepDOWN, stepsVect.at(i).slideDOWN);
		}else{
			if(stepsVect.at(i).stepFeaturesUP.size!=STEP_FEATURE_LENGTH_UP) 
			  {
			    printf("\n[%d] stepFeaturesUP is not good (%d!=%d).\n",i,
				   stepsVect.at(i).stepFeaturesUP.size,STEP_FEATURE_LENGTH_UP); 
			    exit(1);
			  }
			if(stepsVect.at(i).stepFeaturesDOWN.size!=STEP_FEATURE_LENGTH_DOWN) 
			  {
			    printf("\n[%d] stepFeaturesDOWN is not good (%d!=%d).\n",i,
				   stepsVect.at(i).stepFeaturesDOWN.size,STEP_FEATURE_LENGTH_DOWN); 
			    exit(1);
			  }

			stepUP = stepsVect.at(i).stepFeaturesUP;
			stepDOWN = stepsVect.at(i).stepFeaturesDOWN;
			NPSS->addStepFeaturesWithSlide(stepF1, stepUP  , 
						       stepsVect.at(i).slideUP  );
			NPSS->addStepFeaturesWithSlide(stepF1, stepDOWN, 
						       stepsVect.at(i).slideDOWN);
		}
	}//for

	int laststep = min(startFrom+numberOfSteps,(int)stepsVect.size());
	for(int i=startFrom;i<laststep;i++){
		//Smooth stepUP
		if(i!=0){
			maxOne = findMultiple(maxSlideUp,0.005);

			stepUP = stepsVect.at(i).stepFeaturesUP;
			stepsVect.at(i).slideUP = -maxOne;

			NPSS->addStepFeaturesWithSlide(stepF1,stepUP,-maxOne);
		}
		//Smooth stepDown
		minZero = maxSlideDown;
		maxOne = 0.0;
		currentAttempt = maxSlideDown;

		while(minZero - maxOne > T_STOP_DICHO){
			StepFeatures newStep = stepF1;
			stepDOWN = stepsVect.at(i).stepFeaturesDOWN;

			//Test if stepFeature is correct
			if(stepDOWN.size!=STEP_FEATURE_LENGTH_DOWN){
				  printf("\n[%d] stepFeaturesDOWN is not good (%d!=%d).\n",i,
					 stepsVect.at(i).stepFeaturesDOWN.size,STEP_FEATURE_LENGTH_DOWN); 
				  exit(1);
			}

			double currentAttemptTemp = findMultiple(currentAttempt,0.005);
			NPSS->addStepFeaturesWithSlide(newStep,stepDOWN,-currentAttemptTemp);

			int firstIndex = newStep.size - TEST_LENGTH;
			if(firstIndex<0){
				firstIndex = 0;
			}
			CGFBT->generateTrajectory(trajTimedRadQ,newStep, firstIndex);

			NB_TEST++;

			//int res = -1;
			//res = CC->checkCollisions(trajTimedRadQ, trajTimedWaist, 0 ,10);

			//res = CC->checkCollisions(trajTimedRadQ, newStep, 0 ,3);
			//if( res==0 ){
			maxOne = currentAttempt;
			//}else{
			//  minZero = currentAttempt;
			//}
			currentAttempt = (maxOne+minZero)/2.0;
		}

		if(maxOne<slideMin){
			maxOne = slideMin;
		}else{
			maxOne = findMultiple(maxOne,0.005);
		}

		stepDOWN = stepsVect.at(i).stepFeaturesDOWN;
		stepsVect.at(i).slideDOWN = -maxOne;
		NPSS->addStepFeaturesWithSlide(stepF1,stepDOWN,-maxOne);
	}
	return stepF1;
}
std::vector<double> MotionGenerator::generateWholeBodyMotionFromStepVector(
				std::vector<step> &vectStep, int lastStepSmoothed, 
				double sf_x, double sf_y, double sf_t, char sf_f){
	//###############################################################################
	//###############################################################################
	//fsi to step format
	//###############################################################################
	//###############################################################################

	std::vector<double> q;
	if(lastStepSmoothed > vectStep.size()-1){
		ROS_INFO("last step: %d / %d -> exit", lastStepSmoothed, vectStep.size());
		return q;
	}

	//int lastStep = vectStep.size();
//###############################################################################
//###############################################################################
// NO smooth STEPS
//computeStepFeaturesWithoutSmoothing
//###############################################################################
//###############################################################################
	recomputeZMP(vectStep, sf_f, sf_x, sf_y, sf_t);

	bool smoothing=true;
	int size = -1;
	StepFeatures stepF;
	if(smoothing){
		stepF = computeFeaturesWithSmoothing(vectStep, lastStepSmoothed, 2);
		size = vectStep[lastStepSmoothed].stepFeaturesUP.size 
			   + vectStep[lastStepSmoothed].stepFeaturesDOWN.size + 
			 + (int) (200.0*( vectStep [lastStepSmoothed].slideUP
					+ vectStep [lastStepSmoothed].slideDOWN) 
					+ 0.001) 
			 + 1;
	}else{
		stepF = computeFeaturesWithoutSmoothing(vectStep);
		size = vectStep[lastStepSmoothed].stepFeaturesUP.size
			 + vectStep[lastStepSmoothed].stepFeaturesDOWN.size
			 + (int) (200.0*( vectStep [lastStepSmoothed-1].slideUP
					+ vectStep [lastStepSmoothed-1].slideDOWN) 
					+ 0.001) 
			 + 1;
	}

//###############################################################################
//###############################################################################
//Generate full body trajectory
//###############################################################################
//###############################################################################

	int firstIndex = getFirstIndex( vectStep, lastStepSmoothed);                                                                                                   
	ROS_INFO("stepLength %d, firstIndex %d, size %d", vectStep.size(), firstIndex, size);

	if(size>0){
		vector<vector<double> > trajTimedRadQ;
		CGFBT->generateTrajectory( trajTimedRadQ, stepF, firstIndex, size);
		q = createArticularValuesVector(trajTimedRadQ, stepF, firstIndex, 0, size);
	}

	return q;

}
