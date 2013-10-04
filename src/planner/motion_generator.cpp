#include "planner/motion_generator.h"
#define DEBUG(x) x
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

			int res = -1;
			res = this->checkCollisions(trajTimedRadQ, newStep, 0 ,3);
			if( res==0 ){
				maxOne = currentAttempt;
			}else{
			  minZero = currentAttempt;
			}
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
void MotionGenerator::init_checkCollisionsPQP(std::string path, int ratio)
{
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  mp_aHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);

  ostringstream mainWrl;
  ostringstream jointRank;
  ostringstream specificities;
  mainWrl << path << "/HRP2JRLmainSmallOld.wrl";
  jointRank << path << "/HRP2LinkJointRankSmallOld.xml";
  specificities << path << "/HRP2SpecificitiesSmallOld.xml";

  filebuf fb;
  fb.open ( strdup(mainWrl.str().c_str()), ios::in);
  if(!fb.is_open()){
    printf("Failed to read file '%s'\n",mainWrl.str().c_str());
    exit(1);
  }
  fb.close();
  fb.open ( strdup(jointRank.str().c_str()), ios::in);
  if(!fb.is_open()){
    printf("Failed to read file '%s'\n",jointRank.str().c_str());
    exit(1);
  }
  fb.close();
  fb.open ( strdup(specificities.str().c_str()), ios::in);
  if(!fb.is_open()){
    printf("Failed to read file '%s'\n",specificities.str().c_str());
    exit(1);
  }
  fb.close();

  string RobotFileName = mainWrl.str();
  string LinkJointRank_ = jointRank.str();
  string SpecificitiesFileName_ = specificities.str();

  dynamicsJRLJapan::parseOpenHRPVRMLFile(*mp_aHDR,
					 RobotFileName,
					 LinkJointRank_,
					 SpecificitiesFileName_);

  mp_aVecOfJoints = mp_aHDR->jointVector(); //joint 0: the BODY. Then, 6 joints in the right leg, and 6 in the left leg.
  string margin;

  switch(ratio)
    {
    case 100: margin = "100"; break;
    case 110: margin = "110"; break;
    case 120: margin = "120"; break;
    default:  margin = "120"; break;
    }

  mp_Objects.clear();
  for(int i=0;i<4;i++)
    {
      ostringstream ostrstrRight (ostringstream::out);
      ostrstrRight << path << "/tris/rleg" << i+2 << "."<<margin<<".tris";
      FILE *fp = fopen(ostrstrRight.str().c_str(),"r");
      if(fp == NULL) { fprintf(stderr,"Couldn't open %s\n",ostrstrRight.str().c_str()); exit(1); }
      PQPobject PQPobj;
      PQPobj.id = i + 3;
      PQP_Model *model = new PQP_Model;
      int ntris;
      if (fscanf(fp,"%d",&ntris)<0) 
	{throw("Problem while reading file - tag 1"); }
      model->BeginModel();
      for (int i = 0; i < ntris; i++)
        {
	  double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
	  if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
		     &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z)<0)
	    { throw("Problem while reading file - tag 2"); }
	  PQP_REAL p1[3],p2[3],p3[3];
	  p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
	  p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
	  p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
	  model->AddTri(p1,p2,p3,i);
        }
      model->EndModel();
      PQPobj.model = model;
      mp_Objects.push_back(PQPobj);
    }

  for(int i=0;i<4;i++)
    {
      ostringstream ostrstrLeft (ostringstream::out);
      ostrstrLeft << path << "/tris/lleg" << i+2 << "."<<margin<<".tris";
      FILE *fp = fopen(ostrstrLeft.str().c_str(),"r");
      if(fp == NULL) { fprintf(stderr,"Couldn't open %s\n",ostrstrLeft.str().c_str()); exit(1); }
      PQPobject PQPobj;
      PQPobj.id = i + 9;
      PQP_Model *model = new PQP_Model;
      int ntris;
      if (fscanf(fp,"%d",&ntris)<0)
	{ throw("Problem while reading file - tag 3");} 
      model->BeginModel();
      for (int i = 0; i < ntris; i++)
        {
	  double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
	  if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
		     &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z)<0)
	    {throw("Problem while reading file - tag 4");}
	  PQP_REAL p1[3],p2[3],p3[3];
	  p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
	  p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
	  p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
	  model->AddTri(p1,p2,p3,i);
        }
      model->EndModel();
      PQPobj.model = model;
      mp_Objects.push_back(PQPobj);
    }

}

int MotionGenerator::checkCollisions(vector<vector<double> >& trajTimedRadQ, StepFeatures& stepF, int start, int offset)
{

  MAL_VECTOR_DIM(totall,double,36);
  CjrlJoint *aJoint;
  PQP_CollideResult cres;

  int NB_TEST = 0;
  for(unsigned int count = start; count < trajTimedRadQ.size() ; count+=offset)
	{

		double dtmp1 = min(min(min(min(min(min(min(min(min(min(min(trajTimedRadQ[count][1]*180/PI+30,30-trajTimedRadQ[count][1]*180/PI),trajTimedRadQ[count][2]*180/PI+30),20-trajTimedRadQ[count][2]*180/PI),trajTimedRadQ[count][3]*180/PI+120),40-trajTimedRadQ[count][3]*180/PI),trajTimedRadQ[count][4]*180/PI-2),150-trajTimedRadQ[count][4]*180/PI),trajTimedRadQ[count][5]*180/PI+75),40-trajTimedRadQ[count][5]*180/PI),trajTimedRadQ[count][6]*180/PI+20),30-trajTimedRadQ[count][6]*180/PI);
		double dtmp2 = min(min(min(min(min(min(min(min(min(min(min(trajTimedRadQ[count][7]*180/PI+30,30-trajTimedRadQ[count][7]*180/PI),trajTimedRadQ[count][8]*180/PI+20),30-trajTimedRadQ[count][8]*180/PI),trajTimedRadQ[count][9]*180/PI+120),40-trajTimedRadQ[count][9]*180/PI),trajTimedRadQ[count][10]*180/PI-2),150-trajTimedRadQ[count][10]*180/PI),trajTimedRadQ[count][11]*180/PI+75),40-trajTimedRadQ[count][11]*180/PI),trajTimedRadQ[count][12]*180/PI+30),20-trajTimedRadQ[count][12]*180/PI);

		double dtmp = min(dtmp1, dtmp2);

		if( dtmp < 0.001)
		{
			DEBUG( printf("Collision 3 after : %d tests.\n",NB_TEST);)
			return 3;
		}

		//FIXME compute fast inverse kinematic
		int index = stepF.size - trajTimedRadQ.size() + count;

		//freeflyer
		totall[0] = stepF.comTrajX[index];
		totall[1] = stepF.comTrajY[index] - 0.095;
		totall[2] = 0.65;
		totall[3] = totall[4] = 0.0;
		totall[5] = stepF.waistOrient[index]*PI/180.0;

		for(unsigned int i=6;i<6+12;i++)
		{
			totall[i]=trajTimedRadQ[count][i+1-6]; //the legs: 6 for the right leg (starting from the hip yaw), 6 for the left leg
		}
		for(unsigned int i=6+12;i<6+30;i++)
		{
			totall[i]=0; //upper body =0
		}

		mp_aHDR->currentConfiguration(totall);
		mp_aHDR->computeForwardKinematics();

		for(unsigned int i=0; i < mp_Objects.size(); i++)
		{

			matrix4d aCurrentM;
			matrix4d initialM;

			MAL_S3x3_MATRIX(aCurrentRot,double);
			MAL_S3x3_MATRIX(aInitRotTranspose,double);
			MAL_S3x3_MATRIX(aResultRot,double);

			aJoint = mp_aVecOfJoints[mp_Objects[i].id];
			initialM = aJoint->initialPosition();
			aCurrentM = aJoint->currentTransformation();

			for(unsigned int li=0;li<3;li++)
			{
				for(unsigned int lj=0;lj<3;lj++)
				{
					MAL_S3x3_MATRIX_ACCESS_I_J(aCurrentRot,li,lj) = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,li,lj);
					MAL_S3x3_MATRIX_ACCESS_I_J(aInitRotTranspose,li,lj) = MAL_S4x4_MATRIX_ACCESS_I_J(initialM,li,lj); //transposed
				}
			}

			aResultRot =  aCurrentRot * aInitRotTranspose;

			for(unsigned int u1=0; u1 < 3; u1++)
			{
				for(unsigned int u2=0; u2 < 3; u2++)
				{
					mp_Objects[i].R[u1][u2] = MAL_S4x4_MATRIX_ACCESS_I_J(aResultRot,u1,u2);
				}
			}

			mp_Objects[i].T[0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
			mp_Objects[i].T[1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);
			mp_Objects[i].T[2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,3);
		}

		for(unsigned int i=0; i < 4; i++)
		{
			for(unsigned int j=4; j < 8; j++)
			{
				//NB_TEST++;
				PQP_Collide(&cres, mp_Objects[i].R, mp_Objects[i].T, mp_Objects[i].model, mp_Objects[j].R, mp_Objects[j].T, mp_Objects[j].model, PQP_FIRST_CONTACT);
				if(cres.NumPairs()>0){
					DEBUG( printf("Collision between %d and %d at index %d.\n",i,j, count);)
					DEBUG( printf("Collision 1 after : %d tests.\n",NB_TEST);)
					return 1;
				}
			}
		}

		for(unsigned int i=0; i < 8; i++)
		{
			for(unsigned int j=0; j < mp_Obstacles.size(); j++)
			{
					NB_TEST++;
					PQP_Collide(&cres, mp_Objects[i].R, mp_Objects[i].T, mp_Objects[i].model, mp_Obstacles[j].R, mp_Obstacles[j].T, mp_Obstacles[j].model, PQP_FIRST_CONTACT);
					if(cres.NumPairs()>0) {
						DEBUG( printf("Collision 2 after : %d tests.\n",NB_TEST); )
						DEBUG( printf("Collision between %d and %d at index %d.\n",i,j, count);)
						return 2;
					}
			}
		}


	}
  DEBUG( printf("No collision after : %d tests.\n",NB_TEST); )
  return 0;
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
