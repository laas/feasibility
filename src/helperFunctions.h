int getFirstIndex(vector<step> &stepVect, int nStep)                                                                                                      
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

std::vector<double> createArticularValuesVector(vector<vector<double> >& trajTimedRadQ, 
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


void halfFootStepToStep(std::vector< std::vector<double> > &fsi, std::vector<step> &vectStep){
	for(uint i=0;i<fsi.size();i++){
		//half-foot-step-format v.3.0:
		// 1: x
		// 2: y
		// 3: theta
		// 4: ascii code for L or R
		// 5-11: absolute x,y,theta?
		step t;
		t.x = fsi.at(i).at(5);
		t.y = fsi.at(i).at(6);
		t.theta = fsi.at(i).at(7);
		t.left_or_right = fsi.at(i).at(3);
		vectStep.push_back(t);
	}
}
//void manualCalibration()                                                                                                                                                     
//{
//  curDrift_.shift_x = -0.000; //m
//  curDrift_.shift_y = -0.000; //m
//
//  if(curTraj_.start_state!=tools::RUN)
//    {
//      if (useWaist_)
//        curDrift_.currPosWaist[0] =
//          curDrift_.currPosWaist[1] =
//          curDrift_.currPosWaist[2] = 0.0;
//      else
//        {
//          curDrift_.currPosFoot[0]  =
//            curDrift_.currPosFoot[2]  = 0.0;
//          curDrift_.currPosFoot[1] = 0.095;
//        }
//    }
//}
void createFeatures( step& s0, step& s1 )
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
void recomputeZMP(vector<step>& vectStep, char foot='L', 
                                 double x_init=0.0, double y_init=0.19, double t_init=0.0)
{
	int nb=3;
	int nbModif = 0;
	if(vectStep.at(0).stepFeaturesUP.size == 0){
		step initialPosition; 
		initialPosition.x = x_init; 
		initialPosition.y = y_init; 
		initialPosition.theta = t_init; 
		initialPosition.left_or_right=foot;

		createFeatures( initialPosition, vectStep.at(0));
		nbModif++;
		if(nbModif==nb){ 
			return;
		}
	}
    
	for(unsigned int i=1; i<vectStep.size();i++){
		if(vectStep.at(i).stepFeaturesUP.size == 0 || 
			vectStep.at(i).stepFeaturesDOWN.size == 0){
			createFeatures( vectStep.at(i-1), vectStep.at(i));
			nbModif++;
			if(nbModif==nb){ 
				return;
			}
		}
	}

}


