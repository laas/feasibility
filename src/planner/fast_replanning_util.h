void fastReplanning::createFeatures( step& s0, step& s1 )
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
void fastReplanning::computeZMP(vector<step>& vectStep, char foot, double x_init, double y_init, double t_init)
{
  step initialPosition; 
  initialPosition.x = x_init; 
  initialPosition.y = y_init; 
  initialPosition.theta = t_init; 
  initialPosition.left_or_right=foot;

  createFeatures( initialPosition, vectStep.at(0));

  for(unsigned int i=1;i< vectStep.size();i++){
    createFeatures( vectStep.at(i-1), vectStep.at(i));
  }
}
int fastReplanning::recomputeZMP(vector<step>& vectStep, int nb, char foot, 
				 double x_init, double y_init, double t_init)
{
  if(nb <= 0) nb = vectStep.size();

  int nbModif = 0;
  if(vectStep.at(0).stepFeaturesUP.size == 0){
    step initialPosition; 
    initialPosition.x = x_init; 
    initialPosition.y = y_init; 
    initialPosition.theta = t_init; 
    initialPosition.left_or_right=foot;

    createFeatures( initialPosition, vectStep.at(0));
    
    nbModif++;
    if(nbModif==nb)
      return nbModif;
  }

  for(unsigned int i=1; i<vectStep.size();i++){

    if(vectStep.at(i).stepFeaturesUP.size == 0 || 
       vectStep.at(i).stepFeaturesDOWN.size == 0){

      createFeatures( vectStep.at(i-1), vectStep.at(i));
      
      /*
      printf("createFeatures for step %d : %d/%d.\n",i,
	     vectStep.at(i).stepFeaturesUP.size,
	     vectStep.at(i).stepFeaturesDOWN.size);
      */
      nbModif++;
      if(nbModif==nb)
	return nbModif;
    }
  }
  return nbModif;
}

