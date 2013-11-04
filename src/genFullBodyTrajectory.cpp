//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#include "genFullBodyTrajectory.h"

CgenFullBodyTrajectory::CgenFullBodyTrajectory() {
}
	
CgenFullBodyTrajectory::~CgenFullBodyTrajectory() {
}

void CgenFullBodyTrajectory::getWaistFootKinematics(const matrix4d & jointRootPosition,
						    const matrix4d & jointEndPosition,
						    vectorN &q,
						    vector3d Dt)
{
  double epsilon_=1.0e-6;
  // definition des variables relatif au design du robot
  double A = 0.3;//m_FemurLength;
  double B = 0.3;//m_TibiaLength;
  double C = 0.0;
  double c5 = 0.0;
  double q6a = 0.0;
  
  vector3d r;
  
  /* Build sub-matrices */
  matrix3d Foot_R,Body_R;
  vector3d Foot_P,Body_P;
  for(unsigned int i=0;i<3;i++)
    {
      for(unsigned int j=0;j<3;j++)
	{
	  MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j) = 
	    MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j);
	  MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j) = 
	    MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j);
	}
      Body_P(i) = MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3);
      Foot_P(i) = MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3);
    }

  matrix3d Foot_Rt;
  MAL_S3x3_TRANSPOSE_A_in_At(Foot_R,Foot_Rt);
  
  // Initialisation of q
  if (MAL_VECTOR_SIZE(q)!=6)
    MAL_VECTOR_RESIZE(q,6);

  for(unsigned int i=0;i<6;i++)
    q(i)=0.0;
  
  double OppSignOfDtY = Dt(1) < 0.0 ? 1.0 : -1.0; // if Dt(1)<0.0 then Opp=1.0 else Opp=-1.0
  
  vector3d d2,d3;
  d2 = Body_P + Body_R * Dt;
  d3 = d2 - Foot_P;
  
  double l0 = sqrt(d3(0)*d3(0)+d3(1)*d3(1)+d3(2)*d3(2) - 0.035*0.035);
  c5 = 0.5 * (l0*l0-A*A-B*B) / (A*B);
  if (c5 > 1.0-epsilon_)
    {
      q[3] = 0.0;
    }
  if (c5 < -1.0+epsilon_)
    {
      q[3] = M_PI;
    }
  if (c5 >= -1.0+epsilon_ && c5 <= 1.0-epsilon_) 
    {
      q[3] = acos(c5);
    }

  vector3d r3;
  r3 = Foot_Rt * d3;
  
  q6a = asin((A/l0)*sin(M_PI- q[3]));
  
  double l3 = sqrt(r3(1)*r3(1) + r3(2)*r3(2));
  double l4 = sqrt(l3*l3 - 0.035*0.035);
  
  double phi = atan2(r3(0), l4);
  q[4] = -phi - q6a;
  
  double psi1 = atan2(r3(1), r3(2)) * OppSignOfDtY;
  double psi2 = 0.5*M_PI - psi1;
  double psi3 = atan2(l4, 0.035);
  q[5] = (psi3 - psi2) * OppSignOfDtY;
  
  if (q[5] > 0.5*M_PI)
    {
      q[5] -= M_PI;
    }
  else if (q[5] < -0.5*M_PI)
    {
      q[5] += M_PI;
    }
  
  matrix3d R;
  matrix3d BRt;
  MAL_S3x3_TRANSPOSE_A_in_At(Body_R,BRt);
  
  matrix3d Rroll;
  double c = cos(q[5]);
  double s = sin(q[5]);
  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,0,0) = 1.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,0,1) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,0,2) = 0.0;

  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,1,0) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,1,1) = c;  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,1,2) = s;
  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,2,0) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,2,1) = -s;  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rroll,2,2) = c;
  
  matrix3d Rpitch;
  c = cos(q[4]+q[3]);
  s = sin(q[4]+q[3]);

  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,0,0) = c;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,0,1) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,0,2) = -s;

  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,1,0) = 0.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,1,1) = 1.0;  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,1,2) = 0.0;
  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,2,0) = s;
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,2,1) = 0.0;  
  MAL_S3x3_MATRIX_ACCESS_I_J(Rpitch,2,2) = c;
    
  R = BRt * Foot_R * Rroll * Rpitch;
  q[0] = atan2(-R(0,1),R(1,1));
  
  double cz = cos(q[0]);
  double sz = sin(q[0]);
  
  q[1] = atan2(R(2,1), -R(0,1)*sz+R(1,1)*cz);
  q[2] = atan2( -R(2,0), R(2,2));
}
/*
  int CgenFullBodyTrajectory::ComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX(,double) &Body_R,
  MAL_S3_VECTOR( ,double) &Body_P,
  MAL_S3_VECTOR( ,double) &Dt,
  MAL_S3x3_MATRIX( ,double) &Foot_R,
  MAL_S3_VECTOR( ,double) &Foot_P,
  MAL_VECTOR( ,double)&q)
  {

  double m_KneeAngleBound=0.0*M_PI/180.0;
  double m_KneeAngleBoundCos=cos(m_KneeAngleBound);
  double m_KneeAngleBound1=30.0*M_PI/180.0; //sets a minimum angle for the knee and protects for overstretch
  double m_KneeAngleBoundCos1=cos(m_KneeAngleBound1);  //used during inverse kin calculations
  
  double m_FemurLength=0.30;
  double m_TibiaLength=0.30;
  
  //m_KneeAngleBound=15.0*M_PI/180.0; //sets a minimum angle for the knee and protects for overstretch
  double m_KneeAngleBoundCos2= 1.336;

  double A=m_FemurLength,B=m_TibiaLength,C=0.0,c5=0.0,q6a=0.0;
  MAL_S3_VECTOR( r,double);
  MAL_S3x3_MATRIX( rT,double);
  MAL_S3x3_MATRIX( Foot_Rt,double);
  double NormofDt=0.0;

  // New part for the inverse kinematics specific to the HRP-2
  // robot. The computation of rx, ry and rz is different.

  // We compute the position of the body inside the reference
  // frame of the foot.
  MAL_S3_VECTOR( v,double);
  double theta=0.0, psi=0.0, Cp=0.0;
  float OppSignOfDtY = Dt(1) < 0.0 ? 1.0 : -1.0;


  Foot_Rt = MAL_S3x3_RET_TRANSPOSE(Foot_R);
  v = Body_P - Foot_P;
  v = MAL_S3x3_RET_A_by_B(Foot_Rt , v);
  //  cout << "v : "<< v <<endl;
  r(0) = v(0);
  NormofDt = sqrt(Dt(0)*Dt(0) + Dt(1)*Dt(1) + Dt(2)*Dt(2));
  //  cout << "Norm of Dt: " << NormofDt << endl;
  Cp = sqrt(v(1)*v(1)+v(2)*v(2) - NormofDt * NormofDt);
  psi = OppSignOfDtY * atan2(NormofDt,Cp);

  
  //  cout << "vz: " << v(2,0) << " vy :" << v(1,0) << endl;
  theta = atan2(v(2),v(1));
  
  r(1) = cos(psi+theta)*Cp;

  r(2) = sin(psi+theta)*Cp;

  //  r = rT * (Body_P +  Body_R * Dt - Foot_P);
  C = sqrt(r(0)*r(0)+
  r(1)*r(1)+
  r(2)*r(2));
  //C2 =sqrt(C1*C1-D*D);
  c5 = (C*C-A*A-B*B)/(2.0*A*B);

  if (c5>=m_KneeAngleBoundCos)
  {

  q(3)=m_KneeAngleBound;

  }
  else if (c5<=-1.0)
  {
  q(3)= M_PI;
  }
  else 
  {
  q(3)= acos(c5);
  }
  q6a = asin((A/C)*sin(M_PI- q(3)));


  float c,s,cz,sz;

  q(5) = atan2(r(1),r(2));
  if (q(5)>M_PI/2.0)
  {
  q(5) = q(5)-M_PI;
  }
  else if (q(5)<-M_PI/2.0)
  {
  q(5)+= M_PI;
  }

  q(4) = -atan2(r(0), (r(2)<0? -1.0:1.0)*sqrt(r(1)*r(1)+r(2)*r(2) )) - q6a;

  MAL_S3x3_MATRIX(R,double);
  MAL_S3x3_MATRIX(BRt,double);

  BRt = MAL_S3x3_RET_TRANSPOSE(Body_R);
  
  MAL_S3x3_MATRIX( Rroll,double);
  c = cos(-q(5));
  s = sin(-q(5));
  
  Rroll(0,0) = 1.0;   Rroll(0,1) = 0.0;   Rroll(0,2) = 0.0; 
  Rroll(1,0) = 0.0;   Rroll(1,1) = c;   Rroll(1,2) = -s; 
  Rroll(2,0) = 0.0;   Rroll(2,1) = s;   Rroll(2,2) = c; 
  
  
  MAL_S3x3_MATRIX( Rpitch,double);
  c = cos(-q(4)-q(3));
  s = sin(-q(4)-q(3));
  
  Rpitch(0,0) = c;     Rpitch(0,1) = 0;   Rpitch(0,2) = s; 
  Rpitch(1,0) = 0.0;   Rpitch(1,1) = 1;   Rpitch(1,2) = 0; 
  Rpitch(2,0) = -s;    Rpitch(2,1) = 0;   Rpitch(2,2) = c; 
  
  //  cout << " BRt"  << BRt << endl;
  R = MAL_S3x3_RET_A_by_B(BRt, Foot_R );
  MAL_S3x3_MATRIX(Rtmp,double);
  Rtmp = MAL_S3x3_RET_A_by_B(Rroll,Rpitch);
  R = MAL_S3x3_RET_A_by_B(R,Rtmp);,0,5

  q(0) = atan2(-R(0,1),R(1,1));
  
  cz = cos(q(0)); sz = sin(q(0));
  q(1) = atan2(R(2,1), -R(0,1)*sz + R(1,1) *cz);
  q(2) = atan2( -R(2,0), R(2,2));
  
  //  exit(0);
  return 0;
  }
*/

void CgenFullBodyTrajectory::generateTrajectoryBAK(vector<vector<double> > & trajTimedRadQ, vector<vector<double> >& trajTimedZMP, vector<vector<double> >& trajTimedWaist, StepFeatures & stepF)
{
  trajTimedRadQ.clear();
  trajTimedZMP.clear();

  MAL_VECTOR(,double) jointsRadValues;

  trajTimedRadQ.resize(stepF.size);
  trajTimedZMP.resize(stepF.size);
  trajTimedWaist.resize(stepF.size);

  for(unsigned int count = 0; count < stepF.size ; count++)
    {

      MAL_S3x3_MATRIX(Body_R,double);
      MAL_S3_VECTOR(Body_P,double);
      MAL_S3_VECTOR(Dt,double);
      MAL_S3x3_MATRIX(Foot_R,double);
      MAL_S3_VECTOR(Foot_P,double);
      //MAL_VECTOR_DIM(q,double,6);
      //MAL_VECTOR_DIM(q_mem,double,6);
      vectorN q;
      MAL_VECTOR_RESIZE(q,6);
      vectorN q_mem;
      MAL_VECTOR_RESIZE(q_mem,6);
      matrix4d jointRootPosition;
      matrix4d jointEndPosition;
      vector3d Dtbis;

      //Initialization of these matrices and vectors for the right leg:

      // body rotation
      MAL_S3x3_MATRIX_CLEAR(Body_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = 1.0;
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = 1.0;
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 0) = sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 1) = -sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;

      // body translation : vector (0,0,0)
      MAL_S3_VECTOR_FILL(Body_P,0.0);

      //we describe the horizontal configuration of the feet relative to the waist (considered as the root position and orientation), with 6 parameters: (Left) x,y,t1, (Right) x',y',t2.
      //VERY IMPORTANT REMARK: the positions have been calculated according to the initial orientation of the waist: zero. Therefore a rotation has to be performed in order to deal with that.

      vector<double> feetVector;
      feetVector.resize(6);

      feetVector[0] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) - (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
      feetVector[1] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) + (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
      feetVector[2] = stepF.leftfootOrient[count] - stepF.waistOrient[count];
      feetVector[3] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) - (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
      feetVector[4] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) + (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
      feetVector[5] = stepF.rightfootOrient[count] - stepF.waistOrient[count];

      //we convert from feet to ankles:
      //feetVector[3]+=-cos(feetVector[5]*PI/180-PI/2)*0.035;
      //feetVector[4]+=-sin(feetVector[5]*PI/180-PI/2)*0.035;
      //feetVector[0]+=-cos(feetVector[2]*PI/180+PI/2)*0.035;
      //feetVector[1]+=-sin(feetVector[2]*PI/180+PI/2)*0.035;

      double foot1X1=feetVector[0];
      double foot1Y1=feetVector[1];
      double foot1Theta1=feetVector[2]*PI/180;

      double foot2X1=feetVector[3];
      double foot2Y1=feetVector[4];
      double foot2Theta1=feetVector[5]*PI/180;

      //remark: the InverseKinematics takes into account the position of
      //the ankle so the z-limit for HRP2 is 60cm whereas 10.5cm have to be added to
      //get the actual distance between the ground and the waist root (i
      //suppose here that the HRP2's feet stay flat on the ground)
      //other remark: the ankle "zero" y-translation is 6cm.

      //Now we define the joint values for the right leg:

      // Dt (leg)
      MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
      MAL_S3_VECTOR_ACCESS(Dt,1) = -0.06;
      MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

      // ankle rotation
      MAL_S3x3_MATRIX_CLEAR(Foot_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

      // ankle translation
      MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot2X1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot2Y1;
      // - 0.105
      MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -(stepF.zc - 0.274) + stepF.rightfootHeight[count];

      //ComputeInverseKinematics2ForLegs(Body_R, Body_P,Dt,Foot_R, Foot_P,q);

      //*************************************************************************
      Dtbis(0) = Dt[0]; Dtbis(1) = Dt[1]; Dtbis(2) = Dt[2];
      for(unsigned int i=0;i<3;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	    }
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3) = Body_P(i);
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3) = Foot_P(i);
	}
      getWaistFootKinematics(jointRootPosition,jointEndPosition,q,Dtbis);
      //*************************************************************************


      for(int k=0;k<6;k++)
	{
	  q_mem[k]=q[k];
	}

      //values for the left leg:

      MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
      MAL_S3_VECTOR_ACCESS(Dt,1) = +0.06;
      MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

      // ankle rotation
      MAL_S3x3_MATRIX_CLEAR(Foot_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

      // ankle translation
      MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot1X1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot1Y1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -(stepF.zc - 0.274) + stepF.leftfootHeight[count];

      //ComputeInverseKinematics2ForLegs(Body_R, Body_P, Dt, Foot_R, Foot_P, q);
      //*************************************************************************
      Dtbis(0) = Dt[0]; Dtbis(1) = Dt[1]; Dtbis(2) = Dt[2];
      for(unsigned int i=0;i<3;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	    }
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3) = Body_P(i);
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3) = Foot_P(i);
	}
      getWaistFootKinematics(jointRootPosition,jointEndPosition,q,Dtbis);
      //*************************************************************************

      jointsRadValues.clear();
      jointsRadValues.resize(42);

      jointsRadValues[0]= q_mem[0];
      jointsRadValues[1]= q_mem[1];
      jointsRadValues[2]= q_mem[2];
      jointsRadValues[3]= q_mem[3];
      jointsRadValues[4]= q_mem[4];
      jointsRadValues[5]= q_mem[5];

      jointsRadValues[6]= q[0];
      jointsRadValues[7]= q[1];
      jointsRadValues[8]= q[2];
      jointsRadValues[9]= q[3];
      jointsRadValues[10]= q[4];
      jointsRadValues[11]= q[5];

      jointsRadValues[12]=0.0;
      jointsRadValues[13]=0.0;

      jointsRadValues[14]=0.0;
      jointsRadValues[15]=0.0;

      jointsRadValues[16]=0.2583087;
      jointsRadValues[17]=-0.174533;
      jointsRadValues[18]=0.000000;
      jointsRadValues[19]=-0.523599;
      jointsRadValues[20]=0.000000;
      jointsRadValues[21]=0.000000;
      jointsRadValues[22]=0.174533;
      jointsRadValues[23]=0.174533;

      jointsRadValues[24]=0.2583087;
      jointsRadValues[25]=0.174533;
      jointsRadValues[26]=0.000000;
      jointsRadValues[27]=-0.523599;
      jointsRadValues[28]=0.000000;
      jointsRadValues[29]=0.000000;
      jointsRadValues[30]=0.174533;
      jointsRadValues[31]=0.174533;

      jointsRadValues[32]=0.0;
      jointsRadValues[33]=0.0;
      jointsRadValues[34]=0.0;
      jointsRadValues[35]=0.0;
      jointsRadValues[36]=0.0;
      jointsRadValues[37]=0.0;
      jointsRadValues[38]=0.0;
      jointsRadValues[39]=0.0;
      jointsRadValues[40]=0.0;
      jointsRadValues[41]=0.0;

      //MAL_VECTOR(,double) jointsRadValuesSmall = jointsRadValues;
      //jointsRadValuesSmall.resize(30);
      //for(unsigned int u = 12; u < 30; u++) jointsRadValuesSmall[u]=0;


      vector<double> tmpvectCOM(43);
      vector<double> tmpvectZMP(4);
      vector<double> tmpvectWaist(5);
      tmpvectCOM[0] = (stepF.incrTime * count);
      for(int j = 0; j < 42; j++)
	{
	  tmpvectCOM[j+1] = jointsRadValues[j];
	}
      trajTimedRadQ[count] = tmpvectCOM;

      tmpvectZMP[0] = (stepF.incrTime * count);
      tmpvectZMP[1] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180)
	-(stepF.zmpTrajY[count]-stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180) ;
      tmpvectZMP[2] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180)
	+(stepF.zmpTrajY[count]-stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180) ;
      tmpvectZMP[3] = ( -(stepF.zc - 0.274)-0.105 );
      trajTimedZMP[count] = tmpvectZMP;

      tmpvectWaist[0] = (stepF.incrTime * count);
      tmpvectWaist[1] = ( stepF.comTrajX[count]);
      tmpvectWaist[2] = ( stepF.comTrajY[count]);
      tmpvectWaist[3] = ( stepF.zc - 0.274 + 0.105);
      tmpvectWaist[4] = ( stepF.waistOrient[count]*PI/180);

      trajTimedWaist[count] = tmpvectWaist;

    }

}

void CgenFullBodyTrajectory::generateTrajectory(vector<vector<double> > & trajTimedRadQ, 
						vector<vector<double> > & trajTimedZMP, 
						vector<vector<double> > & trajTimedWaist, 
						StepFeatures & stepF, int from)
{
  trajTimedRadQ.clear();
  trajTimedZMP.clear();

  MAL_VECTOR(,double) jointsRadValues;

  trajTimedRadQ.resize(stepF.size - from);
  trajTimedZMP.resize(stepF.size - from);
  trajTimedWaist.resize(stepF.size - from);

  //cout << "Size : " << stepF.size - from << endl;

  for(unsigned int count = from; count < stepF.size ; count++)
    {

      MAL_S3x3_MATRIX(Body_R,double);
      MAL_S3_VECTOR(Body_P,double);
      MAL_S3_VECTOR(Dt,double);
      MAL_S3x3_MATRIX(Foot_R,double);
      MAL_S3_VECTOR(Foot_P,double);
      //MAL_VECTOR_DIM(q,double,6);
      //MAL_VECTOR_DIM(q_mem,double,6);
      vectorN q;	
      MAL_VECTOR_RESIZE(q,6);
      vectorN q_mem;
      MAL_VECTOR_RESIZE(q_mem,6);
      matrix4d jointRootPosition;
      matrix4d jointEndPosition;
      vector3d Dtbis;           

      //Initialization of these matrices and vectors for the right leg:

      // body rotation
      MAL_S3x3_MATRIX_CLEAR(Body_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = 1.0;
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = 1.0;
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 0) = sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 1) = -sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;

      // body translation : vector (0,0,0)
      MAL_S3_VECTOR_FILL(Body_P,0.0);

      //we describe the horizontal configuration of the feet relative to the waist (considered as the root position and orientation), with 6 parameters: (Left) x,y,t1, (Right) x',y',t2.
      //VERY IMPORTANT REMARK: the positions have been calculated according to the initial orientation of the waist: zero. Therefore a rotation has to be performed in order to deal with that. 

      vector<double> feetVector;
      feetVector.resize(6);

      feetVector[0] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) 
	- (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
      feetVector[1] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) 
	+ (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
      feetVector[2] = stepF.leftfootOrient[count] - stepF.waistOrient[count];
      feetVector[3] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) 
	- (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
      feetVector[4] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) 
	+ (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
      feetVector[5] = stepF.rightfootOrient[count] - stepF.waistOrient[count];

      //we convert from feet to ankles:
      //feetVector[3]+=-cos(feetVector[5]*PI/180-PI/2)*0.035;
      //feetVector[4]+=-sin(feetVector[5]*PI/180-PI/2)*0.035;
      //feetVector[0]+=-cos(feetVector[2]*PI/180+PI/2)*0.035;
      //feetVector[1]+=-sin(feetVector[2]*PI/180+PI/2)*0.035;		

      double foot1X1=feetVector[0];
      double foot1Y1=feetVector[1];
      double foot1Theta1=feetVector[2]*PI/180;

      double foot2X1=feetVector[3];
      double foot2Y1=feetVector[4];
      double foot2Theta1=feetVector[5]*PI/180;

      //remark: the InverseKinematics takes into account the position of
      //the ankle so the z-limit for HRP2 is 60cm whereas 10.5cm have to be added to
      //get the actual distance between the ground and the waist root (i
      //suppose here that the HRP2's feet stay flat on the ground)
      //other remark: the ankle "zero" y-translation is 6cm.

      //Now we define the joint values for the right leg:

      // Dt (leg)
      MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
      MAL_S3_VECTOR_ACCESS(Dt,1) = -0.06;
      MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

      // ankle rotation
      MAL_S3x3_MATRIX_CLEAR(Foot_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

      // ankle translation
      MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot2X1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot2Y1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -(stepF.zc - 0.274) + stepF.rightfootHeight[count];
      // - 0.105

      //ComputeInverseKinematics2ForLegs(Body_R, Body_P,Dt,Foot_R, Foot_P,q);

      //*************************************************************************
      Dtbis(0) = Dt[0]; Dtbis(1) = Dt[1]; Dtbis(2) = Dt[2];			
      for(unsigned int i=0;i<3;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	    }
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3) = Body_P(i);
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3) = Foot_P(i);
	}
      getWaistFootKinematics(jointRootPosition,jointEndPosition,q,Dtbis);
      //*************************************************************************


      for(int k=0;k<6;k++)
	{
	  q_mem[k]=q[k];
	}

      //values for the left leg:

      MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
      MAL_S3_VECTOR_ACCESS(Dt,1) = +0.06;
      MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

      // ankle rotation
      MAL_S3x3_MATRIX_CLEAR(Foot_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

      // ankle translation
      MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot1X1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot1Y1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -(stepF.zc - 0.274) + stepF.leftfootHeight[count];

      //ComputeInverseKinematics2ForLegs(Body_R, Body_P, Dt, Foot_R, Foot_P, q);
      //*************************************************************************
      Dtbis(0) = Dt[0]; Dtbis(1) = Dt[1]; Dtbis(2) = Dt[2];
      for(unsigned int i=0;i<3;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	    }
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3) = Body_P(i);
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3) = Foot_P(i);
	}
      getWaistFootKinematics(jointRootPosition,jointEndPosition,q,Dtbis);
      //*************************************************************************

      jointsRadValues.clear();
      jointsRadValues.resize(42);

      jointsRadValues[0]= q_mem[0];
      jointsRadValues[1]= q_mem[1];
      jointsRadValues[2]= q_mem[2];
      jointsRadValues[3]= q_mem[3];
      jointsRadValues[4]= q_mem[4];
      jointsRadValues[5]= q_mem[5];

      jointsRadValues[6]= q[0];
      jointsRadValues[7]= q[1];
      jointsRadValues[8]= q[2];
      jointsRadValues[9]= q[3];
      jointsRadValues[10]= q[4];
      jointsRadValues[11]= q[5];

      jointsRadValues[12]=0.0;
      jointsRadValues[13]=0.0;

      jointsRadValues[14]=0.0;
      jointsRadValues[15]=0.0;

      jointsRadValues[16]=0.2583087;
      jointsRadValues[17]=-0.174533;
      jointsRadValues[18]=0.000000;
      jointsRadValues[19]=-0.523599;
      jointsRadValues[20]=0.000000;
      jointsRadValues[21]=0.000000;
      jointsRadValues[22]=0.174533;
      jointsRadValues[23]=0.174533;

      jointsRadValues[24]=0.2583087;
      jointsRadValues[25]=0.174533;
      jointsRadValues[26]=0.000000;
      jointsRadValues[27]=-0.523599;
      jointsRadValues[28]=0.000000;
      jointsRadValues[29]=0.000000;
      jointsRadValues[30]=0.174533;
      jointsRadValues[31]=0.174533;

      jointsRadValues[32]=0.0;
      jointsRadValues[33]=0.0;
      jointsRadValues[34]=0.0;
      jointsRadValues[35]=0.0;
      jointsRadValues[36]=0.0;
      jointsRadValues[37]=0.0;
      jointsRadValues[38]=0.0;
      jointsRadValues[39]=0.0;
      jointsRadValues[40]=0.0;
      jointsRadValues[41]=0.0;

      //MAL_VECTOR(,double) jointsRadValuesSmall = jointsRadValues;
      //jointsRadValuesSmall.resize(30);
      //for(unsigned int u = 12; u < 30; u++) jointsRadValuesSmall[u]=0;


      vector<double> tmpvectCOM(43);
      vector<double> tmpvectZMP(4);
      vector<double> tmpvectWaist(5);
      tmpvectCOM[0] = (stepF.incrTime * count);
      for(int j = 0; j < 42; j++)
	{
	  tmpvectCOM[j+1] = jointsRadValues[j];
	}
      trajTimedRadQ[count-from] = tmpvectCOM;

      tmpvectZMP[0] = (stepF.incrTime * count);
      tmpvectZMP[1] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180)
	-(stepF.zmpTrajY[count]-stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180) ;
      tmpvectZMP[2] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180)
	+(stepF.zmpTrajY[count]-stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180) ;
      tmpvectZMP[3] = ( -(stepF.zc - 0.274)-0.105 );
      trajTimedZMP[count-from] = tmpvectZMP;

      tmpvectWaist[0] = (stepF.incrTime * count);
      tmpvectWaist[1] = ( stepF.comTrajX[count]);
      tmpvectWaist[2] = ( stepF.comTrajY[count]);
      tmpvectWaist[3] = ( stepF.zc - 0.274 + 0.105);
      tmpvectWaist[4] = ( stepF.waistOrient[count]*PI/180);

      trajTimedWaist[count-from] = tmpvectWaist;

    }

}

void CgenFullBodyTrajectory::generateTrajectory(vector<vector<double> > & trajTimedRadQ, StepFeatures & stepF, int from, int size)
{

  trajTimedRadQ.clear();
  //trajTimedZMP.clear();

  MAL_VECTOR(,double) jointsRadValues;

  if(size<0) size = stepF.size - from;
  if(from+size>(int)stepF.size) size = stepF.size - from;

  trajTimedRadQ.resize(size);
  //trajTimedZMP.resize(stepF.size - from);

  for(int count = from; count < from + size ; count++)
    {

      MAL_S3x3_MATRIX(Body_R,double);
      MAL_S3_VECTOR(Body_P,double);
      MAL_S3_VECTOR(Dt,double);
      MAL_S3x3_MATRIX(Foot_R,double);
      MAL_S3_VECTOR(Foot_P,double);
      //MAL_VECTOR_DIM(q,double,6);
      //MAL_VECTOR_DIM(q_mem,double,6);
      vectorN q;
      MAL_VECTOR_RESIZE(q,6);
      vectorN q_mem;
      MAL_VECTOR_RESIZE(q_mem,6);
      matrix4d jointRootPosition;
      matrix4d jointEndPosition;
      vector3d Dtbis;

      //Initialization of these matrices and vectors for the right leg:

      // body rotation
      MAL_S3x3_MATRIX_CLEAR(Body_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = 1.0;
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = 1.0;
      MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 0) = sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 1) = -sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
      // MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;

      // body translation : vector (0,0,0)
      MAL_S3_VECTOR_FILL(Body_P,0.0);

      //we describe the horizontal configuration of the feet relative to the waist (considered as the root position and orientation), with 6 parameters: (Left) x,y,t1, (Right) x',y',t2.
      //VERY IMPORTANT REMARK: the positions have been calculated according to the initial orientation of the waist: zero. Therefore a rotation has to be performed in order to deal with that.

      vector<double> feetVector;
      feetVector.resize(6);

      feetVector[0] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) 
	- (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
      feetVector[1] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) 
	+ (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
      feetVector[2] = stepF.leftfootOrient[count] - stepF.waistOrient[count];
      feetVector[3] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) 
	- (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
      feetVector[4] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) 
	+ (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
      feetVector[5] = stepF.rightfootOrient[count] - stepF.waistOrient[count];

      //we convert from feet to ankles:
      //feetVector[3]+=-cos(feetVector[5]*PI/180-PI/2)*0.035;
      //feetVector[4]+=-sin(feetVector[5]*PI/180-PI/2)*0.035;
      //feetVector[0]+=-cos(feetVector[2]*PI/180+PI/2)*0.035;
      //feetVector[1]+=-sin(feetVector[2]*PI/180+PI/2)*0.035;

      double foot1X1=feetVector[0];
      double foot1Y1=feetVector[1];
      double foot1Theta1=feetVector[2]*PI/180;

      double foot2X1=feetVector[3];
      double foot2Y1=feetVector[4];
      double foot2Theta1=feetVector[5]*PI/180;

      //remark: the InverseKinematics takes into account the position of
      //the ankle so the z-limit for HRP2 is 60cm whereas 10.5cm have to be added to
      //get the actual distance between the ground and the waist root (i
      //suppose here that the HRP2's feet stay flat on the ground)
      //other remark: the ankle "zero" y-translation is 6cm.

      //Now we define the joint values for the right leg:

      // Dt (leg)
      MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
      MAL_S3_VECTOR_ACCESS(Dt,1) = -0.06;
      MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

      // ankle rotation
      MAL_S3x3_MATRIX_CLEAR(Foot_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot2Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

      // ankle translation
      MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot2X1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot2Y1;
      // - 0.105
      MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -(stepF.zc - 0.274) + stepF.rightfootHeight[count];

      //ComputeInverseKinematics2ForLegs(Body_R, Body_P,Dt,Foot_R, Foot_P,q);

      //*************************************************************************
      Dtbis(0) = Dt[0]; Dtbis(1) = Dt[1]; Dtbis(2) = Dt[2];
      for(unsigned int i=0;i<3;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	    }
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3) = Body_P(i);
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3) = Foot_P(i);
	}
      getWaistFootKinematics(jointRootPosition,jointEndPosition,q,Dtbis);
      //*************************************************************************


      for(int k=0;k<6;k++)
	{
	  q_mem[k]=q[k];
	}

      //values for the left leg:

      MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
      MAL_S3_VECTOR_ACCESS(Dt,1) = +0.06;
      MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

      // ankle rotation
      MAL_S3x3_MATRIX_CLEAR(Foot_R);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot1Theta1);
      MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

      // ankle translation
      MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot1X1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot1Y1;
      MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -(stepF.zc - 0.274) + stepF.leftfootHeight[count];

      //ComputeInverseKinematics2ForLegs(Body_R, Body_P, Dt, Foot_R, Foot_P, q);
      //*************************************************************************
      Dtbis(0) = Dt[0]; Dtbis(1) = Dt[1]; Dtbis(2) = Dt[2];
      for(unsigned int i=0;i<3;i++)
	{
	  for(unsigned int j=0;j<3;j++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Body_R,i,j);
	      MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,j) = MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R,i,j);
	    }
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointRootPosition,i,3) = Body_P(i);
	  MAL_S4x4_MATRIX_ACCESS_I_J(jointEndPosition,i,3) = Foot_P(i);
	}
      getWaistFootKinematics(jointRootPosition,jointEndPosition,q,Dtbis);
      //*************************************************************************

      jointsRadValues.clear();
      jointsRadValues.resize(42);

      jointsRadValues[0]= q_mem[0];
      jointsRadValues[1]= q_mem[1];
      jointsRadValues[2]= q_mem[2];
      jointsRadValues[3]= q_mem[3];
      jointsRadValues[4]= q_mem[4];
      jointsRadValues[5]= q_mem[5];

      jointsRadValues[6]= q[0];
      jointsRadValues[7]= q[1];
      jointsRadValues[8]= q[2];
      jointsRadValues[9]= q[3];
      jointsRadValues[10]= q[4];
      jointsRadValues[11]= q[5];

      jointsRadValues[12]=0.0;
      jointsRadValues[13]=0.0;

      jointsRadValues[14]=0.0;
      jointsRadValues[15]=0.0;

      jointsRadValues[16]=0.2583087;
      jointsRadValues[17]=-0.174533;
      jointsRadValues[18]=0.000000;
      jointsRadValues[19]=-0.523599;
      jointsRadValues[20]=0.000000;
      jointsRadValues[21]=0.000000;
      jointsRadValues[22]=0.174533;
      jointsRadValues[23]=0.174533;

      jointsRadValues[24]=0.2583087;
      jointsRadValues[25]=0.174533;
      jointsRadValues[26]=0.000000;
      jointsRadValues[27]=-0.523599;
      jointsRadValues[28]=0.000000;
      jointsRadValues[29]=0.000000;
      jointsRadValues[30]=0.174533;
      jointsRadValues[31]=0.174533;

      jointsRadValues[32]=0.0;
      jointsRadValues[33]=0.0;
      jointsRadValues[34]=0.0;
      jointsRadValues[35]=0.0;
      jointsRadValues[36]=0.0;
      jointsRadValues[37]=0.0;
      jointsRadValues[38]=0.0;
      jointsRadValues[39]=0.0;
      jointsRadValues[40]=0.0;
      jointsRadValues[41]=0.0;

      //MAL_VECTOR(,double) jointsRadValuesSmall = jointsRadValues;
      //jointsRadValuesSmall.resize(30);
      //for(unsigned int u = 12; u < 30; u++) jointsRadValuesSmall[u]=0;


      vector<double> tmpvectCOM(43);
      //vector<double> tmpvectZMP(4);
      tmpvectCOM[0] = (stepF.incrTime * count);
      for(int j = 0; j < 42; j++)
			{
				tmpvectCOM[j+1] = jointsRadValues[j];
			}
      trajTimedRadQ[count-from] = tmpvectCOM;

      /*tmpvectZMP[0] = (stepF.incrTime * count);
	tmpvectZMP[1] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180)
	-(stepF.zmpTrajY[count]-stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180) ;
	tmpvectZMP[2] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180)
	+(stepF.zmpTrajY[count]-stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180) ;
	tmpvectZMP[3] = ( -(stepF.zc - 0.274)-0.105 );
	trajTimedZMP[count-from] = tmpvectZMP;
      */
      stepF.zmpTrajX[count-from] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180)
									-(stepF.zmpTrajY[count]-stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180) ;
			stepF.zmpTrajY[count-from] = (stepF.zmpTrajX[count]-stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180)
									+(stepF.zmpTrajY[count]-stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180) ;
    }

}


