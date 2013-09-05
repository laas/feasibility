#include "planner/motionplanner_perrin.h"
//Wrapper around fastReplanning library
#define DEBUG(x) x

MotionPlannerPerrin::MotionPlannerPerrin(Environment *env, int &argc, char** &argv): MotionPlanner(env){
	ros::Geometry start_loc = env->getStart();
	tv = new TrajectoryVisualizer(start_loc.x, start_loc.y);

	std::string prefix = get_data_path();
	ROS_INFO("%s", prefix.c_str());

	planner = fastreplanning::fastReplanningInterfaceFactory(prefix, argc, argv);

	planner->setVerboseLevel(10); //0 5 15
	//planner->setStateLevel(fastreplanning::tools::RUN);
	setStart(start_loc);
	planner->initStep(); //init

	_step_finished=false;
}

void MotionPlannerPerrin::addObjectToPlanner(ros::RVIZVisualMarker *m){
	ros::Geometry *g = m->getGeometry();
	ros::TriangleObject *t = static_cast<ros::TriangleObject*>( m );
	//the z-value has to be >0.05 --- otherwise the planner does not find a solution
	planner->addAGenericPQPModel(t->get_pqp_ptr(), g->x, g->y, 0.05, 0.0, 0.0, 0.0); 
}
void MotionPlannerPerrin::cleanObjects(){
	//only pointer was send to planner, so we can ignore it for now
}

void MotionPlannerPerrin::start_planner(){
	/*
	if(!_step_finished){
		ROS_INFO("replan");
		//planner->initStep();
		planner->replanStep();
	}else{
		_step_finished=false;
		planner->replanStep();
	}
	*/
	std::vector<double> foot = vecD( cur_sf_x, cur_sf_y, cur_sf_yaw, cur_sf_foot );
	planner->replanStep(&foot);
	//planner->mainLoop();
	//planner->setStateLevel(fastreplanning::tools::WAIT);

	std::vector<fastreplanning::footStepInterface> fsi;
	planner->getInterfaceSteps(fsi);
	results.steps = fsi.size();
}

void MotionPlannerPerrin::setStart( ros::Geometry &pos ){
	std::vector<double> start = vecD(pos.x, pos.y, pos.z);
	std::vector<double> com = vecD(0,0,0);

	planner->updateLocalizationProtected(start, com);
}

void MotionPlannerPerrin::setGoal( ros::Geometry &pos ){
	std::vector<double> goal;
	goal.push_back(pos.x); goal.push_back(pos.y); goal.push_back(pos.z);
	planner->update3DGoalPositionProtected(goal);
	ROS_INFO("set GOAL TO  %f %f", pos.x , pos.y );
}

bool MotionPlannerPerrin::success(){
	std::vector<fastreplanning::footStepInterface> fsi;
	planner->getInterfaceSteps(fsi);
	return fsi.size()>0;
}

void MotionPlannerPerrin::getAbsoluteFromRelativeFootSteps( std::vector<fastreplanning::footStepInterface> &fsi ){
	//half-foot-step-format v.3.0:
	// 1: x
	// 2: y
	// 3: theta
	// 4: ascii code for L or R
	// NEW: using 5,6,7 for ABSOLUTE positions
	if(!fsi.size()>0){
		return;
	}
	double xold = 0.0;
	double yold = 0.0;
	double told = 0.0;
	for(uint i=0;i<fsi.size();i++){

		double x = fsi.at(i).data.at(0);
		double y = fsi.at(i).data.at(1);
		double t = fsi.at(i).data.at(2);
		char foot = fsi.at(i).data.at(3);

		double abs_x = xold;
		double abs_y = yold;
		double abs_t = told;

		double newX = abs_x + cos(abs_t)*x-sin(abs_t)*y;
		double newY = abs_y + sin(abs_t)*x+cos(abs_t)*y;
		double newT = t + abs_t;

		while(newT<-M_PI) newT+=2*M_PI;
		while(newT>M_PI)  newT-=2*M_PI;

		xold=newX;
		yold=newY;
		told=newT;

		//left foot is computed from the waypoint, transformation from
		//execution-replanning-interface-steps.cpp
		double cy = 0.095;
		if(foot=='L'){
			newY = abs_y - (cos(abs_t)+1)*cy + sin(abs_t)*x + cos(abs_t)*y;
			newX = newX + sin(newT)*cy;
		}
		fsi.at(i).data.at(5) = newX;
		fsi.at(i).data.at(6) = newY;
		fsi.at(i).data.at(7) = newT;
	}

}
void MotionPlannerPerrin::publish(){
	//
	// q vector is filled like this:
	//
	//q[0]=ID
	//q[1]=end of trajectory
	//q[2]=time start
	//q[3]=time end
	//--- 17 values per frame:
	//q[4:15]=q values (12 values)
	//q[16]=comX
	//q[17]=comY
	//q[18]=waistOrient in radian
	//q[19]=zmpX
	//q[20]=zmpY
	//
	//the next starting points for frames i=1:T
	//-> q[4 + i*17 + k], k=0:16;

	//obtain footsteps from planner
	std::vector<fastreplanning::footStepInterface> fsi;
	planner->getInterfaceSteps(fsi);
	getAbsoluteFromRelativeFootSteps(fsi);

	Logger log_steps("playback_steps.dat");
	Logger log_q("playback_q.dat");

	if(fsi.size()>0){
		ROS_INFO("NUMBER OF FOOTSTEPS: %d", fsi.size());

		for(uint i=0;i<fsi.size();i++){
			log_steps(fsi.at(i).data);
			double x = fsi.at(i).data.at(5);
			double y = fsi.at(i).data.at(6);
			double yaw = fsi.at(i).data.at(7);
			char foot = fsi.at(i).data.at(3);

			if(foot == 'R'){
				ros::RightFootMarker f(x,y,yaw);
				if(i==0) f.reset(); //clear all previous footsteps in rviz
				f.publish();
			}else{
				ros::LeftFootMarker f(x,y,yaw);
				if(i==0) f.reset(); //clear all previous footsteps in rviz
				f.publish();
			}

			ROS_INFO("[%d] at x=%f, y=%f, theta=%f", i,x,y,yaw);
		}
	}
	std::vector<double> q = planner->getArticulatedValues();
	//void generateTrajectory(vector<vector<double> > & trajTimedRadQ, StepFeatures & stepF, int from = 0, int size = -1);                                               

	log_q(q, "\n");
	ROS_INFO("Articulated values: %d", q.size());

	uint i = 1;
	if(q.size()>0 && fsi.size()>i){
		//play footstep transition and update starting position
		tv->init(q);
		tv->setCoMOffset(cur_com_x, cur_com_y, cur_com_t);
		ros::Rate r(500);
		while(tv->next()){
			ros::spinOnce();
			r.sleep();
		}
		DEBUG(
			if(fsi.at(i).data.at(1)>100000.0){ HALT("wrong y error spotted") }
		)

		double x = fsi.at(i).data.at(5);
		double y = fsi.at(i).data.at(6);
		double t = fsi.at(i).data.at(7);
		char foot = fsi.at(i).data.at(3);

		std::vector<double> lastFoot = vecD(x,y,t,foot);
		std::vector<double> newCoM = tv->getFinalCoM();

		cur_sf_x = lastFoot.at(0);
		cur_sf_y = lastFoot.at(1);
		cur_sf_yaw = lastFoot.at(2);
		cur_sf_foot = foot;

		cur_com_x = newCoM.at(0);
		cur_com_y = newCoM.at(1);
		cur_com_t = newCoM.at(2);

		planner->updateLocalizationProtected( lastFoot, newCoM );

		uint curIndex;
		planner->getCurrentStepIndex( curIndex );
		ROS_INFO("------------------ CURRENT STEP INDEX %d", curIndex);
		if(curIndex >= fsi.size()-1){
			//std::vector<double> lastFoot = vecD(0,0,0,'L');
			//std::vector<double> newCoM = vecD(0,0,0);
			//ROS_INFO("restarting trajectory");
			//planner->updateLocalizationProtected( lastFoot, newCoM );
		}
	}
	//ROS_INFO("set START TO  %f %f", pos.x , pos.y );

}//publish
