#include <fast-replanning/fast-replanning-interface.hh>
#include "motionplanner.h"
#include "trajectory_visualizer.h"

//Wrapper around fastReplanning library
struct MotionPlannerPerrin: public MotionPlanner{
	fastreplanning::FastReplanningInterface *planner;
	TrajectoryVisualizer *tv;
	bool _step_finished;

	MotionPlannerPerrin(Environment *env, int &argc, char** &argv);
	virtual void addObjectToPlanner(ros::RVIZVisualMarker *m);
	void getAbsoluteFromRelativeFootSteps( std::vector<fastreplanning::footStepInterface> &fsi );
	virtual void cleanObjects();
	void start_planner();
	void setStart( ros::Geometry &pos );
	void setGoal( ros::Geometry &pos );
	bool success();
	void publish();
};
