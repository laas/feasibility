#include <fast-replanning/fast-replanning-interface.hh>
#include "motionplanner.h"
#include "trajectory_visualizer.h"

//Wrapper around fastReplanning library
struct MotionPlannerPerrin: public MotionPlanner{
	fastreplanning::FastReplanningInterface *planner;
	TrajectoryVisualizer *tv;

	MotionPlannerPerrin(Environment *env, int &argc, char** &argv);
	virtual void addObjectToPlanner(ros::RVIZVisualMarker *m);
	virtual void cleanObjects();
	void start_planner();
	void setStart( ros::Geometry &pos );
	void setGoal( ros::Geometry &pos );
	bool success();
	void publish();
};
