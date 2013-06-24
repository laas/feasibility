#include "planner/constraints_checker_naive.h"

#define DEBUG(x)
#define DEBUGOBJ(x)
ConstraintsCheckerNaive::ConstraintsCheckerNaive(){
}

bool ConstraintsCheckerNaive::isFeasible(  
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	std::vector<std::vector<double> > v;
	double x=p.at(0);
	double y=p.at(1);
	std::vector< std::vector<double> >::const_iterator oit;
	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		double ox = (*oit).at(0);
		double oy = (*oit).at(1);
		double radius = (*oit).at(2);
		if( norml2(ox,x,oy,y) < radius+0.2){ //0.2m width of foot
			return false;
		}
	}
	return true;
}
std::vector< std::vector<double> > 
ConstraintsCheckerNaive::prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &objects, 
		double sf_x, double sf_y, double sf_yaw, char foot){

	std::vector< std::vector<double> > v;
	std::vector< std::vector<double> >::const_iterator vit;
	std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
	uint c=0;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		double obj_x = (*oit)->g.x;
		double obj_y = (*oit)->g.y;
		DEBUGOBJ(ROS_INFO("object at %f %f", obj_x, obj_y);)
		DEBUGOBJ(ROS_INFO("pos foot at %f %f %f", sf_x, sf_y, toDeg(sf_yaw));)

		//translate object, so that origin and sf origin conincide
		double tx = obj_x - sf_x;
		double ty = obj_y - sf_y;
		//rotate object around origin, such that object is aligned with
		//sf
		double rx = cos(sf_yaw)*tx + sin(sf_yaw)*ty;
		double ry = sin(sf_yaw)*tx - cos(sf_yaw)*ty;

		std::vector<double> d(4);
		//X,Y,R,H
		d.at(0)=rx;
		d.at(1)=(foot=='R'?-ry:ry);//if the support foot is the right one, we have to invert the object position (precomputation did only take place in the left foot space)
		d.at(2)=(*oit)->g.getRadius();
		d.at(3)=(*oit)->g.getHeight();

		double dist = sqrtf(rx*rx+ry*ry);
		DEBUGOBJ(ROS_INFO("object %d/%d at %f %f (dist %f)", c++, objects.size(), rx, ry, dist);)
		if(dist<MAX_SWEPT_VOLUME_LIMIT){
			v.push_back(d);
		}
		//DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

	}
	return v;
}//prepare objects
