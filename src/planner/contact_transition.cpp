#include <algorithm>//lowerbound, sort
//#include <map>
#include <unordered_map>
#include <algorithm> //max_element
#include "contact_transition.h"
#include "util.h"
#include "rviz/rviz_visualmarker.h"
#include <Eigen/Geometry>
#include <Eigen/Core>

#define DEBUG(x)
#define TIMER_DEBUG(x) x

ContactTransition::ContactTransition(){
}
void ContactTransition::print(){
	g.print();
}
ContactTransition::ContactTransition( ros::Geometry &g){
	this->g = g;
	this->g.setYawRadian( g.getYawRadian() );
}
double ContactTransition::GoalDistanceEstimate( ContactTransition &nodeGoal ){
	//heuristic = distance to goal + distance to obstacles
	double xg = nodeGoal.g.x;
	double yg = nodeGoal.g.y;
	double x = this->g.x;
	double y = this->g.y;
	return norml2(x,xg,y,yg);
}
bool ContactTransition::IsGoal( ContactTransition &nodeGoal ){
	return norml2(this->g.x, nodeGoal.g.x, this->g.y, nodeGoal.g.y) < 0.3;
}

ros::Geometry ContactTransition::computeAbsFFfromRelFF(
		double sf_abs_x, double sf_abs_y, double sf_abs_yaw, 
		double ff_rel_x, double ff_rel_y, double ff_rel_yaw,
		char sf_foot){
	//absolute position of next pos of FF
	double ff_abs_x = sf_abs_x + cos(sf_abs_yaw)*ff_rel_x - sin(sf_abs_yaw)*ff_rel_y;
	double ff_abs_y = sf_abs_y + sin(sf_abs_yaw)*ff_rel_x + cos(sf_abs_yaw)*ff_rel_y;
	double ff_abs_yaw = ff_rel_yaw + sf_abs_yaw;

	if(sf_foot == 'R'){
		//reflection at x-axis [1 0,0 -1], following rotation around z and
		//translation to x,y
		ff_abs_x = sf_abs_x + cos(sf_abs_yaw)*ff_rel_x + sin(sf_abs_yaw)*ff_rel_y;
		ff_abs_y = sf_abs_y + sin(sf_abs_yaw)*ff_rel_x - cos(sf_abs_yaw)*ff_rel_y;
		ff_abs_yaw = -(ff_rel_yaw - sf_abs_yaw);
	}
	while(ff_abs_yaw>M_PI) ff_abs_yaw-=2*M_PI;
	while(ff_abs_yaw<-M_PI) ff_abs_yaw+=2*M_PI;

	ros::Geometry ff_abs;
	ff_abs.x = ff_abs_x;
	ff_abs.y = ff_abs_y;
	ff_abs.setYawRadian(ff_abs_yaw);

	return ff_abs;
}
	
void ContactTransition::showSuccessors( double x, double y, double t, char L_or_R){
	HashMap::const_iterator it;
	char foot = 'L';
	if(L_or_R == 'L'){
		foot='R';
		ros::LeftFootMarker m(x,y,t);
		m.publish();
	}else{
		foot='L';
		ros::RightFootMarker m(x,y,t);
		m.publish();
	}

	double abs_x = x;
	double abs_y = y;
	double abs_t = t;

	uint counter=0;
	//CHECK( (abs_t <= M_PI && abs_t >= -M_PI), "angle not in right interval");

	for(  it = hyperplane.begin(); it != hyperplane.end(); ++it ){

		double valX = it->second.at(5);
		double valY = it->second.at(6);
		double valT = toRad(it->second.at(7)*100.0);

		double newX = abs_x + cos(abs_t)*valX - sin(abs_t)*valY;
		double newY = abs_y + sin(abs_t)*valX + cos(abs_t)*valY;
		double newT = valT + abs_t;

		if(L_or_R == 'R'){
			//reflection at x-axis [1 0,0 -1], following rotation around z and
			//translation to x,y
			newX = abs_x + cos(abs_t)*valX + sin(abs_t)*valY;
			newY = abs_y + sin(abs_t)*valX - cos(abs_t)*valY;
			newT = -(valT - abs_t);
		}
		while(newT>M_PI) newT-=2*M_PI;
		while(newT<-M_PI) newT+=2*M_PI;

		ros::Geometry ng;
		ng.x = newX;
		ng.y = newY;
		ng.setYawRadian(newT);

		if(rand()>0.0){
			if(foot=='L'){
				ros::LeftFootMarker rp(ng.x, ng.y, ng.getYawRadian());
				rp.publish();
			}else{
				ros::RightFootMarker rp(ng.x, ng.y, ng.getYawRadian());
				rp.publish();
			}
		}
		//}
	}
}
bool ContactTransition::GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node ){
	HashMap::const_iterator it;
	char foot;
	ContactTransition *parent = this;

	(parent->L_or_R == 'L')? foot='R':foot='L';

	//NOTE:
	//relative means always with respect to the support foot
	//absolute means the position in the global world coordinate frame

	//get absolute position of support foot (SF)
	double sf_abs_x = parent->g.x;
	double sf_abs_y = parent->g.y;
	double sf_abs_yaw = parent->g.getYawRadian();


	TIMER_DEBUG(timer->begin("prepare"));
	//project all objects into the reference frame of the SF
	std::vector< std::vector<double> > obj = this->prepareObjectPosition(sf_abs_x, sf_abs_y, sf_abs_yaw, parent->L_or_R);
	TIMER_DEBUG(timer->end("prepare"));

	//get the relative position of the starting free foot (FF)
	double ff_rel_x = parent->rel_x_parent;
	double ff_rel_y = parent->rel_y_parent;
	double ff_rel_yaw = parent->rel_yaw_parent;

	//Eigen::VectorXd sf_abs; sf_abs << sf_abs_x, sf_abs_y, sf_abs_yaw;
	//Eigen::VectorXd ff_rel; ff_rel << ff_rel_x, ff_rel_y, ff_rel_yaw;

	std::vector<double> ff_rel(3);
	ff_rel.at(0)=ff_rel_x; ff_rel.at(1)=ff_rel_y; ff_rel.at(2)=ff_rel_yaw;

	//double start_plane_distance = this->computeHyperPlaneDistance(ff_rel, obj);

	uint counter=0;
	TIMER_DEBUG( timer->begin("hyperplanar") );

	for(  it = hyperplane.begin(); it != hyperplane.end(); ++it ){
		//if(counter++ % 2 ==0) continue; //consider only every second step -- simple speed up hack
		if(rand(0,1)>0.2) continue;


		//double hyper =  max(start_plane_distance , end_plane_distance); //nearest point to hyperplane or beyond

		if(!isInCollision( it->second, obj)){ //negative values mean to be outside of hyperplane approximation of the object
			//relative position of the next proposed pos of FF
			TIMER_DEBUG( timer->begin("ff") );

			double next_ff_rel_x = it->second.at(5);
			double next_ff_rel_y = it->second.at(6);
			double next_ff_rel_yaw = toRad(it->second.at(7)*100.0);

			ros::Geometry ng = computeAbsFFfromRelFF(sf_abs_x, sf_abs_y, sf_abs_yaw, next_ff_rel_x, next_ff_rel_y, next_ff_rel_yaw, L_or_R);

			ContactTransition next(ng);
			next.L_or_R = foot;
			next.rel_x_parent = it->second.at(5);
			next.rel_y_parent = it->second.at(6);
			next.rel_yaw_parent = it->second.at(7);

			next.cost_so_far = 0.0;
			astarsearch->AddSuccessor(next);

			DEBUG(
				if(rand(0,1)>0.0){
					if(foot=='L'){
						ros::LeftFootMarker rp(ng.x, ng.y, ng.getYawRadian());
						rp.publish();
					}else{
						ros::RightFootMarker rp(ng.x, ng.y, ng.getYawRadian());
						rp.publish();
					}
				}
			);
			TIMER_DEBUG( timer->end("ff") );
			continue;
		}

	TIMER_DEBUG( timer->end("hyperplanar") );
	}//foreach hyperplane
	return true;
}
// support foot (SF)
//get object position in the coordinate system of the SF -- and prune objects
//which are too far away
std::vector< std::vector<double> > ContactTransition::prepareObjectPosition(double sf_x, double sf_y, double sf_yaw, char foot){
	std::vector< std::vector<double> > v;
	std::vector< std::vector<double> >::const_iterator vit;
	std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		double x = (*oit)->g.x;
		double y = (*oit)->g.y;
		double yaw = (*oit)->g.getYawRadian();

		//translate object, so that origin and sf origin conincide
		double tx = x - sf_x;
		double ty = y - sf_y;
		//rotate object around origin, such that object is aligned with
		//sf
		double rx = cos(sf_yaw)*tx - sin(sf_yaw)*ty;
		double ry = sin(sf_yaw)*tx + cos(sf_yaw)*ty;
		double ryaw = yaw - sf_yaw;

		//swept vlumes are always tested on the right side
		while(ryaw>M_PI) ryaw-=2*M_PI;
		while(ryaw<-M_PI) ryaw+=2*M_PI;

		//--> cylinder coordinates
		std::vector<double> cyl(3);
		cyl.at(0)=sqrtf(rx*rx + ry*ry); cyl.at(1)=atan2(rx,ry); cyl.at(2)=yaw;

		//prune objects, which are far away
		if(cyl.at(0)<0.5){
			v.push_back(cyl);
		}
		DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

	}
	return v;
}

bool ContactTransition::isInCollision(  const std::vector<double> &p, const std::vector< std::vector<double> > &obj){
	double hyper = this->computeHyperPlaneDistance( p, obj);
	if(hyper<0){
		//no collision
		return false;
	}else{
		return true;
	}
}

double ContactTransition::GetCost( ContactTransition &successor ){
	return successor.cost_so_far;
}
bool ContactTransition::IsSameState( ContactTransition &rhs ){
	double xg = rhs.g.x;
	double yg = rhs.g.y;
	double yawg = rhs.g.getYawRadian();
	double x = this->g.x;
	double y = this->g.y;
	double yaw = this->g.getYawRadian();
	return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg) + (yaw-yawg)*(yaw-yawg))<0.01;
}

// OUTPUT: [HyperplaneParams NormalizedConstant RelativeFootPosition]
// ---> [a b c d Z x y theta], whereby a,b,c,d are the hyperplane params,
// Z is sqrt(a*a+b*b+c*c) and x,y,theta is the position of the free foot,
// relative to the support foot

void ContactTransition::loadHyperPlaneParameters(const char *file){

	//FILE *fp = fopen_s(file,'r');
	ROS_INFO("loading parameter from %s", file);
	CSVReader f(file);
	std::vector< std::vector<double> > vv = f.getVV(7);

	bool collision=false;
	for(uint k=0;k<vv.size();k++){
		// p = [a b c d Z], whereby Z=sqrt(a*a+b*b+c*c)
		std::vector<double> params(5);
		for(uint i=0;i<4;i++){
			params.at(i) = (vv.at(k).at(i));
		}

		double norm= 0.0;
		for(uint i=0;i<3;i++){ //a*a+b*b+c*c
			norm += vv.at(k).at(i)*vv.at(k).at(i);
		}
		for(uint i=0;i<4;i++){
			params.at(i)/=sqrtf(norm);
		}
		params.at(4)= sqrtf(norm);

		//position (x,y,t) relative of FF
		std::vector<double> pos;
		for(uint i=4;i<7;i++) pos.push_back(vv.at(k).at(i)/100.0);

		uint hash = hashit<double>(pos);

		ROS_INFO("hash: %d (%f %f %f)", hash, pos.at(0), pos.at(1), pos.at(2));
		if(hyperplane.find(hash)!=hyperplane.end()){
			ROS_INFO("hash collision: %d", hash);
			collision=true;
		}
		std::vector<double> params_and_pos;
		for(uint i=0;i<params.size();i++) params_and_pos.push_back( params.at(i) );
		for(uint i=0;i<pos.size();i++) params_and_pos.push_back( pos.at(i) );
		hyperplane[hash] = params_and_pos;
	}
	if(collision){
		ROS_INFO("WARNING: collision in hyperplane");
		throw "collision in hyperplane error";
		exit(-1);
	}
	ROS_INFO("successfully loaded hyperplane hash map without collisions and %d entries!", hyperplane.size());
}
double ContactTransition::computeHyperPlaneDistance( const std::vector<double> &p, const std::vector< std::vector<double> > &obj){
	if(obj.size()==0){
		return -999;
	}

	double a = p.at(0);
	double b = p.at(1);
	double c = p.at(2);
	double d = p.at(3);

	std::vector<double> cost_per_object;
	std::vector< std::vector<double> >::const_iterator oit;

	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		//compute distance from hyperplane (TODO: port to eigen to make
		//it accesible in N dimensions)
		double ox = (*oit).at(0);
		double oy = (*oit).at(1);
		double ot = (*oit).at(2);
		//double dist = (p.at(0)*ox+p.at(1)*oy+p.at(2)*ot+p.at(3));//sqrtf(a*a+b*b+c*c);
		double dist = (a*ox+b*oy+c*ot+d) - 1.0;
		cost_per_object.push_back( dist );
		//cost_per_object.push_back( ((p.at(0)*(*oit).at(0)+p.at(1)*(*oit).at(1)+p.at(2)*(*oit).at(2)+p.at(3))/p.at(4)) - 1.0 );
		//we learned the objects with a probability metric of +1 >
	}
	return *max_element(cost_per_object.begin(), cost_per_object.end());
}
/*
//############################################################################
//## Fann functions
//############################################################################
std::vector< std::vector<double> > ContactTransition::prepareObjectPositionNN(double sf_x, double sf_y, double sf_r, char foot){
	std::vector< std::vector<double> > v;
	std::vector< std::vector<double> >::const_iterator vit;
	std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		double x = (*oit)->g.x;
		double y = (*oit)->g.y;
		double yaw = (*oit)->g.getYawRadian();

		//translate object, so that origin and sf origin conincide
		double tx = x - sf_x;
		double ty = y - sf_y;
		//rotate object around origin, such that object is aligned with

		//--> cylinder coordinates
		std::vector<double> obj(3);
		cyl.at(0)=tx;
		cyl.at(1)=ty;
		sqrtf(rx*rx + ry*ry); cyl.at(1)=atan2(rx,ry); cyl.at(2)=yaw;

		//prune objects, which are far away
		if(cyl.at(0)<0.5){
			v.push_back(cyl);
		}
		DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

	}
	return v;

}
//input: obj in euclidean coordinates (x,y,r)| relative to footstep of free
//foot (FF), in X,Y,T, whereby T is degree divided by 100
double ContactTransition::computeNNDistance( const std::vector<double> &p, const std::vector< std::vector<double> > &obj){


*/
#include <dirent.h>
void ContactTransition::loadNNParameters(const char *path){
	//struct fann *ann = fann_create_from_file(argv[1]);
	bool collision=false;
	DIR* dpath = opendir( path );
	if ( dpath ) 
	{
		struct dirent* hFile;
		errno = 0;
		while (( hFile = readdir( dpath )) != NULL ){
			if( !strcmp( hFile->d_name, "."  )) continue;
			if( !strcmp( hFile->d_name, ".." )) continue;
			if( hFile->d_name[0] == '.' ) continue; //hidden files

			if( strstr( hFile->d_name, ".net" )){
				printf("found neural network %s, try loading it", hFile->d_name );
				std::string file = hFile->d_name;
				std::vector<double> v = extract_num_from_string(file);

				//compute hash from v (position of free foot)
		
				//load d_name into fann structure
				struct fann *ann = fann_create_from_file(hFile->d_name);

				uint hash = hashit<double>(v);

				if(neuralMap.find(hash)!=neuralMap.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}
				neuralMap[hash] = ann;
				
				
			}
		} 
		closedir( dpath );
	}
}

