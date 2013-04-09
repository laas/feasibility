#include <algorithm>//lowerbound, sort
#include <map>
#include "contact_transition.h"
#include "util.h"
#include "rviz/rviz_visualmarker.h"
#include "environment.h"

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
	double xg = nodeGoal.g.x;
	double yg = nodeGoal.g.y;
	double x = this->g.x;
	double y = this->g.y;
	return sqrt( (x-xg)*(x-xg) + (y-yg)*(y-yg)); 
}
bool ContactTransition::IsGoal( ContactTransition &nodeGoal ){
	return this->GoalDistanceEstimate(nodeGoal) < 0.2;
}
/*
int ContactTransition::indexConversion(double x, double y, double t){
	int i1 = (int)floor(x*100+0.5);
	int i2 = (int)floor(y*100+0.5);
	int i3 = (int)floor(t*100+0.5);
	return (int)((floor((i1+35.0)/5.0))*8*5 + (floor((i2+37.0)/5.0))*5 + (floor((i3+52.0)/26.0)));
}
*/

void ContactTransition::showSuccessors( double x, double y, double t, char L_or_R){
	std::map<int, std::vector<double> >::const_iterator it;
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

		double valX = it->second.at(4);
		double valY = it->second.at(5);
		double valT = toRad(it->second.at(6)*100);

		double newX = abs_x + cos(abs_t)*valX - sin(abs_t)*valY;
		double newY = abs_y + sin(abs_t)*valX + cos(abs_t)*valY;
		double newT = valT - abs_t;

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

		ContactTransition next(ng);
		next.L_or_R = foot;
		//next.cost_so_far = this->getPlanarDistance(ng) + this->GoalDistanceEstimate( next );
		next.cost_so_far = 0.0;
		//compute both hyperplane approx for this --> intermediate step
		//and next -->intermediate step
		//if(first){

//		if(++counter%10 == 0){
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
// support foot (SF)
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

		if(foot == 'R'){
			rx = cos(sf_yaw)*tx + sin(sf_yaw)*ty;
			ry = sin(sf_yaw)*tx - cos(sf_yaw)*ty;
			ryaw = -(yaw - sf_yaw);
		}
		while(ryaw>M_PI) ryaw-=2*M_PI;
		while(ryaw<-M_PI) ryaw+=2*M_PI;

		//--> cylinder coordinates
		std::vector<double> cyl(3);
		cyl.at(0)=sqrtf(rx*rx + ry*ry); cyl.at(1)=atan2(rx,ry); cyl.at(2)=yaw;

		v.push_back(cyl);

	}
	return v;
}
bool ContactTransition::GetSuccessors( AStarSearch<ContactTransition> *astarsearch, ContactTransition *parent_node ){
	std::map<int, std::vector<double> >::const_iterator it;
	char foot;

	if(L_or_R == 'L'){
		foot='R';
	}else{
		foot='L';
	}

	double abs_x = this->g.x;
	double abs_y = this->g.y;
	double abs_t = this->g.getYawRadian();

	std::vector< std::vector<double> > obj = this->prepareObjectPosition(abs_x, abs_y, abs_t, L_or_R);

	for(  it = hyperplane.begin(); it != hyperplane.end(); ++it ){

		double valX = it->second.at(4);
		double valY = it->second.at(5);
		double valT = toRad(it->second.at(6)*100);

		std::vector<double> relStepPos(3);
		relStepPos.at(0)=it->second.at(4); relStepPos.at(1)=it->second.at(5); relStepPos.at(2)=it->second.at(6);

		double newX = abs_x + cos(abs_t)*valX - sin(abs_t)*valY;
		double newY = abs_y + sin(abs_t)*valX + cos(abs_t)*valY;
		double newT = valT - abs_t;

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

		ContactTransition next(ng);
		next.L_or_R = foot;
		//next.cost_so_far = this->getPlanarDistance(ng) + this->GoalDistanceEstimate( next );
		next.cost_so_far = computeHyperPlaneDistance(relStepPos, obj);
		astarsearch->AddSuccessor(next);
		/*
		if(rand()>0.98){
			if(foot=='L'){
				ros::LeftFootMarker rp(ng.x, ng.y, ng.getYawRadian());
				rp.publish();
			}else{
				ros::RightFootMarker rp(ng.x, ng.y, ng.getYawRadian());
				rp.publish();
			}
		}
		*/
	}
	return true;
}

//input: obj in cylinder coordinates (r,phi, yaw)| relative footstep of free
//foot (FF), in X,Y,T, whereby T is degree divided by 100
double ContactTransition::computeHyperPlaneDistance(std::vector<double> &g, std::vector< std::vector<double> > &obj){
	uint hash = hashit<double>(g);
	//ROS_INFO("%d objects", obj.size());

	if(hyperplane.find(hash)==hyperplane.end()){
		ROS_INFO("could not find hyperplane hash for geometry %f %f %f (hash: %d)",g.at(0),g.at(1),g.at(2),hash);
		throw "hash not found";
		exit(-1);
	}
	std::vector<double> params = hyperplane[hash];

	double cost = 0.0;

	double a = params.at(0);
	double b = params.at(1);
	double c = params.at(2);
	double d = params.at(3);

	//ROS_INFO("hyperplane %f %f %f %f", a,b,c,d);

	std::vector< std::vector<double> >::const_iterator oit;
	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		//compute distance from hyperplane (TODO: port to eigen to make
		//it accesible in N dimensions)

		double ox = (*oit).at(0);
		double oy = (*oit).at(1);
		double ot = (*oit).at(2);
		//ROS_INFO("object %f %f %f", ox, oy, ot);

		double dist = (a*ox+b*oy+c*ot+d) / sqrtf(a*a+b*b+c*c);
		if( dist > 0){
			cost += exp(-dist);
		}else{
			cost += exp(-dist);
		}
		//ROS_INFO("dist %f cost %f", dist, cost);

	}

	return cost;
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

void ContactTransition::nearestDiscreteGeometry(std::vector<int> &in, double x, double y, double yaw){

	/*

	std::vector<int> tpos = vecI(-29,-14,0,14,29);
	std::vector<int> xpos = vecI(-34, -29, -24, -19, -14, -9, -4, 0, 5, 10, 15, 20, 25, 30, 35);
	std::vector<int> ypos = vecI(-36, -31, -26, -21, -16, -11, -6, -1);
	cout << xpos << endl;
	cout << ypos << endl;
	cout << tpos << endl;

	std::sort(xpos.begin(), xpos.end());
	std::sort(ypos.begin(), ypos.end());
	std::sort(tpos.begin(), tpos.end());

	cout << x << " " << y << " " <<  t << endl;
	int rx = round2(x);
	int ry = round2(y);
	int rt = round2(t);
	cout << rx << " " << ry << " " << rt << endl;
	int nx = nearest(xpos, rx);
	int ny = nearest(ypos, ry);
	int nt = nearest(tpos, rt);
	cout << nx << " " << ny << " " << nt << endl;
	in.push_back( nearest(xpos, rx) );
	in.push_back( nearest(ypos, ry) );
	in.push_back( nearest(tpos, rt) );
	*/
	in.push_back(x);
	in.push_back(y);
	in.push_back(yaw);

}
void ContactTransition::loadObjectPosition(Environment &env){

}
void ContactTransition::loadHyperPlaneParameters(const char *file){

	//FILE *fp = fopen_s(file,'r');
	ROS_INFO("loading parameter from %s", file);
	CSVReader f(file);
	std::vector< std::vector<double> > vv = f.getVV(7);
	cout << vv.at(0) << endl;
	cout << vv.at(1) << endl;
	cout << vv.at(2) << endl;

	bool collision=false;
	for(uint k=0;k<vv.size();k++){
		std::vector<double> params(4);
		for(uint i=0;i<4;i++) params.push_back(vv.at(k).at(i));

		std::vector<double> pos;

		for(uint i=4;i<7;i++) pos.push_back(vv.at(k).at(i)/100.0);

		uint hash = hashit<double>(pos);

		ROS_INFO("hash: %d (%f %f %f)", hash, pos.at(0), pos.at(1), pos.at(2));
		if(hyperplane.find(hash)!=hyperplane.end()){
			ROS_INFO("hash collision: %d", hash);
			collision=true;
		}
		for(uint i=4;i<7;i++) vv.at(k).at(i)=pos.at(i-4);
		hyperplane[hash] = vv.at(k);
	}
	if(collision){
		ROS_INFO("WARNING: collision in hyperplane");
		throw "collision in hyperplane error";
		exit(-1);
	}
	ROS_INFO("successfully loaded hyperplane hash map without collisions and %d entries!", hyperplane.size());
}
