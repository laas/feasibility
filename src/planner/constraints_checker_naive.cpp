#include <dirent.h>
#include "planner/constraints_checker_naive.h"

#define DEBUG(x)
#define DEBUGOBJ(x)
ConstraintsCheckerNaive::ConstraintsCheckerNaive(){
	this->loadActionSpace("extern/fann/datasets/humanoids/");
}

bool ConstraintsCheckerNaive::isFeasible(  
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	std::vector<std::vector<double> > v;
	double x=p.at(0);
	double y=p.at(1);
	//DEBUGOBJ(ROS_INFO("x %f, y %f", x, y);)
	std::vector< std::vector<double> >::const_iterator oit;
	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		double ox = (*oit).at(0);
		double oy = (*oit).at(1);
		double radius = (*oit).at(2);
		if( norml2(ox,x,oy,y) < (radius+0.132) ){ //0.132m radius of foot
			return false;
		}
	}
	return true;
}
std::vector< std::vector<double> > 
ConstraintsCheckerNaive::prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &objects, 
		double sf_x, double sf_y, double sf_yaw, char sf_foot){

	std::vector< std::vector<double> > v;
	std::vector< std::vector<double> >::const_iterator vit;
	std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
	uint c=0;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		double obj_x = (*oit)->g.x;
		double obj_y = (*oit)->g.y;

		//translate object, so that origin and sf origin conincide
		double tx = obj_x - sf_x;
		double ty = obj_y - sf_y;
		//rotate object around origin, such that object is aligned with
		//sf
		double rx = cos(sf_yaw)*tx + sin(sf_yaw)*ty;
		double ry = sin(sf_yaw)*tx - cos(sf_yaw)*ty;

		if(sf_foot == 'R'){
		}else{
			ry = -ry;
		}

		std::vector<double> d(4);
		//X,Y,R,H
		d.at(0)=rx;
		d.at(1)=ry;//(foot=='R'?-ry:ry);//if the support foot is the right one, we have to invert the object position (precomputation did only take place in the left foot space)
		d.at(2)=(*oit)->g.getRadius();
		d.at(3)=(*oit)->g.getHeight();

		double dist = sqrtf(rx*rx+ry*ry);
		DEBUGOBJ(ROS_INFO("object (%f,%f) -> (%f,%f) (angle:%f)", obj_x, obj_y, d.at(0), d.at(1), toDeg(sf_yaw));)
		if(dist<MAX_SWEPT_VOLUME_LIMIT){
			v.push_back(d);
		}
		//DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

	}
	return v;
}//prepare objects
void ConstraintsCheckerNaive::loadActionSpace(const char *path){
	char postfix[20];
	sprintf(postfix, "%d%s", 16, "neuron.net");
	bool collision=false;
	uint Nfiles = get_num_files_in_dir(path, postfix);
	DIR* dpath = opendir( path );
	if ( dpath ) 
	{
		struct dirent* hFile;
		errno = 0;
		uint number = 0;
		actionSpace.clear();
		while (( hFile = readdir( dpath )) != NULL ){
			if( !strcmp( hFile->d_name, "."  )) continue;
			if( !strcmp( hFile->d_name, ".." )) continue;
			if( hFile->d_name[0] == '.' ) continue; //hidden files

			if( strstr( hFile->d_name, postfix )){
				std::string file = hFile->d_name;
				std::vector<double> v = extract_num_from_string(file);
				v.at(0)/=100.0;
				v.at(1)/=100.0;
				v.at(2)=toRad(v.at(2));

				//compute hash from v (position of free foot)
				uint hash = hashit<double>(v);
				if(actionSpace.find(hash)!=actionSpace.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}

				actionSpace[hash] = v;
			}
		} 
		closedir( dpath );
	}
	if(collision){
		ROS_INFO("WARNING: collision in hashing actions");
		throw "collision hash";
		exit(-1);
	}
}
