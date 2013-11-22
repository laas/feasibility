#include <dirent.h>
#include <algorithm> //max
#include "planner/constraints_checker_naive.h"

#define DEBUG(x)
#define DEBUGOBJ(x) 
ConstraintsCheckerNaive::ConstraintsCheckerNaive(){
  std::string pkg_path = get_data_path(std::string("feasibility"));
  std::string svfn = pkg_path + "/model/fullBodyApprox/";
  this->loadActionSpace(svfn.c_str());
}

bool ConstraintsCheckerNaive::isFeasible(  
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	std::vector<std::vector<double> > v;
	double xr=p.at(0);
	double yr=p.at(1);
  double robo_cylinder = 0.4;
	std::vector< std::vector<double> >::const_iterator oit;
	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
    double xo = (*oit).at(0);
    double yo = (*oit).at(1);
    double ro = (*oit).at(2);
    double dist_robo_obj = dist(xo,xr, yo, yr);
    if(dist_robo_obj <= ro + robo_cylinder){
      //ROS_INFO("[WARNING] robo=( %f , %f ), obj=( %f, %f [%f]), dist=%f", xr, yr, xo, yo, ro, dist_robo_obj);
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
		//double rx = cos(sf_yaw)*tx + sin(sf_yaw)*ty;
		//double ry = -sin(sf_yaw)*tx + cos(sf_yaw)*ty;
		double rx = tx;
		double ry = ty;

		std::vector<double> d(3);
		//X,Y,R,H
    ros::PrimitiveMarkerBox *box = static_cast<ros::PrimitiveMarkerBox*>(*oit);
		d.at(0)=tx;
		d.at(1)=ty;
		//d.at(2)=(*oit)->g.getRadius();
		//d.at(3)=(*oit)->g.getHeight();
		d.at(2) = std::max(box->w,box->l);

		DEBUGOBJ(ROS_INFO("object (%f,%f) -> (%f,%f) (radius:%f)", obj_x, obj_y, d.at(0), d.at(1), d.at(2));)
    v.push_back(d);

	}
	return v;
}//prepare objects
void ConstraintsCheckerNaive::loadActionSpace(const char *path){
	bool collision=false;
	uint Nfiles = get_num_files_in_dir(path, ".tris");
	ROS_INFO("Opening %d files", Nfiles);
	DIR* dpath = opendir( path );
	if ( dpath ) 
	{
		struct dirent* hFile;
		errno = 0;
		uint number = 0;
		while (( hFile = readdir( dpath )) != NULL ){
			if( !strcmp( hFile->d_name, "."  )) continue;
			if( !strcmp( hFile->d_name, ".." )) continue;
			if( hFile->d_name[0] == '.' ) continue; //hidden files

			if( strstr( hFile->d_name, ".tris" )){
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

				std::string rel_file_path = path;
				rel_file_path += file.c_str();


				actionSpace[hash] = v;
				printf("[%d/%d] loaded %s\n", number++, Nfiles, hFile->d_name);
			}
		} 
		ROS_INFO("Loaded Swept Volumes");
		closedir( dpath );
	}
	if(collision){
		ROS_INFO("WARNING: collision in hashing actions");
		throw "collision hash";
		exit(-1);
	}
}
