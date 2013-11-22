#include "planner/constraints_checker_swept_volumer.h"
#include <dirent.h>

#define DEBUG(x)
#define LIGHT_DEBUG(x)
ConstraintsCheckerSweptVolume::ConstraintsCheckerSweptVolume(){
  std::string pkg_path = get_data_path(std::string("feasibility"));
  std::string svfn = pkg_path + "/model/fullBodyApprox/";
  
  this->loadSweptVolumesToHashMap(svfn.c_str());
}

bool ConstraintsCheckerSweptVolume::isFeasible(  
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	double continuous_feasibility = this->computeSVOutput( p, obj);
	if(continuous_feasibility<=0){
		return false;
	}else{
		return true;
	}
}
ros::SweptVolumeObject* ConstraintsCheckerSweptVolume::get_sv_from_hash( uint hash ){
	return sweptvolumeMap.find(hash)->second;
}
double ConstraintsCheckerSweptVolume::computeSVOutput( 
		const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){
	//ignore obj

	DEBUG( ROS_INFO("pos %f %f %f\n", p.at(0), p.at(1), p.at(2));)
	uint hash = hashit<double>(p);
	ros::TriangleObject *sv = sweptvolumeMap.find(hash)->second;

	std::vector<double> cost_per_object;
	std::vector<ros::TriangleObject*>::iterator oit;

	for(  oit = objects_.begin(); oit != objects_.end(); ++oit ){
		//static_cast<ros::TriangleObject&>(**oit);
		cost_per_object.push_back( sv->pqp_distance_to(**oit) );
		//cost_per_object.push_back( sv->fast_distance_to(static_cast<ros::TriangleObject&>(**oit)) );
	}
	if(cost_per_object.empty()){
		return 1.0; //a positive number means, that it is feasible
	}
	double min = *min_element(cost_per_object.begin(), cost_per_object.end());
	return min;

}

std::vector< std::vector<double> > 
ConstraintsCheckerSweptVolume::prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
		double sf_x, double sf_y, double sf_yaw, char sf_foot){
	std::vector<std::vector<double> > v;
	std::vector<ros::RVIZVisualMarker*>::iterator oit;
	objects_.clear();
	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		double x = (*oit)->g.x;
		double y = (*oit)->g.y;
		double yaw = (*oit)->g.getYawRadian();

		//translate object, so that origin and sf origin conincide
		double tx = x - sf_x;
		double ty = y - sf_y;
		////rotate object around origin, such that object is aligned with
		////sf
		////sf
		double rx = cos(sf_yaw)*tx + sin(sf_yaw)*ty;
		double ry = -sin(sf_yaw)*tx + cos(sf_yaw)*ty;
		//double ryaw = yaw - sf_yaw;

		//while(ryaw>M_PI) ryaw-=2*M_PI;
		//while(ryaw<-M_PI) ryaw+=2*M_PI;

		//ATTENTION: no 'deep' copy is created, all pointers are
		//copied 'as is'. this is exactly what we intend to do:
		//we only want to change the position of the object
		//according to its geometry. this will be used in the
		//distance function to compute the neccessary
		//transformation

		ros::RVIZVisualMarker *t = *oit;
		ros::TriangleObject *o = new ros::SweptVolumeObject(); //ligthweight object, such that we can only copy pointer
		o->g = t->g;
		//o->set_bvh_ptr( t->get_bvh_ptr() );
		o->set_pqp_ptr( static_cast<ros::TriangleObject*>(t)->get_pqp_ptr() );

		o->g.x = rx;
		o->g.y = ry;
		objects_.push_back(o);
	}
	return v;
}//prepare objects
void ConstraintsCheckerSweptVolume::loadSweptVolumesToHashMap(const char *path){
//struct fann *ann = fann_create_from_file(argv[1]);
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
			LIGHT_DEBUG(if(number++>20) break;)
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
				if(sweptvolumeMap.find(hash)!=sweptvolumeMap.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}
				if(actionSpace.find(hash)!=actionSpace.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}

				std::string rel_file_path = path;
				rel_file_path += file.c_str();

				ros::Geometry g;
				g.x = v.at(0);
				g.y = v.at(1);
				g.z = 0.0;
				g.setRPYRadian(0,0, v.at(2) );
				ros::SweptVolumeObject* sv = new ros::SweptVolumeObject(rel_file_path.c_str(), g);

				sweptvolumeMap[hash] = sv;
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

