#include "planner/constraints_checker.h"
#include <dirent.h>

ConstraintsChecker::ConstraintsChecker(){
  std::string pkg_path = get_data_path(std::string("feasibility"));
  std::string svfn = pkg_path + "/model/fullBodyApprox/";
  this->loadSweptVolumeFileNames(svfn.c_str());
}
void ConstraintsChecker::loadSweptVolumeFileNames(const char *path){
	bool collision=false;
	//uint Nfiles = get_num_files_in_dir(path, ".tris");
	DIR* dpath = opendir( path );
	if ( dpath ) 
	{
		struct dirent* hFile;
		errno = 0;
		//uint number = 0;
		while (( hFile = readdir( dpath )) != NULL ){
			//if(number++>2) break;
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
				if(sweptvolumes_file_names.find(hash)!=sweptvolumes_file_names.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}
				std::string sv = path;
				sv+=file;
				sweptvolumes_file_names[hash] = file;
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
