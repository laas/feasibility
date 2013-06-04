#include "constraints_checker_ann.h"

#define DEBUG(x) 
Logger logger;
bool ConstraintsCheckerANN::isFeasible(  const std::vector<double> &p, 
		const std::vector< std::vector<double> > &obj){

	double continuous_feasibility = this->computeNNOutput( p, obj);
	logger("%f %f %f %f\n", p.at(0), p.at(1), p.at(2), continuous_feasibility);
	if(continuous_feasibility<=0){
		return false;
	}else{
		return true;
	}
}

// support foot (SF)
//get object position in the coordinate system of the SF -- and prune objects
//which are too far away
std::vector< std::vector<double> > 
ConstraintsCheckerANN::prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &objects, 
		double sf_x, double sf_y, double sf_yaw, char foot){

	std::vector< std::vector<double> > v;
	std::vector< std::vector<double> >::const_iterator vit;
	std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
	uint c=0;
	for(  oit = objects.begin(); oit != objects.end(); ++oit ){
		double x = (*oit)->g.x;
		double y = (*oit)->g.y;

		//translate object, so that origin and sf origin conincide
		double tx = x - sf_x;
		double ty = y - sf_y;
		//rotate object around origin, such that object is aligned with
		//sf
		double rx = cos(sf_yaw)*tx - sin(sf_yaw)*ty;
		double ry = sin(sf_yaw)*tx + cos(sf_yaw)*ty;

		std::vector<double> d(4);
		//X,Y,R,H
		d.at(0)=rx;
		d.at(1)=ry;
		d.at(2)=(*oit)->g.getRadius();
		d.at(3)=(*oit)->g.getHeight();

		if(foot=='R'){
			ry=-ry;//if the support foot is the right one, we have to invert the object position (precomputation did only take place in the left foot space)
		}
		//prune objects, which are far away
		double dist = sqrtf(rx*rx+ry*ry);
		DEBUG(ROS_INFO("object %d/%d at %f %f (dist %f)", c++, objects.size(), rx, ry, dist);)
		if(dist<1.5){
			v.push_back(d);
		}
		//DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

	}
	return v;
}//prepare objects

//compute maximum distance to objects given the approximated norm
double ConstraintsCheckerANN::computeNNOutput(  const std::vector<double> &p, 
						const std::vector< std::vector<double> > &obj){


	DEBUG( ROS_INFO("pos %f %f %f\n", p.at(0), p.at(1), p.at(2));)
	uint hash = hashit<double>(p);
	struct fann *ann = neuralMap.find(hash)->second;

	std::vector<double> cost_per_object;
	std::vector< std::vector<double> >::const_iterator oit;

	for(  oit = obj.begin(); oit != obj.end(); ++oit ){
		//fann_type *input = &(*oit)[0];
		fann_type in[4];
		in[0]=(*oit).at(0);
		in[1]=(*oit).at(1);
		in[2]=(*oit).at(2);
		in[3]=(*oit).at(3);
		fann_run(ann, in);//(float*)&(*oit)[0]); //use doublefann.h input for double
		fann_type *output = ann->output;
		cost_per_object.push_back( output[0] );
		DEBUG( ROS_INFO("pos %f %f %f obj %f %f %f %f | %f\n", p.at(0), p.at(1), p.at(2), (*oit).at(0),(*oit).at(1),(*oit).at(2),(*oit).at(3),output[0]);)
	}
	if(cost_per_object.empty()){
		return 1.0; //a positive number means, that it is feasible
	}
	double min = *min_element(cost_per_object.begin(), cost_per_object.end());
	DEBUG( ROS_INFO("return %f", min);)
	return min;

}
void ConstraintsCheckerANN::loadNNParameters(const char *path){
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
				std::string file = hFile->d_name;
				std::vector<double> v = extract_num_from_string(file);
				v.at(0)/=100.0;
				v.at(1)/=100.0;
				v.at(2)/=100.0;

				//compute hash from v (position of free foot)
				uint hash = hashit<double>(v);
				if(neuralMap.find(hash)!=neuralMap.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}
				if(actionSpace.find(hash)!=actionSpace.end()){
					ROS_INFO("hash collision: %d", hash);
					collision=true;
				}

				//load d_name into fann structure
				std::string rel_file_path = path;
				rel_file_path += file.c_str();

				struct fann *ann = fann_create_from_file(rel_file_path.c_str());
				neuralMap[hash] = ann;
				actionSpace[hash] = v;
				printf("ANN %s, action %f %f %f, rel_path %s \n", hFile->d_name, v.at(0), v.at(1), v.at(2), rel_file_path.c_str());
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

ConstraintsCheckerANN::~ConstraintsCheckerANN(){
	neuralMap.clear();
}
ConstraintsCheckerANN::ConstraintsCheckerANN(){
	this->loadNNParameters("extern/fann/datasets/humanoids/");
}

