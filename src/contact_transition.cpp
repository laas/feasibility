#include "contact_transition.h"
static void ContactTransition::loadHyperPlaneParameters(const char *file){

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
		std::vector<double> pos(4);
		for(uint i=4;i<7;i++) pos.push_back(vv.at(k).at(i));
		uint hash = hashit<double>(pos);
		ROS_INFO("hash: %d", hash);
		if(hyperplane.find(hash)!=hyperplane.end()){
			ROS_INFO("hash collision: %d", hash);
			collision=true;
		}
		//hyperplane[hash] = params;
	}
	if(collision){
		ROS_INFO("WARNING: collision in hyperplane");
		throw "collision in hyperplane error";
		exit(-1);
	}
}
