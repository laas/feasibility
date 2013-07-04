#include <string>
#include <vector>
#include <fast-replanning/fast-replanning-interface.hh>

#include "util/util.h"
int main (int argc, char *argv[])
{
	std::string path = get_data_path();
	using namespace fastreplanning;
	
	//fastReplanning *FR;
	FastReplanningInterface *planner = fastReplanningInterfaceFactory(path, argc, argv);
	planner->mainLoop();
}
