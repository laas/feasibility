#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

namespace ros{

	ColladaObject::ColladaObject(const char *cfilename): RVIZVisualMarker(){
		filename = std::string(cfilename);

		this->g.setX(0);
		this->g.setY(0);
		this->g.setSX(1.0);
		this->g.setSY(1.0);
		this->g.setSZ(1.0);
		set_color(0,0,0,0);//texture is not shown, if any color!=0

		init_marker();
		marker.mesh_resource = filename;
		marker.mesh_use_embedded_materials=true;
	}
	uint32_t ColladaObject::get_shape(){
		return visualization_msgs::Marker::MESH_RESOURCE;
	}
	std::string ColladaObject::name(){
		return filename;
	}
}
