#include "rviz/rviz_visualmarker.h"

namespace ros{

	ColladaObject::ColladaObject(const char *cfilename): RVIZVisualMarker(){
		filename = std::string(cfilename);

		this->g.x = 0;
		this->g.y = 0;
		this->g.sx = 1.0;
		this->g.sy = 1.0;
		this->g.sz = 1.0;
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
