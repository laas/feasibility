#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

namespace ros{

	ColladaObject::ColladaObject(const char *cfilename): RVIZVisualMarker(){
		filename_ = std::string(cfilename);

		this->g_.x_ = 0;
		this->g_.y_ = 0;
		this->g_.sx_ = 1.0;
		this->g_.sy_ = 1.0;
		this->g_.sz_ = 1.0;
		set_color(0,0,0,0);//texture is not shown, if any color!=0

		init_marker();
		marker_.mesh_resource = filename_;
		marker_.mesh_use_embedded_materials=true;
	}
	uint32_t ColladaObject::get_shape(){
		return visualization_msgs::Marker::MESH_RESOURCE;
	}
	std::string ColladaObject::name(){
		return filename_;
	}
}
