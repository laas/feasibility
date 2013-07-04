#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

#define DEBUG(x) x
namespace ros{

	TriangleMeshDecorator::TriangleMeshDecorator(TriangleObject *t, const char *deco_name){
		//this->_deco_file = get_tris_str(deco_name);
		this->_tobj = t;
		//this->_tobj->tris2marker( this->_deco_marker, this->_deco_file.c_str() );
		//this->_tobj->init_marker(this->_deco_marker);
	};
	void TriangleMeshDecorator::publish(){
		RVIZVisualMarker::publish();

		//this->_tobj->update_marker(this->_deco_marker);
		//_deco_marker.header.frame_id = FRAME_NAME;
		//_deco_marker.lifetime = ros::Duration(ROS_DURATION);
		//DEBUG(ROS_INFO("publishing deco marker");)
		//this->_tobj->rviz->publish(_deco_marker);
	}
	std::string TriangleMeshDecorator::name(){
		return _tobj->name();
	}
	uint32_t TriangleMeshDecorator::get_shape(){
		//return visualization_msgs::Marker::TRIANGLE_LIST;
		return _tobj->get_shape();
	}
}
