#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

#define DEBUG(x) x
namespace ros{

	TriangleMeshDecorator::TriangleMeshDecorator(TriangleObject *t, const char *deco_name){
		//this->deco_file_ = get_tris_str(deco_name);
		this->tobj_ = t;
		//this->tobj_->tris2marker( this->deco_marker_, this->deco_file_.c_str() );
		//this->tobj_->init_marker(this->deco_marker_);
	};
	void TriangleMeshDecorator::publish(){
		RVIZVisualMarker::publish();

		//this->tobj_->update_marker(this->deco_marker_);
		//deco_marker_.header.frame_id = FRAME_NAME;
		//deco_marker_.lifetime = ros::Duration(ROS_DURATION);
		//DEBUG(ROS_INFO("publishing deco marker");)
		//this->tobj_->rviz->publish(deco_marker_);
	}
	std::string TriangleMeshDecorator::name(){
		return tobj_->name();
	}
	uint32_t TriangleMeshDecorator::get_shape(){
		//return visualization_msgs::Marker::TRIANGLE_LIST;
		return tobj_->get_shape();
	}
}
