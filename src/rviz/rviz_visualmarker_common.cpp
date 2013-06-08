#include "rviz_visualmarker.h"

namespace ros{
	Text::Text(double x, double y, double z, char *c): RVIZVisualMarker(){
		this->g.x = x;
		this->g.y = y;
		this->g.z = z;
		this->g.setYawRadian(0);
		this->g.sz=0.1;
		init_marker();
		marker.text = std::string(c);
		text = marker.text;
	}
	std::string Text::name(){
		return text;
	}
	uint32_t Text::get_shape(){
		return visualization_msgs::Marker::TEXT_VIEW_FACING;
	}
	Color Text::get_color(){
		return ros::TEXT_COLOR;
	}
	SphereMarker::SphereMarker(double x, double y, double r, double z): RVIZVisualMarker() {
		this->g.x = x;
		this->g.y = y;
		this->g.z = z;
		this->g.setYawRadian(0);
		this->g.sx=r;
		this->g.sy=r;
		this->g.sz=0.05;
		this->g.z = this->g.z + this->g.sz/2;
		init_marker();
	}
	std::string SphereMarker::name(){
		return std::string("sphere");
	}
	uint32_t SphereMarker::get_shape(){
		return visualization_msgs::Marker::CYLINDER;
	}
	Color SphereMarker::get_color(){
		return ros::MAGENTA;
	}
	CubeMarker::CubeMarker(double x, double y, double r, double yaw): RVIZVisualMarker() {
		this->g.x = x;
		this->g.y = y;
		this->g.setYawRadian(yaw);
		this->g.sx=r;
		this->g.sy=r;
		this->g.sz=r;
		this->g.z = this->g.z + this->g.sz/2; //bottom of cube should be at desired z, not center
		init_marker();
	}
	std::string CubeMarker::name(){
		return std::string("cuberizer");
	}
	uint32_t CubeMarker::get_shape(){
		return visualization_msgs::Marker::CUBE;
	}
	Color CubeMarker::get_color(){
		return ros::MAGENTA;
	}


}
