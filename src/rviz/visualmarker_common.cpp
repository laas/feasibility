#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

namespace ros{
	Text::Text(double x, double y, double z, char *c): RVIZVisualMarker(){
		this->g_.x_ = x;
		this->g_.y_ = y;
		this->g_.z_ = z;
		this->g_.setRPYRadian(0,0,0);
		this->g_.sz_=0.1;
		set_color(ros::TEXT_COLOR);

		init_marker();
		marker_.text = std::string(c);
		text_ = marker_.text;
	}
	std::string Text::name(){
		return text_;
	}
	uint32_t Text::get_shape(){
		return visualization_msgs::Marker::TEXT_VIEW_FACING;
	}
	SphereMarker::SphereMarker(double x, double y, double r, double z): RVIZVisualMarker() {
		this->g_.x_ = x;
		this->g_.y_ = y;
		this->g_.z_ = z;
		this->g_.setRPYRadian(0,0,0);
		this->g_.sx_=r;
		this->g_.sy_=r;
		this->g_.sz_=0.05;
		this->g_.z_ = this->g_.z_ + this->g_.sz_/2;
		set_color(ros::MAGENTA);
		init_marker();
	}
	std::string SphereMarker::name(){
		return std::string("sphere");
	}
	uint32_t SphereMarker::get_shape(){
		return visualization_msgs::Marker::CYLINDER;
	}
	CuboidMarker::CuboidMarker(double x, double y, double l, double w, double h, double yaw): RVIZVisualMarker() {
		this->g_.x_ = x;
		this->g_.y_ = y;
		this->g_.setRPYRadian(0,0,yaw);
		this->g_.sx_=l;
		this->g_.sy_=w;
		this->g_.sz_=h;
		this->g_.z_ = this->g_.z_ + this->g_.sz_/2; //bottom of cube should be at desired z, not center
		set_color(ros::MAGENTA);
		init_marker();
	}
	std::string CuboidMarker::name(){
		return std::string("cuberizer");
	}
	uint32_t CuboidMarker::get_shape(){
		return visualization_msgs::Marker::CUBE;
	}
	double CuboidMarker::getTextZ(){
		return this->g_.sz_ + 0.10;
	}


}
