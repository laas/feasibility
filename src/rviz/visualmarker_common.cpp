#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

namespace ros{
	Text::Text(double x, double y, double z, char *c): RVIZVisualMarker(){
		this->g.x = x;
		this->g.y = y;
		this->g.z = z;
		this->g.setRPYRadian(0,0,0);
		this->g.sz=0.1;
		set_color(ros::TEXT_COLOR);

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
	SphereMarker::SphereMarker(double x, double y, double r, double z): RVIZVisualMarker() {
		this->g.x = x;
		this->g.y = y;
		this->g.z = z;
		this->g.setRPYRadian(0,0,0);
		this->g.sx=r;
		this->g.sy=r;
		this->g.sz=0.05;
		this->g.z = this->g.z + this->g.sz/2;
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
		this->g.x = x;
		this->g.y = y;
		this->g.setRPYRadian(0,0,yaw);
		this->g.sx=l;
		this->g.sy=w;
		this->g.sz=h;
		this->g.z = this->g.z + this->g.sz/2; //bottom of cube should be at desired z, not center
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
		return this->g.sz + 0.10;
	}


}
