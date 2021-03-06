#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

namespace ros{
	Text::Text(double x, double y, double z, char *c): RVIZVisualMarker(){
		this->g.setX(x);
		this->g.setY(y);
		this->g.setZ(z);
		this->g.setRPYRadian(0,0,0);
		this->g.setSZ(0.1);
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
  void CommonMarker::publish(){
		marker.header.frame_id = FRAME_NAME;
		marker.lifetime = ros::Duration(ROS_DURATION);
		rviz->publish(marker, true);
		if(textHover){
			visualization_msgs::Marker cmarker = createTextMarker();
			rviz->publish(cmarker);
		}
  }
  ArrowMarker::ArrowMarker(double x, double y, double yaw){
    this->g.setX(x);
    this->g.setY(y);
    this->g.setZ(-0.2);
    this->g.setRPYRadian(0,0,yaw);
    double r = 0.5;
    this->g.setSX(0);
    this->g.setSY(r);
    this->g.setSZ(0.5);
    this->g.setZ(this->g.getZ() + this->g.getSZ()/2);
    set_color(ros::MAGENTA);
    init_marker();
  }
  std::string ArrowMarker::name(){
    return std::string("sphere");
  }
  uint32_t ArrowMarker::get_shape(){
    return visualization_msgs::Marker::ARROW;
  }
	SphereMarker::SphereMarker(double x, double y, double r, double z) {
		this->g.setX(x);
		this->g.setY(y);
		this->g.setZ(z);
		this->g.setRPYRadian(0,0,0);
		this->g.setSX(r);
		this->g.setSY(r);
		this->g.setSZ(0.05);
		this->g.setZ(this->g.getZ() + this->g.getSZ()/2);
		set_color(ros::MAGENTA);
		init_marker();
	}
	std::string SphereMarker::name(){
		return std::string("sphere");
	}
	uint32_t SphereMarker::get_shape(){
		return visualization_msgs::Marker::CYLINDER;
	}
	CuboidMarker::CuboidMarker(double x, double y, double l, double w, double h, double yaw) {
		this->g.setX(x);
		this->g.setY(y);
		this->g.setRPYRadian(0,0,yaw);
		this->g.setSX(l);
		this->g.setSY(w);
		this->g.setSZ(h);
		this->g.setZ(this->g.getZ() + this->g.getSZ()/2); //bottom of cube should be at desired z, not center
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
		return this->g.getSZ() + 0.10;
	}


}
