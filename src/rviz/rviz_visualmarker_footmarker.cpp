#include "rviz_visualmarker.h"
namespace ros{
	FootMarker::FootMarker(double x, double y, double yaw): RVIZVisualMarker(){
		this->g.x = x;
		this->g.y = y;
		this->g.setYawRadian(yaw);
		this->g.sx=0.18;
		this->g.sy=0.09;
		this->g.sz=0.02;
		init_marker();
	}
	std::string FootMarker::name(){
		return std::string("foot");
	}
	uint32_t FootMarker::get_shape(){
		return visualization_msgs::Marker::CUBE;
	}
	Color FootMarker::get_color(){
		return Color(0.9,0.9,0.9,0.8);
	}
	void FootMarker::publish(){
		marker.header.frame_id = FRAME_NAME;
		marker.lifetime = ros::Duration();
		rviz->footstep_publish(marker);
		if(textHover){
			visualization_msgs::Marker cmarker = createTextMarker();
			rviz->publish(cmarker);
		}
	}

	LeftFootMarker::LeftFootMarker(double x, double y, double tz): FootMarker(x,y,tz) {
		init_marker();
	}
	std::string LeftFootMarker::name(){
		return std::string("foot_L");
	}
	Color LeftFootMarker::get_color(){
		return Color(0.9,0.1,0.0,0.8);
	}
	RightFootMarker::RightFootMarker(double x, double y, double tz): FootMarker(x,y,tz) {
		init_marker();
	}
	std::string RightFootMarker::name(){
		return std::string("foot_R");
	}
	Color RightFootMarker::get_color(){
		return Color(0.1,0.9,0.0,0.8);
	}
}
