#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"
namespace ros{
	FootMarker::FootMarker(double x, double y, double yaw): RVIZVisualMarker(){
		this->g_.x_ = x;
		this->g_.y_ = y;
		this->g_.setRPYRadian(0,0,yaw);
		this->g_.sx_=0.24;
		this->g_.sy_=0.14;
		this->g_.sz_=0.03;
		set_color(0.9,0.9,0.9,0.8);
		init_marker();
	}
	std::string FootMarker::name(){
		return std::string("foot");
	}
	uint32_t FootMarker::get_shape(){
		return visualization_msgs::Marker::CUBE;
	}
	void FootMarker::publish(){
		marker_.header.frame_id = FRAME_NAME;
		marker_.lifetime = ros::Duration(ROS_DURATION);
		rviz_->publish(marker_, true);
		if(textHover){
			visualization_msgs::Marker cmarker = createTextMarker();
			rviz_->publish(cmarker);
		}
	}

	LeftFootMarker::LeftFootMarker(double x, double y, double tz): FootMarker(x,y,tz) {
		set_color(0.9,0.1,0.0,0.8);
		init_marker();
	}
	std::string LeftFootMarker::name(){
		return std::string("foot_L");
	}
	RightFootMarker::RightFootMarker(double x, double y, double tz): FootMarker(x,y,tz) {
		set_color(0.1,0.9,0.0,0.8);
		init_marker();
	}
	std::string RightFootMarker::name(){
		return std::string("foot_R");
	}
	ColorFootMarker::ColorFootMarker(double x, double y, double tz, const char *color): FootMarker(x,y,tz) {
		struct str2num {
			const char *str;
			double r,g,b,a;
		}registerMap[] = {
		    { "red"  , 1.0, 0.0, 0.0, 1.0 },
		    { "green", 0.0, 1.0, 0.0, 1.0 },
		    { "blue" , 0.0, 0.0, 1.0, 1.0 },
		    { "white" , 1.0, 1.0, 1.0, 1.0 },
		    { NULL , 0.0, 0.0, 0.0, 0.0 }
		};

		int i;
		for (i=0; registerMap[i].str != NULL; i++){
			if(strcmp(color, registerMap[i].str)==0){
				c_.r_=registerMap[i].r;
				c_.g_=registerMap[i].g;
				c_.b_=registerMap[i].b;
				c_.a_=registerMap[i].a;
			}
		}
		init_marker();
	}
	std::string ColorFootMarker::name(){
		return std::string("foot_colored");
	}
}
