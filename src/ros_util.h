#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <utility> //std::pair
#include <iostream>
#include <string>
#include <boost/thread.hpp>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

#include <pqp/PQP.h>
#include "util.h"

struct FCLInterface;

namespace ros{
	//predefined objects
	//
	struct TriangleObjectChair;

	static const char *FRAME_NAME = "/mocap_world";
	static const double ROS_DURATION = 0;

	struct Geometry{
		Geometry(){
			x=0;y=0;z=0;
			tx=0;ty=0;tz=0;tw=1;
			sx=1;sy=1;sz=1;
		}
		double x,y,z;
		double tx,ty,tz,tw; //quaternions
		double sx,sy,sz; //scale
		void print();
	};

	struct Color{
		public:
		Color(){
			r=0;g=0;b=0;a=1;
		}
		Color(double r, double g, double b, double a=0.9){
			this->r = r;this->g=g;this->b=b;this->a=a;
		}
		double r,g,b,a;
	};
	static Color DEFAULT(1.0,0.3,0.0,1.0);
	static Color RED(1.0,0.2,0.0,1.0);
	static Color BLUE(0.1,0.9,0.0,1.0);
	static Color DARKGREEN(0.3,0.7,0.0,1.0);
	static Color WHITE(1.0,1.0,1.0,1.0);
	static Color MAGENTA(0.9,0.0,0.9,1.0);

	static Color TEXT_COLOR(0.9,0.9,0.9,1.0);

	struct RVIZInterface{
	private:
		std::vector<visualization_msgs::Marker> marker_list;
	public:
		ros::Publisher publisher;
		ros::NodeHandle n;
		RVIZInterface();
		bool waitForSubscribers(ros::Publisher &pub, ros::Duration timeout);
		void reset();
		void publish(visualization_msgs::Marker &m);
		void footstep_publish(visualization_msgs::Marker &m);
	};

	class RVIZVisualMarker{
	protected:
		static uint global_id;
		uint id;
		Color c;
		static RVIZInterface *rviz; 
		visualization_msgs::Marker marker;
		//RVIZVisualMarker *textMarker;
		bool textHover;

		std::string geometry_subscribe_topic;
		boost::shared_ptr<boost::thread> m_thread;
		ros::Subscriber m_subscriber;

	private:
		void Callback_updatePosition( const geometry_msgs::TransformStamped& tf);
		void Callback_init();
		void update_marker();
	public:
		Geometry g;
		RVIZVisualMarker();
		virtual void publish();
		void reset();
		virtual std::string name() = 0;
		virtual uint32_t get_shape() = 0;
		virtual Color get_color() = 0;
		void drawLine(double x_in, double y_in);
		void init_marker();
		Geometry* getGeometry();
		~RVIZVisualMarker();
		void addText( char *c );
		void subscribeToEvart(std::string &topic);
		void subscribeToEvart(char *c);
	};

	class FootMarker: public RVIZVisualMarker{

	public:
		FootMarker(double x, double y, double tz): RVIZVisualMarker(){
			this->g.x = x;
			this->g.y = y;
			this->g.tz = tz;
			this->g.sx=0.18;
			this->g.sy=0.09;
			this->g.sz=0.02;
			init_marker();
		}
		virtual std::string name(){
			return string("foot");
		}
		uint32_t get_shape(){
			return visualization_msgs::Marker::CUBE;
		}
		virtual Color get_color(){
			return Color(0.9,0.9,0.9,0.8);
		}
		virtual void publish(){
			marker.header.frame_id = FRAME_NAME;
			marker.header.stamp = ros::Time::now();
			marker.lifetime = ros::Duration(ROS_DURATION);
			rviz->footstep_publish(marker);
		}
	};

	class LeftFootMarker: public FootMarker{
	public:
		LeftFootMarker(double x, double y, double tz): FootMarker(x,y,tz) {
			init_marker();
		}
		virtual std::string name(){
			return string("foot_L");
		}
		virtual Color get_color(){
			return Color(0.9,0.1,0.0,0.8);
		}
	};

	class RightFootMarker: public FootMarker{
	public:
		RightFootMarker(double x, double y, double tz): FootMarker(x,y,tz) {
			init_marker();
		}
		virtual std::string name(){
			return string("foot_R");
		}
		virtual Color get_color(){
			return Color(0.1,0.9,0.0,0.8);
		}
	};

	struct SphereMarker: public RVIZVisualMarker{
	public:
		SphereMarker(double x, double y, double r=0.08, double z=0): RVIZVisualMarker() {
			this->g.x = x;
			this->g.y = y;
			this->g.z = z;
			this->g.tz = 0;
			this->g.sx=r;
			this->g.sy=r;
			this->g.sz=0.05;
			init_marker();
		}
		virtual std::string name(){
			return string("sphere");
		}
		uint32_t get_shape(){
			return visualization_msgs::Marker::CYLINDER;
		}
		virtual Color get_color(){
			return ros::MAGENTA;
		}
	};

	struct Text: public RVIZVisualMarker{
		std::string text;
		public:
		Text(double x, double y, double z, char *c): RVIZVisualMarker(){
			this->g.x = x;
			this->g.y = y;
			this->g.z = z;
			this->g.tz = 0;
			this->g.sz=0.1;
			init_marker();
			marker.text = string(c);
			text = marker.text;
		}
		virtual std::string name(){
			return text;
		}
		uint32_t get_shape(){
			return visualization_msgs::Marker::TEXT_VIEW_FACING;
		}
		virtual Color get_color(){
			return ros::TEXT_COLOR;
		}


	};

	struct TriangleObject: public RVIZVisualMarker{
		typedef fcl::AABB BoundingVolume;
		std::string tris_file_name;
		fcl::BVHModel< BoundingVolume > bvh;
		PQP_Model *pqp_model;
		PQP_Model *pqp_margin;

		static uint mesh_counter;
	public:
		explicit TriangleObject(): RVIZVisualMarker(){
		}
		TriangleObject(std::string f, Geometry &in): RVIZVisualMarker() {
			init_object(f, in);
		}
		void init_object( std::string f, Geometry &in ){
			this->g.x = in.x;
			this->g.y = in.y;
			this->g.z = in.z;
			this->g.tz = in.tz;

			double scale = 0.7;
			this->g.sx = scale;
			this->g.sy = scale;
			this->g.sz = scale;
			this->tris_file_name=f;

			this->pqp_model = new PQP_Model;
			this->pqp_margin = new PQP_Model;
			this->read_tris_to_PQP( this->pqp_model, this->pqp_margin, tris_file_name.c_str() );

			this->read_tris_to_marker( this->marker, tris_file_name.c_str() );
			this->read_tris_to_BVH(this->bvh, tris_file_name.c_str() );
			init_marker();

		}
		virtual std::string name(){
			return string(basename(tris_file_name.c_str()));
		}
		uint32_t get_shape(){
			return visualization_msgs::Marker::TRIANGLE_LIST;
		}
		virtual Color get_color(){
			return ros::WHITE;
		}
		void read_tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname );
		void read_tris_to_PQP(PQP_Model *m, PQP_Model *m_margin, const char *fname );
		void read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname);


		double distance_to(TriangleObject &rhs);

	};
	struct TriangleObjectFloor: public TriangleObject{
		TriangleObjectFloor(double x, double y, std::string fname, std::string mocap): TriangleObject(){
			std::string prefix = get_data_path();
			std::string object_file = get_tris_str(fname);

			Geometry object_pos;
			object_pos.x = x;
			object_pos.y = y;

			this->init_object(object_file, object_pos);
			this->subscribeToEvart( mocap );
		}
	};
	struct TriangleObjectChair: public TriangleObject{
		TriangleObjectChair(std::string mocap): TriangleObject(){
			std::string prefix = get_data_path();
			std::string chair_file = get_chair_str();
			Geometry chair_pos;
			chair_pos.x = 0.49;
			chair_pos.y = -0.1;
			chair_pos.z = 0.0;
			chair_pos.tz = 0.0;
			this->init_object(chair_file, chair_pos);
			this->subscribeToEvart( mocap );
		}
	};
	struct TriangleObjectRobot: public TriangleObject{
		TriangleObjectRobot(std::string mocap): TriangleObject(){
			std::string prefix = get_data_path();
			std::string robot_file = get_robot_str();
			Geometry robot_pos;
			robot_pos.x = -2;
			robot_pos.y = 0;
			robot_pos.tz = 0;
			this->init_object(robot_file, robot_pos);
			this->subscribeToEvart( mocap );
		}
	};

};//namespace ROS
