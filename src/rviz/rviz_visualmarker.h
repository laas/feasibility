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
		Geometry();
		double x,y,z;
		double tx,ty,tz,tw; //quaternions
		double sx,sy,sz; //scale
		void print();
	};

	struct Color{
		Color();
		Color(double r, double g, double b, double a=0.9);
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
		bool waitForSubscribers(ros::Duration timeout);
		void reset();
		void publish(visualization_msgs::Marker &m);
		void footstep_publish(visualization_msgs::Marker &m);
	};

	class RVIZVisualMarker{
	protected:
		static RVIZInterface *rviz; 
		static uint global_id;
		uint id;
		Color c;
		visualization_msgs::Marker marker;
		std::string text;
		bool textHover;
		std::string geometry_subscribe_topic;
		boost::shared_ptr<boost::thread> m_thread;
		ros::Subscriber m_subscriber;
		bool changedPosition;
	private:
		void Callback_updatePosition( const geometry_msgs::TransformStamped& tf);
		void Callback_init();
		void update_marker();
	public:
		Geometry g;
		Geometry g_old;
		RVIZVisualMarker();
		bool isChanged(double threshold=0.01);
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
		virtual double getTextZ(){
			return g.z+g.sz/2+0.1;
		}
	};


	struct SphereMarker: public RVIZVisualMarker{
		SphereMarker(double x, double y, double r=0.08, double z=0);
		virtual std::string name();
		uint32_t get_shape();
		virtual Color get_color();
	};

	struct Text: public RVIZVisualMarker{
		std::string text;
		Text(double x, double y, double z, char *c);
		virtual std::string name();
		uint32_t get_shape();
		virtual Color get_color();
	};

	struct TriangleObject: public RVIZVisualMarker{
		typedef fcl::AABB BoundingVolume;
		std::string tris_file_name;
		fcl::BVHModel< BoundingVolume > bvh;
		PQP_Model *pqp_model;
		PQP_Model *pqp_margin;
		static uint mesh_counter;
	public:
		explicit TriangleObject();
		TriangleObject(std::string f, Geometry &in);
		void init_object( std::string f, Geometry &in );
		virtual std::string name();
		uint32_t get_shape();
		virtual Color get_color();
		void read_tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname );
		void read_tris_to_PQP(PQP_Model *m, PQP_Model *m_margin, const char *fname );
		void read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname);
		double distance_to(TriangleObject &rhs);
	};
	struct FootMarker: public RVIZVisualMarker{
		FootMarker(double x, double y, double tz);
		virtual std::string name();
		uint32_t get_shape();
		virtual Color get_color();
		virtual void publish();
	};
	struct LeftFootMarker: public FootMarker{
		LeftFootMarker(double x, double y, double tz);
		virtual std::string name();
		virtual Color get_color();
	};
	struct RightFootMarker: public FootMarker{
		RightFootMarker(double x, double y, double tz);
		virtual std::string name();
		virtual Color get_color();
	};
	struct TriangleObjectFloor: public TriangleObject{
		TriangleObjectFloor(double x, double y, std::string fname, std::string mocap);
	};
	struct TriangleObjectChair: public TriangleObject{
		TriangleObjectChair(std::string mocap);
	};
	struct TriangleObjectRobot: public TriangleObject{
		TriangleObjectRobot(std::string mocap);
	};

	struct BlenderMeshTriangleObject: public TriangleObject{
		std::string filename;
		BlenderMeshTriangleObject(char *cfilename, char *trisname, double x, double y, double tz);
		uint32_t get_shape();
		virtual double getTextZ();
		virtual Color get_color();
	};

};//namespace ROS
