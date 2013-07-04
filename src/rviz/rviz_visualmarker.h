#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <interactive_markers/interactive_marker_server.h>

#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <utility> //std::pair
#include <iostream>
#include <string>
#include <boost/thread.hpp>

#ifdef FCL_COLLISION_CHECKING
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#endif
#include <Eigen/Core>

#ifdef PQP_COLLISION_CHECKING
#include <pqp/PQP.h>
#endif

#include "util/util.h"

struct FCLInterface;

namespace ros{
	//predefined objects
	//
	typedef fcl::AABB BoundingVolume;
	//typedef fcl::OBBRSS BoundingVolume;
	struct TriangleObjectChair;
	//static const char *FRAME_NAME = "/mocap_world";
	static const char *FRAME_NAME = "/base_link";
	static const double ROS_DURATION = 0;

	class Geometry{
		double rx,ry,rz,rw; //quaternions
	public:
		Geometry();
		double x,y,z;
		double sx,sy,sz; //scale
		double radius,height;
		void print();
		double getQuaternionX();
		double getQuaternionY(); 
		double getQuaternionZ(); 
		double getQuaternionW(); 
		void setQuaternionX(double);
		void setQuaternionY(double); 
		void setQuaternionZ(double); 
		void setQuaternionW(double); 
		void setRPYRadian(double roll, double pitch, double yaw); //yaw, in radians of course 
		double getYawRadian(); //yaw, in radians of course 
		double getHeight();
		double getRadius();
	};

	struct Color{
		Color();
		Color(const Color&);
		Color(double r, double g, double b, double a=0.9);
		double r,g,b,a;
		void print(){
			ROS_INFO("COLOR -> %f %f %f %f", r,g,b,a);
		}
	};
	static Color DEFAULT(1.0,0.3,0.0,1.0);
	static Color RED(1.0,0.2,0.0,1.0);
	static Color BLUE(0.1,0.9,0.0,1.0);
	static Color DARKGREEN(0.3,0.7,0.0,1.0);
	static Color WHITE(1.0,1.0,1.0,1.0);
	static Color MAGENTA(0.9,0.0,0.9,1.0);

	//static Color TEXT_COLOR(0.9,0.9,0.9,1.0);
	static Color TEXT_COLOR(0.1,0.1,0.1,1.0);

	struct RVIZInterface{
	private:
		std::vector<visualization_msgs::Marker> marker_list;
	public:
		ros::Publisher publisher;
		ros::NodeHandle n;
		RVIZInterface();
		bool waitForSubscribers(ros::Duration timeout);
		void reset();
		void publish(visualization_msgs::Marker &m, bool save=false);
	};

	class RVIZVisualMarker{
	protected:
		static RVIZInterface *rviz; 
		static uint global_id;
		uint id;
		Color c;

		visualization_msgs::Marker marker;
		visualization_msgs::InteractiveMarker active_marker;
		static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

		std::string text;
		bool textHover;
		std::string geometry_subscribe_topic;

		boost::shared_ptr<boost::thread> m_thread;
		boost::shared_ptr<boost::thread> m_thread_interactive_marker;

		ros::Subscriber m_subscriber;
		bool changedPosition;
		void interactiveMarkerFeedbackLoop( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	private:
		//Threading for updates from ROS topics
		void Callback_updatePosition( const geometry_msgs::TransformStamped& tf);
		void thread_evart();
		void thread_start();
		void thread_stop();

		void init_marker_interactive();

		void thread_interactive_marker_main();
		void thread_interactive_marker_start();
		void thread_interactive_marker_stop();
	public:
		Geometry g;
		Geometry g_old;
		void setScale(double sx, double sy, double sz);
		void setXYZ(double x, double y, double z);
		void setXYT(double x, double y, double yaw_rad);
		void setRPYRadian(double roll, double pitch, double yaw);
		void make_interactive();

		void print();
		RVIZVisualMarker();
		virtual ~RVIZVisualMarker();
		bool isChanged(double threshold=0.01);
		void set_color(const Color& rhs);
		void set_color(double r, double g, double b, double a);
		Color get_color();
		void addText( std::string s );
		void subscribeToEvart(std::string &topic);
		void subscribeToEvart(const char *c);
		void drawLine(double x_in, double y_in);

		void reset();
		visualization_msgs::Marker createTextMarker();

		void init_marker();
		void init_marker(visualization_msgs::Marker &marker);
		void update_marker();
		void update_marker( visualization_msgs::Marker &marker );

		Geometry* getGeometry();

		//virtual methods
		virtual void publish();
		virtual std::string name() = 0;
		virtual uint32_t get_shape() = 0;

		virtual double getTextZ(){
			return g.z+g.sz/2+0.1;
		}
	};
	struct ColladaObject: public RVIZVisualMarker{
		std::string filename;
		ColladaObject(const char *cfilename);
		uint32_t get_shape();
		virtual std::string name();
	};

	struct CubeMarker: public RVIZVisualMarker{
		CubeMarker(double x, double y, double w=0.08, double yaw=0);
		virtual std::string name();
		virtual uint32_t get_shape();
	};
	struct SphereMarker: public RVIZVisualMarker{
		SphereMarker(double x, double y, double r=0.08, double z=0);
		virtual std::string name();
		virtual uint32_t get_shape();
	};

	struct Text: public RVIZVisualMarker{
		std::string text;
		Text(double x, double y, double z, char *c);
		virtual std::string name();
		virtual uint32_t get_shape();
	};

	struct TriangleObject: public RVIZVisualMarker{
	protected:
#ifdef FCL_COLLISION_CHECKING
		fcl::BVHModel< BoundingVolume > *bvh;
#endif
		std::string tris_file_name;
#ifdef PQP_COLLISION_CHECKING
		PQP_Model *pqp_model;
		PQP_Model *pqp_margin;
#endif
	public:
		TriangleObject();
		virtual ~TriangleObject();
		virtual std::string name();
		virtual uint32_t get_shape();
#ifdef PQP_COLLISION_CHECKING
		double pqp_distance_to(TriangleObject &rhs);
		void set_pqp_ptr( PQP_Model* pqp_in );
		PQP_Model* get_pqp_ptr();
#endif
#ifdef FCL_COLLISION_CHECKING
		void set_bvh_ptr( fcl::BVHModel< BoundingVolume > *bvh_in );
		fcl::BVHModel< BoundingVolume >* get_bvh_ptr();
#endif
		double distance_to(TriangleObject &rhs, Eigen::VectorXd &derivative);
		double fast_distance_to(TriangleObject &rhs);
		double gradient_distance_to(TriangleObject &rhs);

		void tris2marker(visualization_msgs::Marker &marker, const char *fname);

	};

	struct CylinderMarkerTriangles: public TriangleObject{
		CylinderMarkerTriangles(double x, double y, double r, double h);
		virtual std::string name();
		virtual uint32_t get_shape();
		virtual Color default_color();

		void cylinder2marker(visualization_msgs::Marker &marker, uint N, double radius, double height);
		void cylinder2BVH(fcl::BVHModel< BoundingVolume > *m, uint N, double radius, double height);
		void cylinder2PQP(PQP_Model *m, uint N, double radius, double height);
		std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > 
		getCylinderVerticesAndTriangles(uint N, double radius, double height);
		void reloadCylinderBVH(double radius, double height);
	};

	struct TrisTriangleObject: public TriangleObject{
		static uint mesh_counter;
	public:
		explicit TrisTriangleObject();
		TrisTriangleObject(const char *c, Geometry &in);
		TrisTriangleObject(std::string f, Geometry &in);
		TrisTriangleObject(const char *c);
		TrisTriangleObject(std::string f);

		void init_object( std::string f, Geometry &in );
		virtual std::string name();
		virtual uint32_t get_shape();
#ifdef FCL_COLLISION_CHECKING
		void tris2BVH(fcl::BVHModel< BoundingVolume > *m, const char *fname );
		void reloadCylinderBVH(double radius, double height);
#endif
#ifdef PQP_COLLISION_CHECKING
		void tris2PQP(PQP_Model *m, PQP_Model *m_margin, const char *fname );
		void tris2PQP(PQP_Model *m, const char *fname );
#endif

		void reloadBVH();
	};
	struct SweptVolumeObject: public TrisTriangleObject{
		SweptVolumeObject();
		SweptVolumeObject(std::string f, Geometry &in);
	};
	struct FootMarker: public RVIZVisualMarker{
		FootMarker(double x, double y, double yaw);
		virtual std::string name();
		virtual uint32_t get_shape();
		virtual void publish();
	};
	struct ColorFootMarker: public FootMarker{
		ColorFootMarker(double x, double y, double tz, const char *color);
		virtual std::string name();
	};
	struct LeftFootMarker: public FootMarker{
		LeftFootMarker(double x, double y, double tz);
		virtual std::string name();
	};
	struct RightFootMarker: public FootMarker{
		RightFootMarker(double x, double y, double tz);
		virtual std::string name();
	};
	struct TriangleObjectFloor: public TrisTriangleObject{
		TriangleObjectFloor(double x, double y, std::string fname, std::string mocap);
		TriangleObjectFloor(double x, double y, std::string fname);
	};
	struct TriangleObjectCylinder: public TrisTriangleObject{
		TriangleObjectCylinder(double x, double y);
	};
	struct TriangleObjectChair: public TrisTriangleObject{
		TriangleObjectChair(std::string mocap);
	};
	struct TriangleObjectRobot: public TrisTriangleObject{
		TriangleObjectRobot(std::string mocap);
	};
	//augments the triangle object by a mesh with the same size (TODO:
	//embedding into voronoi hull)
	struct TriangleMeshDecorator: public TriangleObject{
	private:
		TriangleObject *_tobj;
		visualization_msgs::Marker _deco_marker;
		std::string _deco_file;

		TriangleMeshDecorator();
	public:
		TriangleMeshDecorator(TriangleObject *, const char *);
		virtual void publish();
		uint32_t get_shape();
		virtual std::string name();
	};

	struct BlenderMeshTriangleObject: public TrisTriangleObject{
		std::string filename;
		BlenderMeshTriangleObject(const char *cfilename, const char *trisname, double x, double y, double tz);
		uint32_t get_shape();
		virtual double getTextZ();
	};

};//namespace ROS
