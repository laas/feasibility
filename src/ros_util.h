#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
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
#include "util.h"

struct FCLInterface;

namespace ros{
	//predefined objects
	//
	struct TriangleObjectChair;

	const char *FRAME_NAME = "/mocap_world";
	static const double ROS_DURATION = 1;

	struct Geometry{
		Geometry(){
			x=0;y=0;z=0;
			tz=0;
			sx=1;sy=1;sz=1;
		}
		double x,y,z;
		double tx,ty,tz,tw; //quaternions
		double sx,sy,sz; //scale

		void print(){
			printf("X %f|Y %f|Z %f\n",x,y,z);
			printf("TX %f|TY %f|TZ %f\n",tx,ty,tz);
			printf("SX %f|SY %f|SZ %f\n",sx,sy,sz);
		}
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
	Color DEFAULT(1.0,0.3,0.0,0.9);
	Color RED(1.0,0.2,0.0,0.9);
	Color BLUE(0.1,0.9,0.0,0.9);
	Color MAGENTA(0.9,0.0,0.9,0.9);

	struct RVIZInterface{
	public:
		ros::Publisher publisher;
		ros::NodeHandle n;

	private:
		typedef std::pair< std::string, uint> MarkerIdentifier;
		typedef std::vector< MarkerIdentifier > MarkerIdentifierVector;
		MarkerIdentifierVector published_marker;

		std::vector<visualization_msgs::Marker> marker_list;

	public:
		RVIZInterface(){
			std::string topic_name = "visualization_marker";
			publisher = n.advertise<visualization_msgs::Marker>(topic_name.c_str(), 1);
		}
		void reset(){
			/*
			MarkerIdentifierVector::iterator it;
			printf("reset %d marker\n", published_marker.size());
			ROS_INFO("marker contains %d footsteps", published_marker.size());
			for(it = published_marker.begin(); it != published_marker.end(); it++){
				visualization_msgs::Marker tmp;
				MarkerIdentifier m = (*it);
				uint32_t shape = visualization_msgs::Marker::CUBE;
				tmp.type = shape;
				tmp.ns = m.first;
				tmp.id = m.second;
				tmp.color.r = 0.0f;
				tmp.color.g = 0.0f;
				tmp.color.b = 1.0f;
				tmp.color.a = 1.0;
				tmp.action = visualization_msgs::Marker::DELETE;
				tmp.header.frame_id = FRAME_NAME;
				tmp.header.stamp = ros::Time::now();
				//tmp.lifetime = ros::Duration();
				publisher.publish(tmp);
				ROS_INFO("deleted marker %s,%d", m.first.c_str(),m.second);
			}
			ROS_INFO("-------------------------------------");
			published_marker.clear();
			*/

			std::vector<visualization_msgs::Marker>::iterator it;
			for(it=marker_list.begin(); it!=marker_list.end(); it++){
				visualization_msgs::Marker tmp = *it;
				//tmp.header.stamp = ros::Time::now();
				ros::Duration d = ros::Duration(0.1);
				tmp.lifetime = d;
				tmp.color.r = 0.0f;
				tmp.color.g = 0.1f;
				tmp.color.b = 1.0f;
				tmp.color.a = 1.0;
				//tmp.action = visualization_msgs::Marker::ADD;
				publisher.publish(tmp);
			}
			marker_list.clear();

		}
		void publish(visualization_msgs::Marker &m){
			publisher.publish(m);
		}
		void footstep_publish(visualization_msgs::Marker &m){
			publisher.publish(m);
			marker_list.push_back(m);

			/*
			MarkerIdentifier cur_m;
			cur_m = std::make_pair( std::string(m.ns), m.id );
			this->published_marker.push_back(cur_m);

			ROS_INFO("created marker %s,%d", cur_m.first.c_str(),cur_m.second);
			*/
		}
		//void getTransformGeometry( Geometry &g );
	};

	class RVIZVisualMarker{
	protected:
		static uint global_id;
		uint id;
		Geometry g;
		Color c;
		static RVIZInterface *rviz; 
		visualization_msgs::Marker marker;

		std::string geometry_subscribe_topic;
		boost::shared_ptr<boost::thread> m_thread;
		ros::Subscriber m_subscriber;
	public:
		RVIZVisualMarker(){
			id=global_id;
			global_id++;
			if(rviz == NULL){
				rviz = new RVIZInterface();
			}
		}
		virtual void publish(){
			marker.header.frame_id = FRAME_NAME;
			marker.header.stamp = ros::Time::now();
			marker.lifetime = ros::Duration();
			rviz->publish(marker);
		}
		void reset(){
			this->rviz->reset();
		}
		virtual std::string name() = 0;
		virtual uint32_t get_shape() = 0;
		virtual Color get_color() = 0;
		void drawLine(double x_in, double y_in);
		void init_marker();


		// evart interface methods
		~RVIZVisualMarker(){
			if(m_thread!=NULL){
				m_thread->interrupt();
				std::string id = boost::lexical_cast<std::string>(m_thread->get_id());
				ROS_INFO("waiting for thread %s to terminate", id.c_str());
				m_thread->join();
			}
		}
	private:
		void Callback_updatePosition( const geometry_msgs::TransformStamped& tf){
			geometry_msgs::Transform t = tf.transform;
			std::string name_id = tf.child_frame_id;
			//ROS_INFO("updating geometry of %s", name_id.c_str());

			g.x = t.translation.x;
			g.y = t.translation.y;
			g.z = t.translation.z;
			g.tx = t.rotation.x;
			g.ty = t.rotation.y;
			g.tz = t.rotation.z;
			g.tw = t.rotation.w;

			update_marker();
			boost::this_thread::interruption_point();
		}

		void Callback_init(){
			assert(!m_subscriber);
			m_subscriber = rviz->n.subscribe(geometry_subscribe_topic.c_str(), 1000, &RVIZVisualMarker::Callback_updatePosition, this);
			std::string name_id = boost::lexical_cast<std::string>(m_thread->get_id());
			ROS_INFO("thread %s %d started", name_id.c_str());
			ros::spin();
			//ros::AsyncSpinner spinner(2);
			//spinner.start();
			ros::waitForShutdown();
			ROS_INFO("thread %s finished", name_id.c_str());
		}

		void update_marker();
	public:
		void subscribeToEvart(std::string &topic){
			//m_thread = boost::thread(Callback_init());
			geometry_subscribe_topic = topic;
			assert(!m_thread);
			m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&RVIZVisualMarker::Callback_init, this) );
		}
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
			marker.lifetime = ros::Duration();
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
		SphereMarker(double x, double y, double r=0.05): RVIZVisualMarker() {
			this->g.x = x;
			this->g.y = y;
			this->g.tz = 0;
			this->g.sx=r;
			this->g.sy=r;
			this->g.sz=0.01;
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


	struct TriangleObject: public RVIZVisualMarker{
		typedef fcl::AABB BoundingVolume;
		std::string tris_file_name;
		fcl::BVHModel< BoundingVolume > bvh;
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
			return ros::MAGENTA;
		}
		void read_tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname );
		void read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname);
		double distance_to(TriangleObject &rhs);

	};
	struct TriangleObjectChair: public TriangleObject{
		TriangleObjectChair(std::string mocap): TriangleObject(){
			std::string prefix = get_data_path();
			std::string chair_file = get_chair_str();
			Geometry chair_pos;
			chair_pos.x = 0.49;
			chair_pos.y = 0.05;
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
			robot_pos.x = 0;
			robot_pos.y = 0;
			robot_pos.tz = 0;

			this->init_object(robot_file, robot_pos);

			this->subscribeToEvart( mocap );
		}
	};

};//namespace ROS
