#include "ros_util.h"

namespace ros{
	RVIZVisualMarker::RVIZVisualMarker(){
		id=global_id;
		textHover = false;
		global_id++;
		if(rviz == NULL){
			rviz = new RVIZInterface();
		}
	}
	void RVIZVisualMarker::addText(char *cc){
		textHover = true;
		//textMarker = new Text( this->g.x, this->g.y, this->g.z + 1, cc);
	}
	void RVIZVisualMarker::publish(){
		marker.header.frame_id = FRAME_NAME;
		marker.header.stamp = ros::Time::now();
		marker.lifetime = ros::Duration(ROS_DURATION);
		rviz->publish(marker);

		if(textHover){
			//textMarker->publish();
		}
		//ROS_INFO("published marker %s", marker.ns.c_str());
	}
	void RVIZVisualMarker::reset(){
		this->rviz->reset();
		global_id = 0;
	}
	Geometry* RVIZVisualMarker::getGeometry(){
		return &g;
	}
	RVIZVisualMarker::~RVIZVisualMarker(){
		if(m_thread!=NULL){
			m_thread->interrupt();
			std::string id = boost::lexical_cast<std::string>(m_thread->get_id());
			ROS_INFO("waiting for thread %s to terminate", id.c_str());
			m_thread->join();
		}
	}
	void RVIZVisualMarker::init_marker(){
		char fname[50];
		std::string name = this->name();
		sprintf(fname, "%d_%s",this->id, name.c_str());

		marker.ns = fname;
		marker.id = this->id;
		marker.type = get_shape();
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = g.x;
		marker.pose.position.y = g.y;
		marker.pose.position.z = g.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = g.tz;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = g.sx;
		marker.scale.y = g.sy;
		marker.scale.z = g.sz;

		Color c = get_color();
		marker.color.r = c.r;
		marker.color.g = c.g;
		marker.color.b = c.b;
		marker.color.a = c.a;
		if(textHover){
		}
	}

	void RVIZVisualMarker::update_marker(){
		marker.id = this->id;
		marker.type = get_shape();
		marker.pose.position.x = g.x;
		marker.pose.position.y = g.y;
		marker.pose.position.z = g.z;
		marker.pose.orientation.x = g.tx;
		marker.pose.orientation.y = g.ty;
		marker.pose.orientation.z = g.tz;
		marker.pose.orientation.w = g.tw;

		marker.scale.x = g.sx;
		marker.scale.y = g.sy;
		marker.scale.z = g.sz;

		Color c = get_color();
		marker.color.r = c.r;
		marker.color.g = c.g;
		marker.color.b = c.b;
		marker.color.a = c.a;
	}

	void RVIZVisualMarker::drawLine(double x_in, double y_in){

		visualization_msgs::Marker line;
		uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

		char fname[50];
		sprintf(fname, "%d_line",this->id);
		global_id++;

		line.ns = fname;
		line.id = this->id;
		line.type = shape;
		line.action = visualization_msgs::Marker::ADD;

		line.scale.x = 0.005;
		geometry_msgs::Point p;
		p.x = g.x;
		p.y = g.y;
		p.z = g.z;

		geometry_msgs::Point p2;
		p2.x = x_in;
		p2.y = y_in;
		p2.z = 0.0;

		line.points.push_back(p);
		line.points.push_back(p2);

		line.color.r = 1.0f;
		line.color.g = 0.5f;
		line.color.b = 0.0f;
		line.color.a = 1.0f;

		line.header.frame_id = FRAME_NAME;
		line.header.stamp = ros::Time::now();
		line.lifetime = ros::Duration();
		this->rviz->publish(line);
	}
	void RVIZVisualMarker::Callback_updatePosition( const geometry_msgs::TransformStamped& tf){
		geometry_msgs::Transform t = tf.transform;
		std::string name_id = tf.child_frame_id;

		g.x = t.translation.x;
		g.y = t.translation.y;

		double qx = t.rotation.x;
		double qy = t.rotation.y;
		double qz = t.rotation.z;
		double qw = t.rotation.w;
		tf::Quaternion qin( qx, qy, qz, qw);
		double yaw = tf::getYaw(qin);

		tf::Quaternion q;
		q.setRPY(0, 0, yaw);
		g.tx = q.getX();
		g.ty = q.getY();
		g.tz = q.getZ();
		g.tw = q.getW();

		update_marker();
		boost::this_thread::interruption_point();
	}

	void RVIZVisualMarker::Callback_init(){
		assert(!m_subscriber);
		m_subscriber = rviz->n.subscribe(geometry_subscribe_topic.c_str(), 1000, &RVIZVisualMarker::Callback_updatePosition, this);
		std::string name_id = boost::lexical_cast<std::string>(m_thread->get_id());
		ROS_INFO("thread %s subscribed to topic %s", name_id.c_str(), geometry_subscribe_topic.c_str());
		ros::spin();
		//ros::AsyncSpinner spinner(2);
		//spinner.start();
		ros::waitForShutdown();
		ROS_INFO("thread %s finished", name_id.c_str());
	}
	void RVIZVisualMarker::subscribeToEvart(char *c){
		std::string s(c);
		subscribeToEvart(s);
	}
	void RVIZVisualMarker::subscribeToEvart(std::string &topic){
		//m_thread = boost::thread(Callback_init());
		geometry_subscribe_topic = topic;
		assert(!m_thread);
		m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&RVIZVisualMarker::Callback_init, this) );
	}
}
