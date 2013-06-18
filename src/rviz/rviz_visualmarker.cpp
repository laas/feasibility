#include "rviz_visualmarker.h"

#define DEBUG(x) x
#define THREAD_DEBUG(x) x
namespace ros{
	RVIZVisualMarker::~RVIZVisualMarker(){
		thread_stop();
	}
	RVIZVisualMarker::RVIZVisualMarker(){
		id=global_id;
		textHover = false;
		global_id++;
		if(rviz == NULL){
			rviz = new RVIZInterface();
		}
		changedPosition = false;
	}
	void RVIZVisualMarker::addText(std::string s){
		textHover = true;
		text = s;
	}
	void RVIZVisualMarker::print(){
		g.print();
	}
	void RVIZVisualMarker::setXYT(double x, double y, double yaw_rad){
		g.x = x;
		g.y = y;
		g.setYawRadian(yaw_rad);
	}
	bool RVIZVisualMarker::isChanged(double threshold){
		if( dist(g.x, g_old.x, g.y, g_old.y) > threshold ){
			g_old = g;
			return true;
		}
		return false;
	}
	void RVIZVisualMarker::publish(){
		marker.header.frame_id = FRAME_NAME;
		marker.lifetime = ros::Duration(ROS_DURATION);
		rviz->publish(marker);
		if(textHover){
			visualization_msgs::Marker cmarker = createTextMarker();
			rviz->publish(cmarker);
		}
	}
	visualization_msgs::Marker RVIZVisualMarker::createTextMarker(){
		visualization_msgs::Marker cmarker;// = new Text( this->g.x, this->g.y, this->g.z + 1, cc);
		//textMarker->publish();
		char fname[50];
		std::string name = this->name();
		sprintf(fname, "%d_%s_text",this->id, name.c_str());

		cmarker.ns = fname;
		cmarker.id = this->id;
		cmarker.type =  visualization_msgs::Marker::TEXT_VIEW_FACING;
		cmarker.action = visualization_msgs::Marker::ADD;
		cmarker.text = this->text;
		cmarker.pose.position.x = g.x;
		cmarker.pose.position.y = g.y;
		cmarker.pose.position.z = this->getTextZ();
		cmarker.pose.orientation.x = 0.0;
		cmarker.pose.orientation.y = 0.0;
		cmarker.pose.orientation.z = g.getQuaternionZ();
		cmarker.pose.orientation.w = 1.0;

		cmarker.scale.z = 0.15;

		Color c = ros::TEXT_COLOR;
		cmarker.color.r = c.r;
		cmarker.color.g = c.g;
		cmarker.color.b = c.b;
		cmarker.color.a = c.a;
		cmarker.header.frame_id = FRAME_NAME;
		cmarker.lifetime = ros::Duration(ROS_DURATION);
		return cmarker;

	}
	void RVIZVisualMarker::reset(){
		this->rviz->reset();
		global_id = 0;
	}
	Geometry* RVIZVisualMarker::getGeometry(){
		return &g;
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
		marker.pose.orientation.x = g.getQuaternionX();
		marker.pose.orientation.y = g.getQuaternionY();
		marker.pose.orientation.z = g.getQuaternionZ();
		marker.pose.orientation.w = g.getQuaternionW();

		marker.scale.x = g.sx;
		marker.scale.y = g.sy;
		marker.scale.z = g.sz;

		Color c = get_color();
		marker.color.r = c.r;
		marker.color.g = c.g;
		marker.color.b = c.b;
		marker.color.a = c.a;

		g_old = g;
	}

	void RVIZVisualMarker::update_marker(){
		marker.id = this->id;
		marker.type = get_shape();
		marker.pose.position.x = g.x;
		marker.pose.position.y = g.y;
		marker.pose.position.z = g.z;
		marker.pose.orientation.x = g.getQuaternionX();
		marker.pose.orientation.y = g.getQuaternionY();
		marker.pose.orientation.z = g.getQuaternionZ();
		marker.pose.orientation.w = g.getQuaternionW();

		marker.scale.x = g.sx;
		marker.scale.y = g.sy;
		marker.scale.z = g.sz;

		Color cc = get_color();
		std_msgs::ColorRGBA c;
		c.r = cc.r;
		c.g = cc.g;
		c.b = cc.b;
		c.a = cc.a;

		marker.color.r=c.r;
		marker.color.g=c.g;
		marker.color.b=c.b;
		marker.color.a=c.a;

		for(uint i=0;i<marker.colors.size();i++){
			marker.colors.at(i)=c;
		}

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
		g.setQuaternionX(q.getX());
		g.setQuaternionY(q.getY());
		g.setQuaternionZ(q.getZ());
		g.setQuaternionW(q.getW());

		update_marker();
		boost::this_thread::interruption_point();
	}

	void RVIZVisualMarker::thread_evart(){
		assert(!m_subscriber);
		ros::Rate r(10); //Hz
		m_subscriber = rviz->n.subscribe(geometry_subscribe_topic.c_str(), 1000, &RVIZVisualMarker::Callback_updatePosition, this);
		std::string name_id = boost::lexical_cast<std::string>(m_thread->get_id());
		THREAD_DEBUG(ROS_INFO("thread %s subscribed to topic %s", name_id.c_str(), geometry_subscribe_topic.c_str()));

		while(1){
			boost::this_thread::interruption_point();
			ros::spinOnce();
		}

		THREAD_DEBUG(ROS_INFO("thread %s finished", name_id.c_str()));
	}
	void RVIZVisualMarker::thread_stop(){
		if(this->m_thread!=NULL){
			this->m_thread->interrupt();
			std::string id = boost::lexical_cast<std::string>(this->m_thread->get_id());
			THREAD_DEBUG(ROS_INFO("RVIZVisualMarker:: waiting for thread %s to terminate", id.c_str()));
			this->m_thread->join();
		}
	}
	void RVIZVisualMarker::thread_start(){
		//assert(!m_thread);
		m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&RVIZVisualMarker::thread_evart, this) );
	}
	void RVIZVisualMarker::subscribeToEvart(const char *c){
		std::string s(c);
		subscribeToEvart(s);
	}
	void RVIZVisualMarker::subscribeToEvart(std::string &topic){
		geometry_subscribe_topic = topic;
		thread_start();
	}
}
