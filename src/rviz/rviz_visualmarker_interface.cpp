#include "rviz/rviz_visualmarker.h"

namespace ros{
	uint TriangleObject::mesh_counter=0;
	RVIZInterface *RVIZVisualMarker::rviz = NULL;
	uint RVIZVisualMarker::global_id = 0;

	//snippet from
	//http://answers.ros.org/question/40223/placing-permanent-visual-marker-in-rviz/?answer=40230#post-id-40230
	Color::Color(){
		r=0;g=0;b=0;a=1;
	}
	Color::Color(double r, double g, double b, double a){
		this->r = r;this->g=g;this->b=b;this->a=a;
	}
	Geometry::Geometry(){
		x=0;y=0;z=0;
		tx=0;ty=0;tz=0;tw=1;
		sx=1;sy=1;sz=1;
	}
	void Geometry::print(){
		printf("X %f|Y %f|Z %f\n",x,y,z);
		printf("TX %f|TY %f|TZ %f|TW %f\n",tx,ty,tz,tw);
		printf("SX %f|SY %f|SZ %f\n",sx,sy,sz);
		std::cout << std::endl;
	}
	RVIZInterface::RVIZInterface(){
		std::string topic_name = "visualization_marker";
		publisher = n.advertise<visualization_msgs::Marker>(topic_name.c_str(), 1000);
	}
	void RVIZInterface::publish(visualization_msgs::Marker &m){
		publisher.publish(m);
	}

	void RVIZInterface::footstep_publish(visualization_msgs::Marker &m){
		marker_list.push_back(m);
		this->publish(m);
		//ROS_INFO("added marker %s,%d", m.ns.c_str(),m.id);
	}
	bool RVIZInterface::waitForSubscribers(ros::Duration timeout)
	{
	    if(publisher.getNumSubscribers() > 0)
		return true;
	    ros::Time start = ros::Time::now();
	    ros::Rate waitTime(0.1);
	    while(ros::Time::now() - start < timeout) {
		    ROS_INFO("wait for subscribers...");
		waitTime.sleep();
		if(publisher.getNumSubscribers() > 0)
		    break;
	    }
	    return publisher.getNumSubscribers() > 0;
	}

	void RVIZInterface::reset(){

		std::vector<visualization_msgs::Marker>::iterator it;
		for(it=marker_list.begin(); it!=marker_list.end(); it++){
			visualization_msgs::Marker tmp = *it;
			tmp.header.stamp = ros::Time::now();
			//ROS_INFO("delete marker %s,%d", tmp.ns.c_str(), tmp.id);
			ros::Duration d = ros::Duration(1);
			tmp.lifetime = d;
			tmp.color.r = 0.0f;
			tmp.color.g = 0.1f;
			tmp.color.b = 1.0f;
			tmp.color.a = 1.0;
			tmp.action = visualization_msgs::Marker::DELETE;
			publisher.publish(tmp);
		}
		marker_list.clear();
	}


};
