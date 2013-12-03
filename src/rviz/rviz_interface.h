#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>

namespace ros{
	struct RVIZInterface{
	private:
		std::vector<visualization_msgs::Marker> marker_list;
    boost::mutex marker_mutex_;
	public:
		ros::Publisher publisher;
		ros::NodeHandle n;
		RVIZInterface();
		bool waitForSubscribers(ros::Duration timeout);
		void reset();
		void publish(visualization_msgs::Marker &m, bool save=false);
	};
}

