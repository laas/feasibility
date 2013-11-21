#include "rviz/visualmarker.h"

#define DEBUG(x)
namespace ros{
RVIZInterface *RVIZVisualMarker::rviz_ = NULL;
uint RVIZVisualMarker::global_id_ = 0;

RVIZInterface::RVIZInterface(){
  std::string topic_name = "visualization_marker";
  ROS_INFO("started RVIZ interface");
  publisher = n.advertise<visualization_msgs::Marker>(topic_name.c_str(), 10000);
}

//bool save: stores the marker in an internal structure, such that we
//can reset it conveniently
void RVIZInterface::publish(visualization_msgs::Marker &m, bool save){
  if(save){
    marker_list.push_back(m);
  }
  DEBUG(ROS_INFO("added marker %s,%d", m.ns.c_str(),m.id););
      
  //time 0 means, that the marker will be displayed, regardless of
  //the internal time
  m.header.stamp = ros::Time();//ros::Time::now();
  publisher.publish(m);
}

bool RVIZInterface::waitForSubscribers(ros::Duration timeout)
{
  if(publisher.getNumSubscribers() > 0) return true;
  ros::Time start = ros::Time::now();
  ros::Rate waitTime(0.1);
  while(ros::Time::now() - start < timeout) {
    waitTime.sleep();
    if(publisher.getNumSubscribers() > 0) break;
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
