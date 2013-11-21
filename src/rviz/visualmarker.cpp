#include "rviz/visualmarker.h"

#define DEBUG(x)
#define THREAD_DEBUG(x) 
namespace ros{
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> RVIZVisualMarker::server_;
Color::Color(){
  r_=1;g_=0;b_=0;a_=1;
}
Color::Color(const Color& c){
  r_=c.r_;g_=c.g_;b_=c.b_;a_=c.a_;
}
Color::Color(double r, double g, double b, double a){
  this->r_ = r;this->g_=g;this->b_=b;this->a_=a;
}

RVIZVisualMarker::RVIZVisualMarker()
{
  id_=global_id_;
  textHover = false;
  global_id_++;
  if(rviz_ == NULL){
    rviz_ = new RVIZInterface();
  }
  changedPosition_ = false;
  const_offset_x_ = 0;
  const_offset_y_ = 0;
  const_offset_yaw_ = 0;

  evart_freeze_ = false;
  evart_it_before_freezing_ = 0;
  evart_it_current_ = 0;
  csf_in_chest_yaw_ = 0;
}
  
void RVIZVisualMarker::setScale(double sx, double sy, double sz){
  this->g_.sx_ = sx;
  this->g_.sy_ = sy;
  this->g_.sz_ = sz;
  update_marker();
}
void RVIZVisualMarker::setXYZ(double x, double y, double z){
  this->g_.x_ = x;
  this->g_.y_ = y;
  this->g_.z_ = z;
  update_marker();
}
void RVIZVisualMarker::setRPYRadian(double roll, double pitch, double yaw){
  this->g_.setRPYRadian(roll, pitch, yaw+csf_in_chest_yaw_);
  update_marker();
}

void RVIZVisualMarker::interactiveMarkerFeedbackLoop( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  THREAD_DEBUG(ROS_INFO_STREAM( feedback->marker_name << " is now at "
                                << feedback->pose.position.x << ", " << feedback->pose.position.y););
  
  //update geometry
  this->g_.x_ = feedback->pose.position.x;
  this->g_.y_ = feedback->pose.position.y;
  this->g_.z_ = feedback->pose.position.z;
  update_marker();
  boost::this_thread::interruption_point();
}

void RVIZVisualMarker::thread_interactive_marker_main(){

  std::string name_id = boost::lexical_cast<std::string>(m_thread_interactive_marker->get_id());
  THREAD_DEBUG(ROS_INFO("thread %s started", name_id.c_str()));
  while(1){
    boost::this_thread::interruption_point();
    ros::spinOnce();
  }
  THREAD_DEBUG(ROS_INFO("thread %s finished", name_id.c_str()));
}

void RVIZVisualMarker::thread_interactive_marker_start(){
  static bool server_init = false;
  if(!server_init){
    THREAD_DEBUG(ROS_INFO("init interactive marker server"));
    //server = new interactive_markers::InteractiveMarkerServer("interactive_markers");
    server_.reset( new interactive_markers::InteractiveMarkerServer("interactive_markers"));
    ros::Duration(0.1).sleep();
    server_init=true;
  }

  THREAD_DEBUG(ROS_INFO("inserting marker %s into server", active_marker.name.c_str()));
  server_->applyChanges();

  //class member function has to be first converted to the
  //callback object, which is defined in interactive marker,
  //afterwards we can insert it into the server without compiler
  //errors
  boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
      bind(&RVIZVisualMarker::interactiveMarkerFeedbackLoop, this, _1);

  server_->insert(active_marker_, bindedLoop );
  server_->applyChanges();

  //thread off
  m_thread_interactive_marker = boost::shared_ptr<boost::thread>(new boost::thread(&RVIZVisualMarker::thread_interactive_marker_main, this) );
}

void RVIZVisualMarker::thread_interactive_marker_stop(){
  if(this->m_thread_interactive_marker!=NULL){
    this->m_thread_interactive_marker->interrupt();
    std::string id = boost::lexical_cast<std::string>(this->m_thread_interactive_marker->get_id());
    THREAD_DEBUG(ROS_INFO("RVIZVisualMarker:: waiting for thread %s to terminate", id.c_str()));
    this->m_thread_interactive_marker->join();
  }
}

void RVIZVisualMarker::make_interactive(){
  make_interactive(0.1);
}
void RVIZVisualMarker::make_interactive(double interaction_radius){
  std::string imarker_name = this->name();
  imarker_name+=boost::lexical_cast<std::string>(this->id_);
  string_validate_chars( imarker_name );

  active_marker_.header.frame_id = FRAME_NAME;
  active_marker_.name = imarker_name.c_str();
  active_marker_.description = "Interactive";

  tf::Vector3 position(this->g_.x_, this->g_.y_, this->g_.z_);
  tf::pointTFToMsg(position, active_marker_.pose.position);
  active_marker_.scale = std::max(interaction_radius, g_.getRadius());//sqrtf(g.sx*g.sx + g.sy*g.sy);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.always_visible = true;
  control.name = "move_xy";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  active_marker_.controls.push_back(control);

  //control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  //control.orientation.w = 1;
  //control.orientation.x = 0;
  //control.orientation.y = 1;
  //control.orientation.z = 0;
  //control.name = "rotate_z";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //active_marker.controls.push_back(control);

  control.always_visible = true;
  control.markers.push_back( this->marker_ );
  active_marker_.controls.push_back(control);

  thread_interactive_marker_start();

}
void RVIZVisualMarker::init_marker_interactive(){
  char fname[50];
  std::string name = this->name();
  sprintf(fname, "%d_%s",this->id_, name.c_str());

  marker_.header.frame_id = FRAME_NAME;

  marker_.ns = fname;
  marker_.id = this->id_;
  marker_.type = get_shape();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = g_.x_;
  marker_.pose.position.y = g_.y_;
  marker_.pose.position.z = g_.z_;
  marker_.pose.orientation.x = g_.getQuaternionX();
  marker_.pose.orientation.y = g_.getQuaternionY();
  marker_.pose.orientation.z = g_.getQuaternionZ();
  marker_.pose.orientation.w = g_.getQuaternionW();

  marker_.scale.x = g_.sx_;
  marker_.scale.y = g_.sy_;
  marker_.scale.z = g_.sz_;

  Color c = get_color();
  marker_.color.r = c.r_;
  marker_.color.g = c.g_;
  marker_.color.b = c.b_;
  marker_.color.a = c.a_;

  g_old_ = g_;
}
RVIZVisualMarker::~RVIZVisualMarker(){
  thread_stop();
}
Color RVIZVisualMarker::get_color(){
  return c_;
}
void RVIZVisualMarker::set_color(const Color& rhs){
  this->c_ = rhs;
  update_marker();
}
void RVIZVisualMarker::set_color(double r, double g, double b, double a){
  this->c_.r_=r;
  this->c_.g_=g;
  this->c_.b_=b;
  this->c_.a_=a;
  update_marker();
}
void RVIZVisualMarker::addText(std::string s){
  textHover = true;
  text_ = s;
}
void RVIZVisualMarker::print(){
  g_.print();
}
void RVIZVisualMarker::setXYT(double x, double y, double yaw_rad){
  g_.x_ = x;
  g_.y_ = y;
  g_.setRPYRadian(0,0,yaw_rad+csf_in_chest_yaw_);
}
bool RVIZVisualMarker::isChanged(double threshold){
  if( dist(g_.x_, g_old_.x_, g_.y_, g_old_.y_) > threshold ){
    g_old_ = g_;
    return true;
  }
  return false;
}
void RVIZVisualMarker::publish(){
  marker_.header.frame_id = FRAME_NAME;
  marker_.lifetime = ros::Duration(ROS_DURATION);
  rviz_->publish(marker_,false);
  if(textHover){
    visualization_msgs::Marker cmarker = createTextMarker();
    rviz_->publish(cmarker);
  }
}
visualization_msgs::Marker RVIZVisualMarker::createTextMarker(){
  visualization_msgs::Marker cmarker;// = new Text( this->g_.x_, this->g_.y_, this->g_.z + 1, cc);
  //textMarker->publish();
  char fname[50];
  std::string name = this->name();
  sprintf(fname, "%d_%s_text",this->id_, name.c_str());

  cmarker.ns = fname;
  cmarker.id = this->id_;
  cmarker.type =  visualization_msgs::Marker::TEXT_VIEW_FACING;
  cmarker.action = visualization_msgs::Marker::ADD;
  cmarker.text = this->text_;
  cmarker.pose.position.x = g_.x_;
  cmarker.pose.position.y = g_.y_;
  cmarker.pose.position.z = this->getTextZ();
  cmarker.pose.orientation.x = 0.0;
  cmarker.pose.orientation.y = 0.0;
  cmarker.pose.orientation.z = g_.getQuaternionZ();
  cmarker.pose.orientation.w = 1.0;

  cmarker.scale.z = 0.15;

  Color c = ros::TEXT_COLOR;
  cmarker.color.r = c.r_;
  cmarker.color.g = c.g_;
  cmarker.color.b = c.b_;
  cmarker.color.a = c.a_;
  cmarker.header.frame_id = FRAME_NAME;
  cmarker.lifetime = ros::Duration(ROS_DURATION);
  return cmarker;

}
void RVIZVisualMarker::reset(){
  this->rviz_->reset();
  global_id_ = 0;
}
Geometry* RVIZVisualMarker::getGeometry(){
  return &g_;
}
void RVIZVisualMarker::init_marker(){
  init_marker(this->marker_);
}
void RVIZVisualMarker::init_marker(visualization_msgs::Marker &marker){
  char fname[50];
  std::string name = this->name();
  sprintf(fname, "%d_%s",this->id_, name.c_str());

  marker.header.frame_id = FRAME_NAME;
  marker.ns = fname;
  marker.id = this->id_;
  marker.type = get_shape();
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = g_.x_;
  marker.pose.position.y = g_.y_;
  marker.pose.position.z = g_.z_;
  marker.pose.orientation.x = g_.getQuaternionX();
  marker.pose.orientation.y = g_.getQuaternionY();
  marker.pose.orientation.z = g_.getQuaternionZ();
  marker.pose.orientation.w = g_.getQuaternionW();

  marker.scale.x = g_.sx_;
  marker.scale.y = g_.sy_;
  marker.scale.z = g_.sz_;

  Color c = get_color();
  marker.color.r = c.r_;
  marker.color.g = c.g_;
  marker.color.b = c.b_;
  marker.color.a = c.a_;

  g_old_ = g_;
}

void RVIZVisualMarker::update_marker(){
  update_marker(this->marker_);
}
void RVIZVisualMarker::update_marker( visualization_msgs::Marker &marker ){
  marker_.id = this->id_;
  marker_.type = get_shape();
  marker_.pose.position.x = g_.x_;
  marker_.pose.position.y = g_.y_;
  marker_.pose.position.z = g_.z_;
  marker_.pose.orientation.x = g_.getQuaternionX();
  marker_.pose.orientation.y = g_.getQuaternionY();
  marker_.pose.orientation.z = g_.getQuaternionZ();
  marker_.pose.orientation.w = g_.getQuaternionW();

  marker_.scale.x = g_.sx_;
  marker_.scale.y = g_.sy_;
  marker_.scale.z = g_.sz_;

  Color cc = get_color();
  std_msgs::ColorRGBA c;
  c.r = cc.r_;
  c.g = cc.g_;
  c.b = cc.b_;
  c.a = cc.a_;

  marker_.color.r=c.r;
  marker_.color.g=c.g;
  marker_.color.b=c.b;
  marker_.color.a=c.a;

  for(uint i=0;i<marker_.colors.size();i++){
    marker_.colors.at(i)=c;
  }

}

void RVIZVisualMarker::drawLine(double x_in, double y_in){

  visualization_msgs::Marker line;
  uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

  char fname[50];
  sprintf(fname, "%d_line",this->id_);
  global_id_++;

  line.ns = fname;
  line.id = this->id_;
  line.type = shape;
  line.action = visualization_msgs::Marker::ADD;

  line.scale.x = 0.01;
  geometry_msgs::Point p;
  p.x = g_.x_;
  p.y = g_.y_;
  p.z = g_.z_;

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
  this->rviz_->publish(line, true);
}
void RVIZVisualMarker::set_constant_offset( double x, double y){
  this->const_offset_x_ = x;
  this->const_offset_y_ = y;
}
void RVIZVisualMarker::set_constant_rotation_radian( double r, double p, double yaw){
  this->const_offset_yaw_ = yaw;
}
void RVIZVisualMarker::set_csf_in_chest_yaw(double csf_yaw)
{
  csf_in_chest_yaw_ = csf_yaw;
}

void RVIZVisualMarker::Callback_updatePosition( const geometry_msgs::TransformStamped& tf){
  if (evart_freeze_)
  {
    if (evart_it_before_freezing_<evart_it_current_)
      return;
    evart_it_current_++;
  }
            
  boost::mutex::scoped_lock lock(util_mutex);
  geometry_msgs::Transform t = tf.transform;
  std::string name_id = tf.child_frame_id;

  g_.x_ = t.translation.x + const_offset_x_;
  g_.y_ = t.translation.y + const_offset_y_;

  g_.x_ =  cos(const_offset_yaw_)*g_.x_ + sin(const_offset_yaw_)*g_.y_;
  g_.y_ = -sin(const_offset_yaw_)*g_.x_ + cos(const_offset_yaw_)*g_.y_;

  double qx = t.rotation.x;
  double qy = t.rotation.y;
  double qz = t.rotation.z;
  double qw = t.rotation.w;
  tf::Quaternion qin( qx, qy, qz, qw);
  double yaw = tf::getYaw(qin) + csf_in_chest_yaw_;
          
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  g_.setQuaternionX(q.getX());
  g_.setQuaternionY(q.getY());
  g_.setQuaternionZ(q.getZ());
  g_.setQuaternionW(q.getW());
          
  DEBUG(
      if( g_.x_ > 10000 || g_.y_ > 10000 ){
        std::cout << "[" << tf.child_frame_id << "] " << g_.x_ << " " << g_.y_ << " " << yaw << std::endl;
        std::cout << "[" << tf.child_frame_id << "] " << t.translation.x << " " << t.translation.y << " " << yaw << std::endl;
      }
        )

      update_marker();
  boost::this_thread::interruption_point();
}

void RVIZVisualMarker::thread_evart(){
  assert(m_subscriber.getNumPublishers()==0);
  //ros::Rate r(1); //Hz
  m_subscriber = rviz_->n.subscribe(geometry_subscribe_topic_.c_str(), 1000, &RVIZVisualMarker::Callback_updatePosition, this);
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
  m_thread = boost::shared_ptr<boost::thread>(new boost::thread(&RVIZVisualMarker::thread_evart, this) );
}
void RVIZVisualMarker::subscribeToEvart(const char *c, bool freeze, unsigned int it_before_freezing){
  std::string s(c);
  subscribeToEvart(s,freeze,it_before_freezing);
}
void RVIZVisualMarker::subscribeToEvart(std::string &topic, bool freeze, unsigned int it_before_freezing){
  evart_freeze_ = freeze;
  evart_it_before_freezing_ = it_before_freezing;
  evart_it_current_ = 0;

  geometry_subscribe_topic_ = topic;
  thread_start();
}
double RVIZVisualMarker::getTextZ(){
  return g_.z_+g_.sz_+0.1;
}
}
