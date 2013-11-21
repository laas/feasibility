#include "rviz/visualmarker.h"

#define DEBUG(x)
#define THREAD_DEBUG(x) 
namespace ros{
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> RVIZVisualMarker::server;
  Color::Color(){
    r=0.6;g=0;b=0.6;a=0.4;
  }
  Color::Color(const Color& c){
    r=c.r;g=c.g;b=c.b;a=c.a;
  }
  Color::Color(double r, double g, double b, double a){
    this->r = r;this->g=g;this->b=b;this->a=a;
  }

  RVIZVisualMarker::RVIZVisualMarker()
  {
    id=global_id;
    textHover = false;
    global_id++;
    if(rviz == NULL){
      rviz = new RVIZInterface();
    }
    changedPosition = false;
    const_offset_x = 0;
    const_offset_y = 0;
    const_offset_yaw = 0;

    evart_freeze_ = false;
    evart_it_before_freezing_ = 0;
    evart_it_current_ = 0;
    csf_in_chest_yaw_ = 0;
  }
  
  void RVIZVisualMarker::setScale(double sx, double sy, double sz){
    this->g.sx = sx;
    this->g.sy = sy;
    this->g.sz = sz;
    update_marker();
  }
  void RVIZVisualMarker::setXYZ(double x, double y, double z){
    this->g.x = x;
    this->g.y = y;
    this->g.z = z;
    update_marker();
  }
  void RVIZVisualMarker::setRPYRadian(double roll, double pitch, double yaw){
    this->g.setRPYRadian(roll, pitch, yaw+csf_in_chest_yaw_);
    update_marker();
  }

  void RVIZVisualMarker::interactiveMarkerFeedbackLoop( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
    THREAD_DEBUG(ROS_INFO_STREAM( feedback->marker_name << " is now at "
                                  << feedback->pose.position.x << ", " << feedback->pose.position.y);)

      //update geometry
      this->g.x = feedback->pose.position.x;
    this->g.y = feedback->pose.position.y;
    this->g.z = feedback->pose.position.z;
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
      server.reset( new interactive_markers::InteractiveMarkerServer("interactive_markers"));
      ros::Duration(0.1).sleep();
      server_init=true;
    }

    THREAD_DEBUG(ROS_INFO("inserting marker %s into server", active_marker.name.c_str()));
    server->applyChanges();

    //class member function has to be first converted to the
    //callback object, which is defined in interactive marker,
    //afterwards we can insert it into the server without compiler
    //errors
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
      bind(&RVIZVisualMarker::interactiveMarkerFeedbackLoop, this, _1);

    server->insert(active_marker, bindedLoop );
    server->applyChanges();

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
    imarker_name+=boost::lexical_cast<std::string>(this->id);
    string_validate_chars( imarker_name );

    active_marker.header.frame_id = FRAME_NAME;
    active_marker.name = imarker_name.c_str();
    active_marker.description = "Interactive";

    tf::Vector3 position(this->g.x, this->g.y, this->g.z);
    tf::pointTFToMsg(position, active_marker.pose.position);
    active_marker.scale = std::max(interaction_radius, g.getRadius());//sqrtf(g.sx*g.sx + g.sy*g.sy);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.always_visible = true;
    control.name = "move_xy";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    active_marker.controls.push_back(control);

    //control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    //control.orientation.w = 1;
    //control.orientation.x = 0;
    //control.orientation.y = 1;
    //control.orientation.z = 0;
    //control.name = "rotate_z";
    //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    //active_marker.controls.push_back(control);

    control.always_visible = true;
    control.markers.push_back( this->marker );
    active_marker.controls.push_back(control);

    thread_interactive_marker_start();

  }
  void RVIZVisualMarker::init_marker_interactive(){
    char fname[50];
    std::string name = this->name();
    sprintf(fname, "%d_%s",this->id, name.c_str());

    marker.header.frame_id = FRAME_NAME;

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
  RVIZVisualMarker::~RVIZVisualMarker(){
    thread_stop();
  }
  Color RVIZVisualMarker::get_color(){
    return c;
  }
  void RVIZVisualMarker::set_color(const Color& rhs){
    this->c = rhs;
    update_marker();
  }
  void RVIZVisualMarker::set_color(double r, double g, double b, double a){
    this->c.r=r;
    this->c.g=g;
    this->c.b=b;
    this->c.a=a;
    update_marker();
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
    g.setRPYRadian(0,0,yaw_rad+csf_in_chest_yaw_);
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
    rviz->publish(marker,false);
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
    init_marker(this->marker);
  }
  void RVIZVisualMarker::init_marker(visualization_msgs::Marker &marker){
    char fname[50];
    std::string name = this->name();
    sprintf(fname, "%d_%s",this->id, name.c_str());

    marker.header.frame_id = FRAME_NAME;
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
    update_marker(this->marker);
  }
  void RVIZVisualMarker::update_marker( visualization_msgs::Marker &marker ){
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

    line.scale.x = 0.01;
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
    this->rviz->publish(line, true);
  }
  void RVIZVisualMarker::set_constant_offset( double x, double y){
    this->const_offset_x = x;
    this->const_offset_y = y;
  }
  void RVIZVisualMarker::set_constant_rotation_radian( double r, double p, double yaw){
    this->const_offset_yaw = yaw;
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

    g.x = t.translation.x + const_offset_x;
    g.y = t.translation.y + const_offset_y;

    g.x =  cos(const_offset_yaw)*g.x + sin(const_offset_yaw)*g.y;
    g.y = -sin(const_offset_yaw)*g.x + cos(const_offset_yaw)*g.y;

    double qx = t.rotation.x;
    double qy = t.rotation.y;
    double qz = t.rotation.z;
    double qw = t.rotation.w;
    tf::Quaternion qin( qx, qy, qz, qw);
    double yaw = tf::getYaw(qin) + csf_in_chest_yaw_;
          
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    g.setQuaternionX(q.getX());
    g.setQuaternionY(q.getY());
    g.setQuaternionZ(q.getZ());
    g.setQuaternionW(q.getW());
          
    DEBUG(
          if( g.x > 10000 || g.y > 10000 ){
            std::cout << "[" << tf.child_frame_id << "] " << g.x << " " << g.y << " " << yaw << std::endl;
            std::cout << "[" << tf.child_frame_id << "] " << t.translation.x << " " << t.translation.y << " " << yaw << std::endl;
          }
          )

      update_marker();
    boost::this_thread::interruption_point();
  }

  void RVIZVisualMarker::thread_evart(){
    assert(m_subscriber.getNumPublishers()==0);
    //ros::Rate r(1); //Hz
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

    geometry_subscribe_topic = topic;
    thread_start();
  }
  double RVIZVisualMarker::getTextZ(){
    return g.z+g.sz+0.1;
  }
}
