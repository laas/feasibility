#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "ros_util.h"

namespace ros{
	uint TriangleObject::mesh_counter=0;
	RVIZInterface *RVIZVisualMarker::rviz = NULL;
	uint RVIZVisualMarker::global_id = 0;

	//snippet from
	//http://answers.ros.org/question/40223/placing-permanent-visual-marker-in-rviz/?answer=40230#post-id-40230
	void Geometry::print(){
		printf("X %f|Y %f|Z %f\n",x,y,z);
		printf("TX %f|TY %f|TZ %f|TW %f\n",tx,ty,tz,tw);
		printf("SX %f|SY %f|SZ %f\n",sx,sy,sz);
		cout << endl;
	}
	RVIZInterface::RVIZInterface(){
		std::string topic_name = "visualization_marker";
		publisher = n.advertise<visualization_msgs::Marker>(topic_name.c_str(), 1);
	}
	void RVIZInterface::publish(visualization_msgs::Marker &m){
		waitForSubscribers(publisher, ros::Duration(5));
		publisher.publish(m);
	}

	void RVIZInterface::footstep_publish(visualization_msgs::Marker &m){
		marker_list.push_back(m);
		this->publish(m);
		ROS_INFO("added marker %s,%d", m.ns.c_str(),m.id);
	}
	bool RVIZInterface::waitForSubscribers(ros::Publisher & pub, ros::Duration timeout)
	{
	    if(pub.getNumSubscribers() > 0)
		return true;
	    ros::Time start = ros::Time::now();
	    ros::Rate waitTime(0.2);
	    while(ros::Time::now() - start < timeout) {
		waitTime.sleep();
		if(pub.getNumSubscribers() > 0)
		    break;
	    }
	    return pub.getNumSubscribers() > 0;
	}

	void RVIZInterface::reset(){
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
			tmp.header.stamp = ros::Time::now();
			ROS_INFO("delete marker %s,%d", tmp.ns.c_str(), tmp.id);
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
	RVIZVisualMarker::RVIZVisualMarker(){
		id=global_id;
		global_id++;
		if(rviz == NULL){
			rviz = new RVIZInterface();
		}
	}
	void RVIZVisualMarker::publish(){
		marker.header.frame_id = FRAME_NAME;
		marker.header.stamp = ros::Time::now();
		marker.lifetime = ros::Duration(ROS_DURATION);
		rviz->publish(marker);
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

	double TriangleObject::distance_to(TriangleObject &rhs){
		//rotation z-axis (as visualized in rviz)
		
		double t = g.tz;
		double tr = rhs.g.tz;
		fcl::Matrix3f r1 (cos(t),-sin(t),0,
				  sin(t),cos(t) ,0,
				  0     ,0      ,1);
		fcl::Matrix3f r2 (cos(tr),-sin(tr),0,
				  sin(tr),cos(tr) ,0,
				  0     ,0      ,1);

		fcl::Vec3f d1(g.x,g.y,g.z);
		fcl::Vec3f d2(rhs.g.x,rhs.g.y,rhs.g.z);

		fcl::Transform3f Tlhs(r1, d1);
		fcl::Transform3f Trhs(r2, d2);

		fcl::DistanceRequest request;
		fcl::DistanceResult result;
		double d = fcl::distance (&this->bvh, Tlhs, &rhs.bvh, Trhs, request, result);
		//double md = result.penetration_depth;
		ROS_INFO("distance: %f", d);
		//result.clear();

		return d;
	}
	void TriangleObject::read_tris_to_PQP(PQP_Model *m, PQP_Model *m_margin, const char *fname ){
		int ntris;

		FILE *fp = fopen_s(fname,"r");
		int res=fscanf(fp, "%d", &ntris);
		CHECK(res==1, "fscanf failed");
		m->BeginModel();
		m_margin->BeginModel();

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		double scale = 1.0; //bigger obstacles for planning phase
		double scale_x = 1.1; //bigger obstacles for planning phase
		double scale_y = 1.1; //bigger obstacles for planning phase
		double scale_z = 1.1; //bigger obstacles for planning phase
		for (int i = 0; i < ntris; i++){
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			CHECK(res==9, "fscanf failed");

			PQP_REAL p1[3],p2[3],p3[3],p4[3],p5[3],p6[3];
			p1[0] = (PQP_REAL)scale*p1x; p1[1] = (PQP_REAL)scale*p1y; p1[2] = (PQP_REAL)scale*p1z;
			p2[0] = (PQP_REAL)scale*p2x; p2[1] = (PQP_REAL)scale*p2y; p2[2] = (PQP_REAL)scale*p2z;
			p3[0] = (PQP_REAL)scale*p3x; p3[1] = (PQP_REAL)scale*p3y; p3[2] = (PQP_REAL)scale*p3z;
			m->AddTri(p1,p2,p3,i);
			p4[0] = (PQP_REAL)scale_x*p1x; p4[1] = (PQP_REAL)scale_y*p1y; p4[2] = (PQP_REAL)scale_z*p1z;
			p5[0] = (PQP_REAL)scale_x*p2x; p5[1] = (PQP_REAL)scale_y*p2y; p5[2] = (PQP_REAL)scale_z*p2z;
			p6[0] = (PQP_REAL)scale_x*p3x; p6[1] = (PQP_REAL)scale_y*p3y; p6[2] = (PQP_REAL)scale_z*p3z;
			m_margin->AddTri(p4,p5,p6,i);
			
		}
		m->EndModel();
		m_margin->EndModel();
		fclose(fp);

		ROS_INFO("[%s] created PQP object with %d triangles.\n", name().c_str(), m->num_tris);

	}
	void TriangleObject::read_tris_to_BVH(fcl::BVHModel< BoundingVolume > &m, const char *fname ){
		
		int ntris;
		FILE *fp = fopen_s(fname,"r");
		int res=fscanf(fp, "%d", &ntris);
		CHECK(res==1, "fscanf failed");

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		for (int i = 0; i < 3*ntris; i+=3){
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			CHECK(res==9, "fscanf failed");
			
			fcl::Vec3f a(p1x, p1y, p1z);
			fcl::Vec3f b(p2x, p2y, p2z);
			fcl::Vec3f c(p3x, p3y, p3z);
			vertices.push_back(a);
			vertices.push_back(b);
			vertices.push_back(c);

			fcl::Triangle t(i,i+1,i+2);
			triangles.push_back(t);
		}
		fclose(fp);

		bvh.bv_splitter.reset (new fcl::BVSplitter<BoundingVolume>(fcl::SPLIT_METHOD_MEAN));
		bvh.beginModel();
		bvh.addSubModel(vertices, triangles);
		bvh.endModel();
		ROS_INFO("created object in FCL with %d triangles and %d vertices.\n", bvh.num_tris, bvh.num_vertices);
	}

	void TriangleObject::read_tris_to_marker(visualization_msgs::Marker &marker, const char *fname){
		
		int ntris;
		FILE *fp = fopen_s(fname,"r");

		int res=fscanf(fp, "%d", &ntris);
		CHECK(res==1, "fscanf failed");


		Color cc = get_color();
		std_msgs::ColorRGBA c;
		c.r = cc.r;
		c.g = cc.g;
		c.b = cc.b;
		c.a = cc.a;

		for (int i = 0; i < 3*ntris; i+=3){
			geometry_msgs::Point p;
			geometry_msgs::Point p1;
			geometry_msgs::Point p2;
			double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
			res=fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			       &p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
			
			double sX = 1.0;
			double sY = 1.0;
			double sZ = 1.0;
			p.x = sX*p1x;p.y = sY*p1y;p.z = sZ*p1z;
			p1.x = sX*p2x;p1.y = sY*p2y;p1.z = sZ*p2z;
			p2.x = sX*p3x;p2.y = sY*p3y;p2.z = sZ*p3z;
			marker.points.push_back(p);
			marker.points.push_back(p1);
			marker.points.push_back(p2);
			marker.colors.push_back(c);
			marker.colors.push_back(c);
			marker.colors.push_back(c);
		}
		fclose(fp);
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
};
