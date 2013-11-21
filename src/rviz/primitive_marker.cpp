#include "rviz/visualmarker.h"

namespace ros{
	uint32_t PrimitiveMarkerTriangle::get_shape(){
		return visualization_msgs::Marker::TRIANGLE_LIST;
	}

	PrimitiveMarkerTriangle::PrimitiveMarkerTriangle(){
	}
	void PrimitiveMarkerTriangle::initPrimitiveMarker( PrimitiveMarkerTriangle *pmt){

		std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > vt = pmt->getVerticesAndTriangles();

#ifdef FCL_COLLISION_CHECKING
		this->bvh = new fcl::BVHModel< BoundingVolume >();
		primitiveMarker2BVH(this->bvh, vt);
#endif
		this->pqp_model = new PQP_Model;

		primitiveMarker2PQP(this->pqp_model, vt);
		primitiveMarker2RVIZMarker(this->marker, vt);

		init_marker();
	}
#ifdef FCL_COLLISION_CHECKING

	void PrimitiveMarkerTriangle::primitiveMarker2PQP(PQP_Model *m, std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > &vt){

		std::vector<fcl::Vec3f> vertices = vt.first;
		std::vector<fcl::Triangle> triangles = vt.second;

		m->BeginModel();
			
		uint counter=0;
		double safety_scale = 1.0;
		for (uint i = 0; i < vertices.size(); i+=3){
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = safety_scale*vertices.at(i)[0];
			p1[1] = safety_scale*vertices.at(i)[1];
			p1[2] = safety_scale*vertices.at(i)[2];
			if(i==0 || i==3){
				ROS_INFO("1: x %f, y %f, z %f", p1[0], p1[1], p1[2]);
			}
				ROS_INFO("1: x %f, y %f, z %f", p1[0], p1[1], p1[2]);

			p2[0] = safety_scale*vertices.at(i+1)[0];
			p2[1] = safety_scale*vertices.at(i+1)[1];
			p2[2] = safety_scale*vertices.at(i+1)[2];
			if(i==0 || i==3){
				ROS_INFO("2: x %f, y %f, z %f", p2[0], p2[1], p2[2]);
			}
				ROS_INFO("2: x %f, y %f, z %f", p2[0], p2[1], p2[2]);

			p3[0] = safety_scale*vertices.at(i+2)[0];
			p3[1] = safety_scale*vertices.at(i+2)[1];
			p3[2] = safety_scale*vertices.at(i+2)[2];
			if(i==0 || i==3){
				ROS_INFO("3: x %f, y %f, z %f", p3[0], p3[1], p3[2]);
			}
				ROS_INFO("3: x %f, y %f, z %f", p3[0], p3[1], p3[2]);
				ROS_INFO("-------------------------------");

			int err = m->AddTri(p1,p2,p3,counter);
			if(err){
				ROS_INFO("ERROR %d", err);
			}
			counter++;
		}
		int err = m->EndModel();
			if(err){
				ROS_INFO("ERROR %d", err);
			}

	}
	void PrimitiveMarkerTriangle::primitiveMarker2BVH(fcl::BVHModel< BoundingVolume > *m, std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > &vt){

		std::vector<fcl::Vec3f> vertices = vt.first;
		std::vector<fcl::Triangle> triangles = vt.second;

		bvh->bv_splitter.reset (new fcl::BVSplitter<BoundingVolume>(fcl::SPLIT_METHOD_MEAN));
		bvh->beginModel();
		bvh->addSubModel(vertices, triangles);
		bvh->endModel();
	}
	void PrimitiveMarkerTriangle::primitiveMarker2RVIZMarker(visualization_msgs::Marker &marker, std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > &vt){

		std::vector<fcl::Vec3f> vertices = vt.first;
		std::vector<fcl::Triangle> triangles = vt.second;

		Color cc = get_color();
		std_msgs::ColorRGBA c;
		c.r = cc.r;
		c.g = cc.g;
		c.b = cc.b;
		c.a = cc.a;

		for (uint i = 0; i < vertices.size(); i++){
			geometry_msgs::Point p;
			p.x = vertices.at(i)[0];
			p.y = vertices.at(i)[1];
			p.z = vertices.at(i)[2];
			marker.points.push_back(p);
			marker.colors.push_back(c);
		}
	}
#endif
}

