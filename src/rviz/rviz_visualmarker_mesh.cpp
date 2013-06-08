#include "rviz/rviz_visualmarker.h"

namespace ros{
	TriangleMeshDecorator::TriangleMeshDecorator(TriangleObject *tobj, const char *meshname){
		marker.mesh_resource = filename;
		marker.mesh_use_embedded_materials=true;
	}
	uint32_t TriangleMeshDecorator::get_shape(){
		return visualization_msgs::Marker::MESH_RESOURCE;
	}

	BlenderMeshTriangleObject::BlenderMeshTriangleObject(const char *cfilename, const char *trisname, double x, double y, double tz): TrisTriangleObject(){
		filename = std::string(cfilename);

		tris_file_name = get_tris_str(trisname);

		this->g.x = x;
		this->g.y = y;

		tf::Quaternion qin;
		qin.setRPY(M_PI/2,0,-M_PI/2);
		this->g.setQuaternionX(qin.getX());
		this->g.setQuaternionY(qin.getY());
		this->g.setQuaternionZ(qin.getZ());
		this->g.setQuaternionW(qin.getW());

		this->g.sx = 0.33;
		this->g.sy = 0.33;
		this->g.sz = 0.33;

#ifdef PQP_COLLISION_CHECKING
		this->pqp_model = new PQP_Model;
		this->pqp_margin = new PQP_Model;
		this->tris2PQP( this->pqp_model, this->pqp_margin, tris_file_name.c_str() );
#endif

		init_marker();
		marker.mesh_resource = filename;
		marker.mesh_use_embedded_materials=true;
	}
	uint32_t BlenderMeshTriangleObject::get_shape(){
		return visualization_msgs::Marker::MESH_RESOURCE;
	}
	double BlenderMeshTriangleObject::getTextZ(){
		return 0.8;
	}
	Color BlenderMeshTriangleObject::get_color(){
		return ros::TEXT_COLOR;
	}
}
