#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"

namespace ros{

	BlenderMeshTriangleObject::BlenderMeshTriangleObject(const char *cfilename, const char *trisname, double x, double y, double tz): TrisTriangleObject(){
		filename = std::string(cfilename);

		tris_file_name = get_tris_str(trisname);

		this->g.x = x;
		this->g.y = y;

		this->g.sx = 1.0;
		this->g.sy = 1.0;
		this->g.sz = 1.0;

		this->pqp_model = new PQP_Model;
		this->pqp_margin = new PQP_Model;
		this->tris2PQP( this->pqp_model, this->pqp_margin, tris_file_name.c_str() );

		set_color(0,0,0,0);
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
}
