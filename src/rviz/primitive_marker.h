#pragma once
#ifdef FCL_COLLISION_CHECKING
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#endif
#include <Eigen/Core>
#include <pqp/PQP.h>

#include "rviz/visualmarker.h"

namespace ros{
	struct PrimitiveMarkerTriangle: public TriangleObject{
		public:
			uint32_t get_shape();
			PrimitiveMarkerTriangle();
		protected:
			void initPrimitiveMarker( PrimitiveMarkerTriangle *pmt );
			void primitiveMarker2PQP(PQP_Model *m, std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > &vt);
			void primitiveMarker2BVH(fcl::BVHModel< BoundingVolume > *m, std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > &vt);
			void primitiveMarker2RVIZMarker(visualization_msgs::Marker &marker, std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > &vt);
			uint Ntriangles; //number of triangles
			virtual std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > 
							getVerticesAndTriangles() = 0;
	};

	struct PrimitiveMarkerCylinder: public PrimitiveMarkerTriangle{
		double height;
		double radius;
		
		public:
			PrimitiveMarkerCylinder(double x, double y, double r, double h);
			virtual std::string name();

			virtual std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > 
			getVerticesAndTriangles();
			void reloadCylinderBVH(double radius, double height);
	};
	struct PrimitiveMarkerBox: public PrimitiveMarkerTriangle{
		public:
			double w,l,h;
			PrimitiveMarkerBox(double x, double y, double l, double w, double h);
			virtual std::string name();
			virtual std::pair< std::vector<fcl::Vec3f>, std::vector<fcl::Triangle> > 
			getVerticesAndTriangles();
	};
};

