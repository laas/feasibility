#include <tf/tf.h>
#include "rviz/rviz_visualmarker.h"
#include "util/util.h"

namespace ros{
	double Geometry::getYawRadian(){
		tf::Quaternion q(this->rx, this->ry, this->rz, this->rw);
		double roll, pitch, yaw;

#if ROS_VERSION_MINIMUM(1,8,0)
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
#else
		btMatrix3x3(q).getRPY(roll, pitch, yaw);
#endif

		return yaw;
	}
	double Geometry::getQuaternionX(){
		return rx;
	}
	double Geometry::getQuaternionY(){
		return ry;
	}
	double Geometry::getQuaternionZ(){
		return rz;
	}
	double Geometry::getQuaternionW(){
		return rw;
	}
	void Geometry::setQuaternionX(double r){
		this->rx=r;
	}
	void Geometry::setQuaternionY(double r){
		this->ry=r;
	}
	void Geometry::setQuaternionZ(double r){
		this->rz=r;
	}
	void Geometry::setQuaternionW(double r){
		this->rw=r;
	}
	void Geometry::setRPYRadian(double roll, double pitch, double yaw){
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw); //in radian
		this->rx = q.getX();
		this->ry = q.getY();
		this->rz = q.getZ();
		this->rw = q.getW();
	}
	Geometry::Geometry(){
		x=0;y=0;z=0;
		rx=0;ry=0;rz=0;rw=1;
		sx=1;sy=1;sz=1;
	}
	void Geometry::print(){
		printf("X %f|Y %f|Z %f\n",x,y,z);
		printf("rx %f|ry %f|rz %f|rw %f\n",rx,ry,rz,rw);
		printf("SX %f|SY %f|SZ %f\n",sx,sy,sz);
		std::cout << std::endl;
	}
	double Geometry::getHeight(){
		return this->height;
	}
	double Geometry::getRadius(){
		return this->radius;
	}
};
