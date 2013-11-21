#include <tf/tf.h>
#include "rviz/visualmarker.h"
#include "rviz/visualmarker.h"
#include "util/util.h"

namespace ros{
	double Geometry::getYawRadian(){
		tf::Quaternion q(this->rx_, this->ry_, 
                                 this->rz_, this->rw_);
		double roll, pitch, yaw;

#if ROS_VERSION_MINIMUM(1,8,0)
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
#else
		btMatrix3x3(q).getRPY(roll, pitch, yaw);
#endif

		return yaw;
	}
	double Geometry::getQuaternionX(){
		return rx_;
	}
	double Geometry::getQuaternionY(){
		return ry_;
	}
	double Geometry::getQuaternionZ(){
		return rz_;
	}
	double Geometry::getQuaternionW(){
		return rw_;
	}
	void Geometry::setQuaternionX(double r){
		this->rx_=r;
	}
	void Geometry::setQuaternionY(double r){
		this->ry_=r;
	}
	void Geometry::setQuaternionZ(double r){
		this->rz_=r;
	}
	void Geometry::setQuaternionW(double r){
		this->rw_=r;
	}
	void Geometry::setRPYRadian(double roll, double pitch, double yaw){
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw); //in radian
		this->rx_ = q.getX();
		this->ry_ = q.getY();
		this->rz_ = q.getZ();
		this->rw_ = q.getW();
	}
	Geometry::Geometry(){
		x_=0;y_=0;z_=0;
		rx_=0;ry_=0;rz_=0;rw_=1;
		sx_=1;sy_=1;sz_=1;
	}
	void Geometry::print(){
		printf("X %f|Y %f|Z %f\n",x_,y_,z_);
		printf("rx %f|ry %f|rz %f|rw %f\n",rx_,ry_,rz_,rw_);
		printf("SX %f|SY %f|SZ %f\n",sx_,sy_,sz_);
		std::cout << std::endl;
	}
	double Geometry::getHeight(){
		return this->height_;
	}
	double Geometry::getRadius(){
		return this->radius_;
	}
};
