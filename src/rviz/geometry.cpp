#include <tf/tf.h>
#include <ros/ros.h>
#include "rviz/geometry.h"
#include "util/util.h"

namespace ros{
  void Geometry::lock(){
    mutex.lock();
  }
  void Geometry::unlock(){
    mutex.unlock();
  }
  Geometry::Geometry( const Geometry& rhs ){
    this->lock();
    if(this!=&rhs){

      this->setX( rhs.getX() );
      this->setY( rhs.getY() );
      this->setZ( rhs.getZ() );
      this->setSX( rhs.getSX() );
      this->setSY( rhs.getSY() );
      this->setSZ( rhs.getSZ() );
      this->setQuaternionX( rhs.getQuaternionX() );
      this->setQuaternionY( rhs.getQuaternionY() );
      this->setQuaternionZ( rhs.getQuaternionZ() );
      this->setQuaternionW( rhs.getQuaternionW() );
      this->setRadius( rhs.getRadius() );
      this->setHeight( rhs.getHeight() );
    }
    this->unlock();
  }

  Geometry& Geometry::operator=(Geometry rhs){
    rhs.lock();
    this->lock();
    if(this!=&rhs){
      this->setX( rhs.getX() );
      this->setY( rhs.getY() );
      this->setZ( rhs.getZ() );
      this->setSX( rhs.getSX() );
      this->setSY( rhs.getSY() );
      this->setSZ( rhs.getSZ() );
      this->setQuaternionX( rhs.getQuaternionX() );
      this->setQuaternionY( rhs.getQuaternionY() );
      this->setQuaternionZ( rhs.getQuaternionZ() );
      this->setQuaternionW( rhs.getQuaternionW() );
      this->setRadius( rhs.getRadius() );
      this->setHeight( rhs.getHeight() );
    }
    rhs.unlock();
    this->unlock();
    return *this;
  }

  char Geometry::getFoot() const { return this->foot; }
	double Geometry::getX() const { return this->x; }
	double Geometry::getY() const { return this->y; }
	double Geometry::getZ() const { return this->z; }
	double Geometry::getSX() const { return this->sx; }
	double Geometry::getSY() const { return this->sy; }
	double Geometry::getSZ() const { return this->sz; }
	double Geometry::getRadius() const { return this->radius; }
	double Geometry::getHeight() const { return this->height; }
	double Geometry::getQuaternionX() const { return this->rx; }
	double Geometry::getQuaternionY() const { return this->ry; }
	double Geometry::getQuaternionZ() const { return this->rz; }
	double Geometry::getQuaternionW() const { return this->rw; }
	double Geometry::getYawRadian() const{
		tf::Quaternion q(this->rx, this->ry, this->rz, this->rw);
		double roll, pitch, yaw;

#if ROS_VERSION_MINIMUM(1,8,0)
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
#else
		btMatrix3x3(q).getRPY(roll, pitch, yaw);
#endif

		return yaw;
	}

  void Geometry::setFoot(char r){ this->foot = r; }
	void Geometry::setX(double r){ this->x=r; }
	void Geometry::setY(double r){ this->y=r; }
	void Geometry::setZ(double r){ this->z=r; }
	void Geometry::setSX(double r){ this->sx=r; }
	void Geometry::setSY(double r){ this->sy=r; }
	void Geometry::setSZ(double r){ this->sz=r; }
	void Geometry::setRadius(double r){ this->radius=r; }
	void Geometry::setHeight(double r){ this->height=r; }

	void Geometry::setQuaternionX(double r){ this->rx=r; }
	void Geometry::setQuaternionY(double r){ this->ry=r; }
	void Geometry::setQuaternionZ(double r){ this->rz=r; }
	void Geometry::setQuaternionW(double r){ this->rw=r; }
	void Geometry::setYawRadian(double r){ 
    setRPYRadian(0,0,r);
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
		printf("FOOT: %f\n", (double)foot);
		std::cout << std::endl;
	}
};
