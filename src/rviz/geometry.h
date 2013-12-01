#pragma once
#include <boost/thread.hpp>
namespace ros{
	class Geometry{
  private:
		double rx,ry,rz,rw; //quaternions
		double x,y,z; //position world frame
		double sx,sy,sz; //scale
		double radius,height;
    boost::mutex mutex; //NON-COPYABLE
    char foot;

	public:
		Geometry( const ros::Geometry& rhs );
    Geometry& operator=(Geometry rhs);
		Geometry();
		void print();
		void lock();
		void unlock();

    char getFoot() const;
    double getX() const;
    double getY() const;
    double getZ() const;
    double getSX() const;
    double getSY() const;
    double getSZ() const;
		double getQuaternionX() const;
		double getQuaternionY() const;
		double getQuaternionZ() const;
		double getQuaternionW() const;
		double getYawRadian() const; //yaw, in radians of course 
		double getHeight() const;
		double getRadius() const;

    void setFoot(char f);
		void setX(double);
		void setY(double);
		void setZ(double);
		void setSX(double);
		void setSY(double);
		void setSZ(double);
		void setQuaternionX(double);
		void setQuaternionY(double); 
		void setQuaternionZ(double); 
		void setQuaternionW(double); 
		void setYawRadian(double);
		void setRPYRadian(double roll, double pitch, double yaw); //yaw, in radians of course 
    void setRadius(double r);
    void setHeight(double r);
	};
};

