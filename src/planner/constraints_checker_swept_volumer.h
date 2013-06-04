#pragma once
#include <vector>

class ConstraintsCheckerSweptVolume{
public:
	ConstraintsCheckerSweptVolume(){
		//loadSweptVolumes("data/planeparams.dat");
	}

	virtual bool isFeasible(  const std::vector<double> &p, const std::vector< std::vector<double> > &obj){
		exit(-1);
		return true;
	}

	virtual 
	std::vector< std::vector<double> > 
	prepareObjectPosition(std::vector<ros::RVIZVisualMarker*> &obj, 
			double sf_x, double sf_y, double sf_yaw, char foot){
		std::vector< std::vector<double> > v;
		std::vector< std::vector<double> >::const_iterator vit;
		std::vector<ros::RVIZVisualMarker*>::const_iterator oit;
		for(  oit = objects.begin(); oit != objects.end(); ++oit ){
			double x = (*oit)->g.x;
			double y = (*oit)->g.y;
			double yaw = (*oit)->g.getYawRadian();

			//translate object, so that origin and sf origin conincide
			double tx = x - sf_x;
			double ty = y - sf_y;
			//rotate object around origin, such that object is aligned with
			//sf
			double rx = cos(sf_yaw)*tx - sin(sf_yaw)*ty;
			double ry = sin(sf_yaw)*tx + cos(sf_yaw)*ty;
			double ryaw = yaw - sf_yaw;

			//swept vlumes are always tested on the right side
			while(ryaw>M_PI) ryaw-=2*M_PI;
			while(ryaw<-M_PI) ryaw+=2*M_PI;

			//--> cylinder coordinates
			//std::vector<double> cyl(3);
			//cyl.at(0)=sqrtf(rx*rx + ry*ry); cyl.at(1)=atan2(rx,ry); cyl.at(2)=yaw;

			std::vector<double> d(3);
			d.at(0)=rx;
			d.at(0)=ry;
			d.at(0)=ryaw;
			double dist = sqrtf(rx*rx+ry*ry);

			//prune objects, which are far away
			if(dist<0.5){
				v.push_back(d);
			}
		}
		return v;
	}//prepare objects

};
