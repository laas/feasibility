#pragma once
#include <vector>

class ConstraintsCheckerHyperPlanar{
public:
	ConstraintsCheckerHyperPlanar(){
		loadHyperPlaneParameters("data/planeparams.dat");
	}

	virtual bool isFeasible(  const std::vector<double> &p, const std::vector< std::vector<double> > &obj){
		double hyper = this->computeHyperPlaneDistance( p, obj);
		if(hyper<0){
			//no collision
			return false;
		}else{
			return true;
		}
	}

// support foot (SF)
//get object position in the coordinate system of the SF -- and prune objects
//which are too far away
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
			std::vector<double> cyl(3);
			cyl.at(0)=sqrtf(rx*rx + ry*ry); cyl.at(1)=atan2(rx,ry); cyl.at(2)=yaw;

			//prune objects, which are far away
			if(cyl.at(0)<0.5){
				v.push_back(cyl);
			}
			DEBUG( ROS_INFO("object transformed from %f %f %f --> %f %f %f (rel to %f %f %f)", x, y, yaw, rx, ry, ryaw, sf_x, sf_y, sf_yaw) );

		}
		return v;
	}//prepare objects
	// OUTPUT: [HyperplaneParams NormalizedConstant RelativeFootPosition]
	// ---> [a b c d Z x y theta], whereby a,b,c,d are the hyperplane params,
	// Z is sqrt(a*a+b*b+c*c) and x,y,theta is the position of the free foot,
	// relative to the support foot

	void loadHyperPlaneParameters(const char *file){

		//FILE *fp = fopen_s(file,'r');
		ROS_INFO("loading parameter from %s", file);
		CSVReader f(file);
		std::vector< std::vector<double> > vv = f.getVV(7);

		bool collision=false;
		for(uint k=0;k<vv.size();k++){
			// p = [a b c d Z], whereby Z=sqrt(a*a+b*b+c*c)
			std::vector<double> params(5);
			for(uint i=0;i<4;i++){
				params.at(i) = (vv.at(k).at(i));
			}

			double norm= 0.0;
			for(uint i=0;i<3;i++){ //a*a+b*b+c*c
				norm += vv.at(k).at(i)*vv.at(k).at(i);
			}
			for(uint i=0;i<4;i++){
				params.at(i)/=sqrtf(norm);
			}
			params.at(4)= sqrtf(norm);

			//position (x,y,t) relative of FF
			std::vector<double> pos;
			for(uint i=4;i<7;i++) pos.push_back(vv.at(k).at(i)/100.0);

			uint hash = hashit<double>(pos);

			ROS_INFO("hash: %d (%f %f %f)", hash, pos.at(0), pos.at(1), pos.at(2));
			if(hyperplane.find(hash)!=hyperplane.end()){
				ROS_INFO("hash collision: %d", hash);
				collision=true;
			}
			std::vector<double> params_and_pos;
			for(uint i=0;i<params.size();i++) params_and_pos.push_back( params.at(i) );
			for(uint i=0;i<pos.size();i++) params_and_pos.push_back( pos.at(i) );
			hyperplane[hash] = params_and_pos;
		}
		if(collision){
			ROS_INFO("WARNING: collision in hyperplane");
			throw "collision in hyperplane error";
			exit(-1);
		}
		ROS_INFO("successfully loaded hyperplane hash map without collisions and %d entries!", hyperplane.size());
	}
	double computeHyperPlaneDistance( const std::vector<double> &p, const std::vector< std::vector<double> > &obj){
		if(obj.size()==0){
			return -999;
		}

		double a = p.at(0);
		double b = p.at(1);
		double c = p.at(2);
		double d = p.at(3);

		std::vector<double> cost_per_object;
		std::vector< std::vector<double> >::const_iterator oit;

		for(  oit = obj.begin(); oit != obj.end(); ++oit ){
			//compute distance from hyperplane (TODO: port to eigen to make
			//it accesible in N dimensions)
			double ox = (*oit).at(0);
			double oy = (*oit).at(1);
			double ot = (*oit).at(2);
			//double dist = (p.at(0)*ox+p.at(1)*oy+p.at(2)*ot+p.at(3));//sqrtf(a*a+b*b+c*c);
			double dist = (a*ox+b*oy+c*ot+d) - 1.0;
			cost_per_object.push_back( dist );
			//cost_per_object.push_back( ((p.at(0)*(*oit).at(0)+p.at(1)*(*oit).at(1)+p.at(2)*(*oit).at(2)+p.at(3))/p.at(4)) - 1.0 );
			//we learned the objects with a probability metric of +1 >
		}
		return *max_element(cost_per_object.begin(), cost_per_object.end());
	}
private:
	ConstraintsChecker();

};
