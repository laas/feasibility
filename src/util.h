#pragma once
#include <cstdlib> //rand
#include <sstream> //fstream

#define CUR_LOCATION "@" << __FILE__ << ":" << __FUNCTION__ << ":" << __LINE__ << ">>"
#define PRINT(msg) std::cout << CUR_LOCATION << " >> " << msg << std::endl
#define ABORT(msg) PRINT(msg); throw msg;
#define HALT(msg) PRINT(msg); exit(1);
#define COUT(msg) PRINT(msg);
#define CHECK(cond, str) if(!(cond)){ PRINT(str); throw (str); }


using std::endl;
using std::cout;

FILE *fopen_s(const char *path, const char *mode){
	errno=0;
	FILE *tmp = fopen(path,mode);
	if(tmp==NULL){
		HALT("fopen failed, error code: " << errno << endl);
	}
	return tmp;
}
void *fscanf_s(const char *path, const char *mode){
	errno=0;
	FILE *tmp = fopen(path,mode);
	if(tmp==NULL){
		HALT("fopen failed, error code: " << errno << endl);
	}
	return tmp;
}
double rand(double lowerLimit, double upperLimit){
	//ROS_INFO("%f %f",lowerLimit, upperLimit);
	double x= ((double)rand() / (double)RAND_MAX); //[0,1]
	//ROS_INFO("%f",x);
	double xs = x*(upperLimit-lowerLimit); //[0, upperLimit+lowerLimit]
	//ROS_INFO("%f",xs);
	double xp = xs + lowerLimit; //[lowerLimit, upperLimit]
	//ROS_INFO("%f",xp);
	return xp;
}

//approximate sampling from N(m,stddev) (probabilistic robotics, p.124)
double randn(double m, double stddev){
	double s=0;
	for(uint i=0;i<12;i++){
		s+=rand(-stddev,+stddev);
	}
	return 0.5*s+m;
}

int hashit(const char *str){
    int h = 0;
    while (*str) h = h << 1 ^ *str++;
    return h;
}


struct Logger{
private:
	FILE *fp;
	std::string name;
public:
	Logger(std::string name = "log_output.tmp"){
		this->name = name;
		fp = fopen_s(name.c_str(), "w");
		fclose(fp);
	}
	void operator()(std::string fmt, ...){
		va_list list;
		fp = fopen(this->name.c_str(),"a");
		va_start(list, fmt);
		for(const char *p=fmt.c_str(); *p; ++p){
			if(*p!='%'){
				fputc( *p, fp);
			}else{
				switch(*++p){
				case 's':
					fprintf(fp,"%s", va_arg(list, char*));
					continue;
				case 'd':
					fprintf(fp,"%d", va_arg( list, int ));
					continue;
				case 'f':
					fprintf(fp,"%f", va_arg( list, double ));
					continue;
				default:
					fputc( *p, fp );
				}
			}
		}
		va_end(list);
		fputc('\n', fp);
		fclose(fp);
	}
};
