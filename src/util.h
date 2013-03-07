#pragma once
#include <cstdlib> //rand
#include <sstream> //fstream
#include <iostream> //cout
#include <string> //std::string
#include <vector>
#include <errno.h> //errno
#include <stdio.h> //fopen
#include <stdarg.h> //va_arg
#include <string.h> //strlen

#define CUR_LOCATION "@" << __FILE__ << ":" << __FUNCTION__ << ":" << __LINE__ << ">>"

//see also UTIL::cout for special outputs

#define PRINT(msg) std::cout << CUR_LOCATION << " >> " << msg << std::endl
#define ABORT(msg) PRINT(msg); throw msg;
#define HALT(msg) PRINT(msg); exit(1);
#define COUT(msg) PRINT(msg);
#define CHECK(cond, str) if(!(cond)){ PRINT(str); throw(str); }

using std::endl;
using std::cout;
using std::string;
//##########################################################################
// stable utility functions (misc)
//##########################################################################

inline FILE *fopen_s(const char *path, const char *mode){
	errno=0;
	FILE *tmp = fopen(path,mode);
	if(tmp==NULL){
		HALT("file "<< path<<" failed, error code: " << errno << endl);
	}
	return tmp;
}
inline double rand(double lowerLimit, double upperLimit){
	double x= ((double)rand() / (double)RAND_MAX); //[0,1]
	double xs = x*(upperLimit-lowerLimit); //[0, upperLimit+lowerLimit]
	double xp = xs + lowerLimit; //[lowerLimit, upperLimit]
	return xp;
}

//approximate sampling from N(m,stddev) (probabilistic robotics, p.124)
inline double randn(double m, double stddev){
	double s=0;
	for(uint i=0;i<12;i++){
		s+=rand(-stddev,+stddev);
	}
	return 0.5*s+m;
}

inline double randn_boxmuller(double m, double stddev){
	double x1 = rand(0,1);
	double x2 = rand(0,1);
	double y1 = sqrtf( -2*log(x1) )*cos(2*M_PI*x2);
	//double y2 = sqrtf( -2*log(x1) )*sin(2*pi*x2);
	return stddev*y1 + m;
}
inline double normpdf(double x, double mu=0.0, double s=1.0){
	return (1.0/(s*sqrtf(2*M_PI))) * exp (- (x-mu)*(x-mu) / (2*s*s));
}

inline int hashit(const char *str){
    int h = 0;
    while (*str) h = h << 1 ^ *str++;
    return h;
}

inline std::string get_data_path(){
	FILE *fp;
	fp = fopen_s("robotDATA.dat", "r");
	char line[1024];
	//fgets(line, 1024, fp);
	if ( fgets(line, 1024, fp) != NULL )
	{
		line[strlen(line)-1] = '\0';
		fprintf(stderr, "%s\n", line);
	}
	string path = string(line);
	return path;
}

inline std::string get_robot_str(){
	std::string prefix = get_data_path(); //robotDATA.dat path
	char robot_file[200];
	sprintf(robot_file, "%s%s", prefix.c_str(), "fullBodyApprox/fullbody_-14_-21_-29.tris");

	std::string robot = robot_file;
	return robot;

}
inline std::string get_chair_str(){
	std::string prefix = get_data_path(); //robotDATA.dat path
	char chair_file[200];
	sprintf(chair_file, "%s%s", prefix.c_str(), "chairLabo.tris");
	std::string chair = chair_file;
	return chair;


}


//##########################################################################
// logger / stable
//##########################################################################
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

//##########################################################################
//Special functions / Experimental stuff / not yet stable
//##########################################################################
#ifdef __GXX_EXPERIMENTAL_CXX0X__
	//generic array build by using recursive variadic templates
	template<class T>
	void ahelper(Array<T> &z, T t){ z(z.N-1)=t; }

	template<class T, typename ...A>
	void ahelper(Array<T> &z, T t, A ...args){
		z(z.N-1-sizeof...(args))=t;
		if(sizeof...(args)) ahelper<T>(z, args...);
	}

	//can be called by using e.g. 
	//Array<double> d = ARRAY(1, 2, 3) //infinite arguments possible
	template<class T, typename ...A>
	Array<T> ARRAY( A ...args) {
		Array<T> z(sizeof...(args));
		ahelper<T>(z, args...);
		return z;
	}
#endif

// ostream to include current location and counter
// use as "using util::cout" instead of "std::cout"
// --> util::cout << "hello world" << util::endl;
/*
namespace util{
	static uint counter = 0;
	class LineNumberBuffer: public std::stringbuf{
		std::ostream &output;
	public:
		LineNumberBuffer(std::ostream &str): output(str){}

		virtual int sync(){
			output <<  " " << str();
			str("");
			output.flush();
			util::counter++;
			return 0;
		}
	};
	class UtilStream: public std::ostream{
		LineNumberBuffer buffer;

	public:
		UtilStream(std::ostream& str): buffer(str), std::ostream(&buffer){}
	};

	UtilStream stream(std::cout);
	//workaround for adding file and linenumber to stream
	#define UTIL_LOCATION "[" << util::counter << "]@" << __FILE__ << ":" << __FUNCTION__ << ":" << __LINE__ << ">>"
	#define ccout stream << UTIL_LOCATION
}


*/
//##########################################################################
