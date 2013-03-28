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
inline void std_seed(){
	srand(0);
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
	if ( fgets(line, 1024, fp) != NULL )
	{
		line[strlen(line)-1] = '\0';
		//fprintf(stderr, "%s\n", line);
	}
	string path = string(line);
	return path;
}

inline std::string get_tris_str(std::string relative_path){
	std::string prefix = get_data_path(); //robotDATA.dat path
	char tris_file[200];
	sprintf(tris_file, "%s%s", prefix.c_str(), relative_path.c_str());
	std::string tris = string(tris_file);
	return tris;
}
inline std::string get_robot_str(){
	return get_tris_str("fullBodyApprox/fullbody_-14_-21_-29.tris");
}
inline std::string get_chair_str(){
	return get_tris_str("chairLabo.tris");
}
inline std::string get_logging_str(char* prefix, std::string s){
	char logfile[200];


	std::string::reverse_iterator rit=s.rbegin();
	std::string::iterator it=s.begin();

	std::string x1,x2,x3;

	printf("orig: %s\n",s.c_str());
	//assume that file is called *[num]_[num]_[num].tris
	while(*it != '_'){ it++;}
	it++;
	while(*it != '_'){ x1+=*it;it++; }
	it++;
	while(*it != '_'){ x2+=*it;it++; }
	it++;
	while(*it != '.'){ x3+=*it;it++; }
	it++;
	printf("%s\n",x1.c_str());
	printf("%s\n",x2.c_str());
	printf("%s\n",x3.c_str());


	sprintf(logfile, "%ssample_%s_%s_%s.tmp", prefix, x1.c_str(),x2.c_str(),x3.c_str());
	printf("%s\n",logfile);
	return logfile;
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

struct CSVReader{
private:
	FILE *fp;
	std::string name;
public:
	CSVReader(std::string name, char delimiter=' '){
		this->name = name;
		fp = fopen_s(name.c_str(), "r");
	}
	bool line(std::vector<double> &in){
		char line[200];
		char t;
		if( fgets( line, 200, fp ) ){
		  	double d;
			for(uint i=0;i<11;i++){
				fscanf( fp, "%lg", &d);
				fread(&t, sizeof(char), 1, fp);
				in.push_back(d);
			}
		}
		if( in.size() == 0){
			return false;
		}else{
			return true;
		}
	}
	void getVV(std::vector< std::vector<double> > &in){
		char line[300];
		while( fgets( line, 300, fp ) )
		{
		  	double x,y,t,c;
			std::vector<double> vv;
			char *tmp = line;
			sscanf( tmp, "%lg %lg %lg %lg ", &x,&y,&t,&c);
			vv.push_back(x);
			vv.push_back(y);
			vv.push_back(t);
			vv.push_back(c);
			vv.push_back(0);
			in.push_back(vv);
		}
		//}

	}
	~CSVReader(){
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
