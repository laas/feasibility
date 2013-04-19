#include <cstdlib> //rand
#include <math.h> //sqrt, rand
#include <sstream> //fstream
#include <iostream> //cout
#include <ostream> //cout
#include <string> //std::string
#include <vector> //std::vector
#include <errno.h> //errno
#include <stdio.h> //fopen
#include <stdarg.h> //va_arg
#include <string.h> //strlen
#include "util.h"
using std::endl;
using std::cout;
using std::string;

FILE *fopen_s(const char *path, const char *mode){
	errno=0;
	FILE *tmp = fopen(path,mode);
	if(tmp==NULL){
		HALT("file "<< path<<" failed, error code: " << errno << endl);
	}
	return tmp;
}
void std_seed(){
	srand(0);
}
double norml2(double x1, double x2, double y1, double y2){
	return sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}
double norml1(double x1, double x2, double y1, double y2){
	return abs(x1-x2) + abs(y1-y2);
}
double rand(double lowerLimit, double upperLimit){
	double x= ((double)rand() / (double)RAND_MAX); //[0,1]
	double xs = x*(upperLimit-lowerLimit); //[0, upperLimit+lowerLimit]
	double xp = xs + lowerLimit; //[lowerLimit, upperLimit]
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

double randn_boxmuller(double m, double stddev){
	double x1 = rand(0,1);
	double x2 = rand(0,1);
	double y1 = sqrtf( -2*log(x1) )*cos(2*M_PI*x2);
	//double y2 = sqrtf( -2*log(x1) )*sin(2*pi*x2);
	return stddev*y1 + m;
}

double normpdf(double x, double mu, double s){
	return (1.0/(s*sqrtf(2*M_PI))) * exp (- (x-mu)*(x-mu) / (2*s*s));
}

double dist(double x1, double x2, double y1, double y2){
	return sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}
unsigned int JSHash(const std::string& str){
	unsigned int hash = 1315423911;

	for(std::size_t i = 0; i < str.length(); i++){
		hash ^= ((hash << 5) + str[i] + (hash >> 2));
	}

	return (hash & 0x7FFFFFFF);
}
double toRad(double d){
	return d*M_PI/180.0;
}
double toDeg(double d){
	return d*180.0/M_PI;
}

int hashit(const char *str){
	return JSHash(string(str));
}

std::string get_data_path(){
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

std::string get_tris_str(const char *relative_path){
	return get_tris_str(string(relative_path));
}
std::string get_tris_str(std::string relative_path){
	std::string prefix = get_data_path(); //robotDATA.dat path
	char tris_file[200];
	sprintf(tris_file, "%s%s", prefix.c_str(), relative_path.c_str());
	std::string tris = string(tris_file);
	return tris;
}
std::string get_robot_str(const char *sv_file){
	return get_tris_str(sv_file);
}
std::string get_robot_str(){
	return get_robot_str("fullBodyApprox/fullbody_-14_-21_-29.tris");
}
std::string get_chair_str(){
	return get_tris_str("chairLabo.tris");
}
std::string get_logging_str(const char* prefix, std::string s){
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
int round2(double d)
{
	return floor(d + 0.5);
}

//##########################################################################
// logger / stable
//##########################################################################
Logger::Logger(std::string name){
	this->name = name;
	fp = fopen_s(name.c_str(), "w");
	fclose(fp);
}
void Logger::operator()(std::string fmt, ...){
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
	//fputc('\n', fp);
	fclose(fp);
}
void Logger::operator()(std::vector<double> &v){
	fp = fopen(this->name.c_str(),"a");
	std::vector<double>::iterator it;
	for(it=v.begin();it!=v.end();it++){
		fprintf(fp, "%f", *it);
	}
	fputc('\n', fp);
	fclose(fp);
}

CSVReader::CSVReader(std::string name, char delimiter){
	this->name = name;
	fp = fopen_s(name.c_str(), "r");
}
bool CSVReader::line(std::vector<double> &in){
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
std::vector< std::vector<double> > CSVReader::getVV(uint numbers_per_line){
	std::vector< std::vector<double> > in;
	char line[300];
	while( fgets( line, 300, fp ) )
	{
		std::vector<double> vv;
		char *tmp = line;
		char *old = line;
		for(uint i=0;i<numbers_per_line;i++){
			while(*tmp++ != ',');
			double x = 0;
			sscanf( old, "%lg", &x);
			old = tmp;
			vv.push_back(x);
		}
		in.push_back(vv);
	}
	return in;
}
CSVReader::~CSVReader(){
	fclose(fp);
}


//##########################################################################
