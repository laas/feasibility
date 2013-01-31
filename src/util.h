#pragma once

#define CUR_LOCATION "@" << __FILE__ << ":" << __FUNCTION__ << ":" << __LINE__ << ">>"
#define PRINT(msg) std::cout << CUR_LOCATION << " >> " << msg << std::endl
#define ABORT(msg) PRINT(msg); throw msg;
#define EXIT(msg) PRINT(msg); exit;
#define COUT(msg) PRINT(msg);

using std::endl;
using std::cout;

FILE *fopen_s(const char *path, const char *mode){
	errno=0;
	FILE *tmp = fopen(path,mode);
	if(tmp==NULL){
		EXIT("fopen failed, error code: " << errno << endl);
	}
	return tmp;
}

int hashit(char *str){
    int h = 0;
    while (*str) h = h << 1 ^ *str++;
    return h;
}
