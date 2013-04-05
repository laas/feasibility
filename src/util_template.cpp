#include <sstream> //fstream
#include <iostream> //cout
#include <string> //std::string
//includes all template functions, which have to be loaded into the header

template<typename T>
int hashit(std::vector<T> in){
	std::stringstream ss;
	for(uint i=0;i<in.size();i++){
		ss << in.at(i);
	}
	return hashit(ss.str().c_str());
}

//##########################################################################
//Special functions / Experimental stuff / not yet stable
//##########################################################################
#ifdef __GXX_EXPERIMENTAL_CXX0X__
	//generic array build by using recursive variadic templates
	template<class T>
	void vecahelper(std::vector<T> &z, T t){ z.at(z.size()-1)=t; }

	template<class T, typename ...A>
	void vecahelper(std::vector<T> &z, T t, A ...args){
		z.at(z.size()-1-sizeof...(args))=t;
		if(sizeof...(args)) vecahelper<T>(z, args...);
	}

	//can be called by using e.g. 
	//std::vector<double> d = ARRAY<double>(1, 2, 3) //infinite arguments possible
	template<class T, typename ...A>
	std::vector<T> ARRAY( A ...args) {
		std::vector<T> z(sizeof...(args));
		vecahelper<T>(z, args...);
		return z;
	}
#endif
