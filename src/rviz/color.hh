#pragma once
namespace ros{
	struct Color{
		Color();
		Color(const Color&);
		Color(double r, double g, double b, double a=0.9);
		double r,g,b,a;
		void print();
	};
	static Color DEFAULT(1.0,0.3,0.0,1.0);
	static Color RED(1.0,0.2,0.0,1.0);
	static Color BLUE(0.1,0.9,0.0,1.0);
	static Color DARKGREEN(0.3,0.7,0.0,1.0);
	static Color WHITE(1.0,1.0,1.0,1.0);
	static Color MAGENTA(0.9,0.0,0.9,1.0);
	static Color SWEPT_VOLUME(0.6,0.0,0.6,0.3);
	static Color OBSTACLE(0.6,0.0,0.6,0.4);
	//static Color TEXT_COLOR(0.9,0.9,0.9,1.0);
	static Color TEXT_COLOR(0.1,0.1,0.1,1.0);

};
