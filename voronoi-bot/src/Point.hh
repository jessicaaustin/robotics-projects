#ifndef __POINT_HH
#define __POINT_HH

class Point {
public:
	double x, y;
	Point();
	Point(double x, double y);
	bool operator==(const Point &other) const {
		return x == other.x && y == other.y;
	}
	bool operator!=(const Point &other) const {
		return !(this->operator==(other));
	}
};

#endif
