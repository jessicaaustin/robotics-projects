#ifndef __MODEL_HH
#define __MODEL_HH

#include <vector>
#include <string>
#include "Point.hh"

class Model {
	Point origin;
	std::vector<Point> boundary;
	bool rayCrossesLine(Point p1, Point p2, Point p);
	bool pointOnEdge(Point p1, Point p2, Point p);
	bool pointBoundedAboveByY(Point p1, Point p2, Point p);
	bool pointBoundedLeftAndRightByX(Point p1, Point p2, Point p);
	bool pointBoundedLeftAndRightByXInclusive(Point p1, Point p2, Point p);
	float calculateLine(Point p1, Point p2, double x);
	float slope(Point p1, Point p2);
public:
	Model(Point origin, std::vector<Point> boundary);
	bool encloses(Point point);
	virtual bool isPointReachable(Point point) = 0;
	std::string plot();
};

#endif
