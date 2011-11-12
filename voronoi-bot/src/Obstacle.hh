#ifndef __OBSTACLE_HH
#define __OBSTACLE_HH

#include "Model.hh"

class Obstacle : public Model {
public:
	Obstacle(Point origin, std::vector<Point> boundary);
	bool isPointReachable(Point point);
};

#endif
