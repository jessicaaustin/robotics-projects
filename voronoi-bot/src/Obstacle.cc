#include "Obstacle.hh"

Obstacle::Obstacle(Point origin, std::vector<Point> boundary) :
	Model(origin, boundary) {
}

bool Obstacle::isPointReachable(Point point) {
	return !encloses(point);
}
