#include "Enclosure.hh"

Enclosure::Enclosure(Point origin, std::vector<Point> boundary) :
	Model(origin, boundary) {
}

bool Enclosure::isPointReachable(Point point) {
	return encloses(point);
}
