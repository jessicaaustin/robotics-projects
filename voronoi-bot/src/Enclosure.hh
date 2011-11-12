#ifndef __ENCLOSURE_HH
#define __ENCLOSURE_HH

#include "Model.hh"

class Enclosure : public Model {
public:
	Enclosure(Point origin, std::vector<Point> boundary);
	bool isPointReachable(Point point);
};

#endif
