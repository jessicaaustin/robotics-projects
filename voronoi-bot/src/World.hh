#ifndef __WORLD_HH
#define __WORLD_HH

#include "Model.hh"

class World {
	Point origin;
	std::vector<Model*> models;
public:
	World(Point origin, std::vector<Model*> models);
	bool isPointReachable(Point point);
};

#endif
