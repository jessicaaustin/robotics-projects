#include "World.hh"

World::World(Point origin, std::vector<Model*> models) :
	origin(origin), models(models) {
}

bool World::isPointReachable(Point point) {
	for (int i = 0; i < models.size(); i++) {
		if (!models.at(i)->isPointReachable(point)) {
			return false;
		}
	}
	return true;
}
