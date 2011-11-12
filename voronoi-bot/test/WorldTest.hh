#include <cxxtest/TestSuite.h>
#include "World.hh"

class WorldPointReachableTest: public CxxTest::TestSuite {
public:
	void testPointIsReachableIfItIsInsideTheEnclosureButOutsideObstacles(void) {
		std::vector<Point> enclosureBoundary;
		enclosureBoundary.push_back(Point(5, 5));
		enclosureBoundary.push_back(Point(5, -5));
		enclosureBoundary.push_back(Point(-5, -5));
		enclosureBoundary.push_back(Point(-5, 5));
		Enclosure enclosure = Enclosure(Point(0, 0), enclosureBoundary);

		std::vector<Point> obstacleBoundary;
		obstacleBoundary.push_back(Point(1, 1));
		obstacleBoundary.push_back(Point(1, -1));
		obstacleBoundary.push_back(Point(-1, -1));
		obstacleBoundary.push_back(Point(-1, 1));
		Obstacle obstacle = Obstacle(Point(0, 0), obstacleBoundary);

		std::vector<Model*> models;
		models.push_back(&enclosure);
		models.push_back(&obstacle);

		World world = World(Point(0, 0), models);

		TS_ASSERT(!world.isPointReachable(Point(0, 0)));
		TS_ASSERT(world.isPointReachable(Point(2, 2)));
		TS_ASSERT(!world.isPointReachable(Point(10, 10)));
	}
};
