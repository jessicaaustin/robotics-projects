#include <cxxtest/TestSuite.h>
#include "Obstacle.hh"

class ObstaclePointReachableTest: public CxxTest::TestSuite {
public:
	void testPointIsReachableIfItIsOutsideTheObstacle(void) {
		std::vector<Point> obstacleBoundary;
		obstacleBoundary.push_back(Point(1, 1));
		obstacleBoundary.push_back(Point(1, -1));
		obstacleBoundary.push_back(Point(-1, -1));
		obstacleBoundary.push_back(Point(-1, 1));
		Obstacle obstacle = Obstacle(Point(0, 0), obstacleBoundary);

		TS_ASSERT(!obstacle.isPointReachable(Point(0, 0)));
		TS_ASSERT(obstacle.isPointReachable(Point(2, 2)));
	}
};
