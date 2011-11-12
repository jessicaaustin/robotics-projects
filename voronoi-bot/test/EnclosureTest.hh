#include <cxxtest/TestSuite.h>
#include "Enclosure.hh"

class EnclosurePointReachableTest: public CxxTest::TestSuite {
public:
	void testPointIsReachableIfItIsInsideTheEnclosure(void) {
		std::vector<Point> modelBoundary;
		modelBoundary.push_back(Point(1, 1));
		modelBoundary.push_back(Point(1, -1));
		modelBoundary.push_back(Point(-1, -1));
		modelBoundary.push_back(Point(-1, 1));
		Enclosure enclosure = Enclosure(Point(0, 0), modelBoundary);

		TS_ASSERT(enclosure.isPointReachable(Point(0, 0)));
		TS_ASSERT(!enclosure.isPointReachable(Point(2, 2)));
	}
};
