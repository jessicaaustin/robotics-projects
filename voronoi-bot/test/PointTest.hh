#include <cxxtest/TestSuite.h>
#include "Point.hh"

class PointEqualsTest: public CxxTest::TestSuite {
public:
	void testPointsAreEqualIfXAndYCoordinatesAreEqual(void) {
		Point p1 = Point(1,2);
		Point p2 = Point(1,2);
		Point p3 = Point(2,1);
		TS_ASSERT(p1 == p2);
		TS_ASSERT(p2 == p1);
		TS_ASSERT(p1 != p3);
		TS_ASSERT(!(p1 == p3));
	}
};
