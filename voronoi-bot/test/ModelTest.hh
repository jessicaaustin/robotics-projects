#include <cxxtest/TestSuite.h>
#include "Model.hh"
#include "Obstacle.hh"

// TODO origin != 0

class ModelSquareTest: public CxxTest::TestSuite {
public:
	Model *model;
	void setUp() {
		std::vector<Point> modelBoundary;
		modelBoundary.push_back(Point(1, 1));
		modelBoundary.push_back(Point(1, -1));
		modelBoundary.push_back(Point(-1, -1));
		modelBoundary.push_back(Point(-1, 1));
		model = new Obstacle(Point(0, 0), modelBoundary);
	}
	void tearDown() {
		delete model;
	}

	void testEnclosesReturnsFalseIfPointOutsideModel(void) {
		TS_ASSERT(!model->encloses(Point(10, 10)));
		TS_ASSERT(!model->encloses(Point(-10, -10)));
	}
	void testEnclosesReturnsTrueIfPointInsideModel(void) {
		TS_ASSERT(model->encloses(Point(0, 0)));
		TS_ASSERT(model->encloses(Point(.5, .5)));
		TS_ASSERT(model->encloses(Point(.5, -.5)));
		TS_ASSERT(model->encloses(Point(0, .5)));
		TS_ASSERT(model->encloses(Point(.5, 0)));
	}
	void testEnclosesReturnsTrueIfPointOnModelEdge(void) {
		TS_ASSERT(model->encloses(Point(1, 1)));
		TS_ASSERT(model->encloses(Point(1, 0)));
		TS_ASSERT(model->encloses(Point(0, 1)));
		TS_ASSERT(model->encloses(Point(-1, 0)));
		TS_ASSERT(model->encloses(Point(0, -1)));
	}

};

class ModelTriangleTest: public CxxTest::TestSuite {
public:
	Model *model;
	void setUp() {
		std::vector<Point> modelBoundary;
		modelBoundary.push_back(Point(0, 1));
		modelBoundary.push_back(Point(1, -1));
		modelBoundary.push_back(Point(-1, -1));
		model = new Obstacle(Point(0, 0), modelBoundary);
	}
	void tearDown() {
		delete model;
	}
	void testEnclosesReturnsFalseIfPointOutsideModel(void) {
		TS_ASSERT(!model->encloses(Point(1, 0)));
		TS_ASSERT(!model->encloses(Point(-1, 0)));
		TS_ASSERT(!model->encloses(Point(0, -2)));
		TS_ASSERT(!model->encloses(Point(-2, 0)));
	}
	void testEnclosesReturnsTrueIfPointInsideModel(void) {
		TS_ASSERT(model->encloses(Point(0, 0)));
		TS_ASSERT(model->encloses(Point(0, .5)));
		TS_ASSERT(model->encloses(Point(0, -.5)));
	}
	void testEnclosesReturnsTrueIfPointOnModelEdge(void) {
		TS_ASSERT(model->encloses(Point(0, 1)));
		TS_ASSERT(model->encloses(Point(1, -1)));
		TS_ASSERT(model->encloses(Point(-1, -1)));
		TS_ASSERT(model->encloses(Point(.5, 0)));
		TS_ASSERT(model->encloses(Point(-.5, 0)));
	}
};

class ModelConcaveShapeTest: public CxxTest::TestSuite {
public:
	Model *model;
	void setUp() {
		std::vector<Point> modelBoundary;
		modelBoundary.push_back(Point(2, 2));
		modelBoundary.push_back(Point(-0.5, 0));
		modelBoundary.push_back(Point(1, -2));
		modelBoundary.push_back(Point(-1, -1));
		modelBoundary.push_back(Point(-1, 1));
		model = new Obstacle(Point(0, 0), modelBoundary);
	}
	void tearDown() {
		delete model;
	}
	void testEnclosesReturnsFalseIfPointOutsideModel(void) {
		TS_ASSERT(!model->encloses(Point(0, 1.5)));
		TS_ASSERT(!model->encloses(Point(1.5, 2)));
		TS_ASSERT(!model->encloses(Point(0, 0)));
		TS_ASSERT(!model->encloses(Point(.5, -1)));
		TS_ASSERT(!model->encloses(Point(0, -2)));
		TS_ASSERT(!model->encloses(Point(-.5, -1.5)));
		TS_ASSERT(!model->encloses(Point(1.5, .5)));
	}
	void testEnclosesReturnsTrueIfPointInsideModel(void) {
		TS_ASSERT(model->encloses(Point(1, 1.5)));
		TS_ASSERT(model->encloses(Point(0, 1)));
		TS_ASSERT(model->encloses(Point(-.75, 0)));
		TS_ASSERT(model->encloses(Point(.5, -1.5)));
		TS_ASSERT(model->encloses(Point(-.5, 1)));
	}
	void testEnclosesReturnsTrueIfPointOnModelEdge(void) {
		TS_ASSERT(model->encloses(Point(0, -1.5)));
		TS_ASSERT(model->encloses(Point(-.5, 0)));
		TS_ASSERT(model->encloses(Point(-1, 0)));
		TS_ASSERT(model->encloses(Point(1, -2)));
	}
};

class ModelComplexShapeTest: public CxxTest::TestSuite {
public:
	Model *model;
	void setUp() {
		std::vector<Point> modelBoundary;
		modelBoundary.push_back(Point(0, 2));
		modelBoundary.push_back(Point(.5, 2.5));
		modelBoundary.push_back(Point(1, 2));
		modelBoundary.push_back(Point(0, 1));
		modelBoundary.push_back(Point(.5, -.5));
		modelBoundary.push_back(Point(2, 2.5));
		modelBoundary.push_back(Point(3, 0));
		modelBoundary.push_back(Point(1.5, -2));
		modelBoundary.push_back(Point(-1, -1.5));
		modelBoundary.push_back(Point(-2, -.5));
		modelBoundary.push_back(Point(-1.5, .5));
		modelBoundary.push_back(Point(-.5, .25));
		model = new Obstacle(Point(0, 0), modelBoundary);
	}
	void tearDown() {
		delete model;
	}
	void testEnclosesReturnsFalseIfPointOutsideModel(void) {
		TS_ASSERT(!model->encloses(Point(.5, 0)));
		TS_ASSERT(!model->encloses(Point(1, 1.5)));
		TS_ASSERT(!model->encloses(Point(-.5, .5)));
		TS_ASSERT(!model->encloses(Point(-1.5, -1.5)));
		TS_ASSERT(!model->encloses(Point(2.5, -1.5)));
		TS_ASSERT(!model->encloses(Point(.75, 1)));
		TS_ASSERT(!model->encloses(Point(1, 2.5)));
		TS_ASSERT(!model->encloses(Point(.5, -2)));
		TS_ASSERT(!model->encloses(Point(1, -2)));
		TS_ASSERT(!model->encloses(Point(.75, -2)));
	}
	void testEnclosesReturnsTrueIfPointInsideModel(void) {
		TS_ASSERT(model->encloses(Point(0, 0)));
		TS_ASSERT(model->encloses(Point(.5, -1)));
		TS_ASSERT(model->encloses(Point(.5, 2)));
		TS_ASSERT(model->encloses(Point(.25, 0)));
		TS_ASSERT(model->encloses(Point(1, 0)));
		TS_ASSERT(model->encloses(Point(-.5, -1)));
		TS_ASSERT(model->encloses(Point(2.5, 0)));
		TS_ASSERT(model->encloses(Point(-1.5, 0)));
	}
	void testEnclosesReturnsTrueIfPointOnModelEdge(void) {
		TS_ASSERT(model->encloses(Point(.5, -.5)));
		TS_ASSERT(model->encloses(Point(1, .5)));
		TS_ASSERT(model->encloses(Point(-2, -.5)));
		TS_ASSERT(model->encloses(Point(1, 2)));
		TS_ASSERT(model->encloses(Point(.5, 1.5)));
	}
};


class ModelPlotTest: public CxxTest::TestSuite {
public:
	Model *model;
	void setUp() {
		std::vector<Point> modelBoundary;
		modelBoundary.push_back(Point(1, 1));
		modelBoundary.push_back(Point(1, -1));
		modelBoundary.push_back(Point(-1, -1));
		modelBoundary.push_back(Point(-1, 1));
		model = new Obstacle(Point(0, 0), modelBoundary);
	}
	void tearDown() {
		delete model;
	}

	void testOutputsPointsInMatlabFormat(void) {
		std::string expectedPlot = "model=[1 1; 1 -1; -1 -1; -1 1; ];";
		TS_ASSERT_EQUALS(expectedPlot, model->plot());
	}
};
