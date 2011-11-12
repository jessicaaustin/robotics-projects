#include "Model.hh"
#include <math.h>
#include <sstream>

using std::string;
using std::ostringstream;

static const float eps = 0.000001; // TODO what is this number?

Model::Model(Point origin, std::vector<Point> boundary) :
	origin(origin), boundary(boundary) {
}

/**
 * Using a ray casting algorithm. Point is inside if a ray starting from the point crosses
 * the edge of the polygon an odd number of times.
 *
 * Implementation details:
 *  - the ray is directed straight up from the point.
 *  - points on an edge are considered inside the polygon
 *
 * Assumptions: the points along the boundary are given in clockwise order.
 *
 */
bool Model::encloses(Point point) {
	int crossings = 0;

	for (int i = 0; i < boundary.size(); i++) {
		Point currentBoundaryPoint = boundary.at(i);
		Point nextBoundaryPoint = boundary.at((i + 1) % boundary.size());

		if (pointOnEdge(currentBoundaryPoint, nextBoundaryPoint, point)) {
			crossings = 1;
			break;
		}

		if (pointBoundedLeftAndRightByX(currentBoundaryPoint,
				nextBoundaryPoint, point) && pointBoundedAboveByY(
				currentBoundaryPoint, nextBoundaryPoint, point)
				&& rayCrossesLine(currentBoundaryPoint, nextBoundaryPoint,
						point)) {
			crossings++;
		}
	}

	return crossings % 2 == 1;
}

std::string Model::plot() {
	ostringstream plot;
	plot << "model=[";
	for (int i = 0; i < boundary.size(); i++) {
		Point point = boundary.at(i);
		plot << point.x << " " << point.y << "; ";
	}
	plot << "];";
	return plot.str();
}

bool Model::rayCrossesLine(Point p1, Point p2, Point p) {
	return p.y <= calculateLine(p1, p2, p.x);
}

bool Model::pointOnEdge(Point p1, Point p2, Point p) {
	bool pointOnEdge = pointBoundedLeftAndRightByXInclusive(p1, p2, p)
			&& slope(p1, p2) == slope(p2, p);
	bool pointOnVertex = p1 == p || p2 == p;
	return pointOnEdge || pointOnVertex;
}

bool Model::pointBoundedAboveByY(Point p1, Point p2, Point p) {
	return p.y <= p1.y || p.y <= p2.y;
}

bool Model::pointBoundedLeftAndRightByX(Point p1, Point p2, Point p) {
	return (p.x >= p1.x && p.x < p2.x) || (p.x >= p2.x && p.x < p1.x);
}

bool Model::pointBoundedLeftAndRightByXInclusive(Point p1, Point p2, Point p) {
	return (p.x >= p1.x && p.x <= p2.x) || (p.x >= p2.x && p.x <= p1.x);
}

float Model::calculateLine(Point p1, Point p2, double x) {
	float m = slope(p1, p2);
	return m * x + (p1.y - m * p1.x);
}

float Model::slope(Point p1, Point p2) {
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;

	if (fabs(dx) < eps) {
		return INFINITY;
	}
	return dy / dx;
}
