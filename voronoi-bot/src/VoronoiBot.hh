#include <libplayerc++/playerc++.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "args.h"  //make sure this header file exists in your include path
#include "Point.hh"

const double TIMESTEP = .05; //seconds
const double TOLERANCE = .05; //meters
const double THETA_TOLERANCE = .05; //radians

// length of the robot
const double LENGTH = .3; //meters

// maximum speed and turnrate allowed for the robot
const double MAX_SPEED = 1; //m/s
const double MAX_TURNRATE = 1; //rad/s

// control parameters (for point-to-point controller)
const double LAMBDA = .2;
const double A1 = 2;
const double A2 = 1;

// parameters for GVG
const double GRID_SIZE = .5;
const double DIST_TOLERANCE = sqrt(2)*GRID_SIZE;


using namespace PlayerCc; // using the PlayerCc namespace

class VoronoiBot {

public:

  double xpos, ypos, thetapos;
  Point goal;

  PlayerClient* robot;
  Position2dProxy* pp;

  std::vector< std::vector<Point> > obstacles; // the set of obstacle boundaries
  std::vector< std::vector<Point> > enclosureEdges; // the edges of the enclosure
  std::vector<Point> enclosure; // a set of points defining the robot's enclosure
  std::vector<Point> gvg; //the Generalized Voronoi Graph

  std::vector<Point> pointsVisited; // points on the graph that we've visited. keep track of this
                               // so we know if we've failed to reach the goal
  
  // if we visit all points on the GVG and don't get to the goal, the bot fails
  bool failure;

  // Constructor
  VoronoiBot(std::string gHostname, uint gPort, uint gIndex);

  // Destructor
  ~VoronoiBot();

  // FUNCTIONS

  // go to a point
  bool goToPoint(double x, double y);
  bool goToPoint(Point p);

  // create the GVG
  void createGVG();

  // check if a point is inside the enclosure but outside all obstacles
  bool isPointReachable(Point p);

  // distance between two points
  double dist(Point p1, Point p2);

  // check how many obstacles are equally close to a given point
  // note that "closest" is done within some tolerance 
  // this is used to calculate the GVG
  bool numClosestObstacles(Point p);

  // return the closest point on the GVG
  Point getClosestPoint(Point p);

  // get the next point on the GVG towards the goal (or the goal itself)
  Point getNextPoint(Point p);

};
