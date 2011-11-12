#include "VoronoiBot.hh"

using namespace PlayerCc;

VoronoiBot::VoronoiBot(std::string gHostname, uint gPort, uint gIndex)
{

  // initalize everything
  robot = new PlayerClient(gHostname, gPort); // establishes the fact that you have a robot
  pp = new Position2dProxy(robot, gIndex); // established a proxy to query odometry values and receive motor commands to the robot

  robot->Read();
  xpos = pp->GetXPos();
  ypos = pp->GetYPos();
  thetapos = pp->GetYaw();

  failure = false;

}

VoronoiBot::~VoronoiBot()
{
  delete pp;
  delete robot;
}

void VoronoiBot::createGVG()
{

  // find range(x) and range(y) for the enclosure
  // this defines the size of the grid
  double minX, minY, maxX, maxY;
  minX = 10000000;
  minY = 10000000;
  maxX = -10000000;
  maxY = -10000000;

  for (std::vector<Point>::iterator it = enclosure.begin();
       it != enclosure.end(); it++) {

    if (it->x < minX)
      minX = it->x;
    if (it->y < minY)
      minY = it->y;
    if (it->x > maxX)
      maxX = it->x;
    if (it->y > maxY)
      maxY = it->y;

  }

  // for each point in the grid, determine if it is reachable
  // if so, check if it is equally close to 2 or more obstacles
  // if so, add it to the GVG

  double currX,currY;
  currX = minX + GRID_SIZE;
  currY = minY + GRID_SIZE;

  // iterate over all grid points
  while (currY < maxY) {

    while(currX < maxX) {

      if (isPointReachable(Point(currX,currY)) && numClosestObstacles(Point(currX,currY)))
	gvg.push_back(Point(currX,currY));

      currX += GRID_SIZE;

    }

    currY += GRID_SIZE;
    currX = minX + GRID_SIZE;

  }

}

bool VoronoiBot::isPointReachable(Point p)
{

  double px = p.x;
  double py = p.y;

  double x1,x2;

  /* FIRST CHECK IF THE POINT IS INSIDE THE ENCLOSURE */

  int crossings = 0;

  int size = enclosure.size();

  /* Iterate through each line */
  for (int i = 0; i < size; i++ ) {
    
    /* This is done to ensure that we get the same result when
       the line goes from left to right and right to left */
    if ( enclosure.at(i).x < enclosure.at((i+1)%size).x ){
      x1 = enclosure.at(i).x;
      x2 = enclosure.at((i+1)%size).x;
    } else {
      x1 = enclosure.at((i+1)%size).x;
      x2 = enclosure.at(i).x;
    }
	
    /* First check if the ray is possible to cross the line */
    if ( px > x1 && px <= x2 && ( py < enclosure.at(i).y || py <= enclosure.at((i+1)%size).y ) ) {
      static const float eps = 0.000001;
      
      /* Calculate the equation of the line */
      float dx =  enclosure.at((i+1)%size).x- enclosure.at(i).x;
      float dy = enclosure.at((i+1)%size).y - enclosure.at(i).y;
      float k;
      
      if ( fabs(dx) < eps ){
	k = INFINITY;	// math.h
      } else {
	k = dy/dx;
      }
      
      float m = enclosure.at(i).y - k * enclosure.at(i).x;
      
      /* Find if the ray crosses the line */
      float y2 = k * px + m;
      if ( py <= y2 ){
	crossings++;
      }
    }
  }

  if (crossings % 2 != 1)
    return false;


  /* NOW CHECK IF THE POINT IS INSIDE AN OBSTACLE */

  for (std::vector< std::vector<Point> >::iterator it = obstacles.begin();
       it != obstacles.end(); it++) {

    std::vector<Point> obs = *it;

    crossings = 0;
    
    size = obs.size();

    /* Iterate through each line */
    for (int i = 0; i < size; i++ ) {
      
      /* This is done to ensure that we get the same result when
	 the line goes from left to right and right to left */
      if ( obs.at(i).x < obs.at((i+1)%size).x ){
	x1 = obs.at(i).x;
	x2 = obs.at((i+1)%size).x;
      } else {
	x1 = obs.at((i+1)%size).x;
	x2 = obs.at(i).x;
      }
      
      /* First check if the ray is possible to cross the line */
      if ( px > x1 && px <= x2 && ( py < obs.at(i).y || py <= obs.at((i+1)%size).y ) ) {
	static const float eps = 0.000001;
	
	/* Calculate the equation of the line */
	float dx =  obs.at((i+1)%size).x- obs.at(i).x;
	float dy = obs.at((i+1)%size).y - obs.at(i).y;
	float k;
	
	if ( fabs(dx) < eps ){
	  k = INFINITY;	// math.h
	} else {
	  k = dy/dx;
	}
	
	float m = obs.at(i).y - k * obs.at(i).x;
	
	/* Find if the ray crosses the line */
	float y2 = k * px + m;
	if ( py <= y2 ){
	  crossings++;
	}
      }
    }
    
    if (crossings % 2 == 1)
      return false; 
  }

  return true;

}

double VoronoiBot::dist(Point p1, Point p2)
{
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

bool VoronoiBot::numClosestObstacles(Point p)
{

  std::vector<double> obsDistances;

  // store the distance to each obstacle
  for (std::vector< std::vector<Point> >::iterator it = obstacles.begin();
       it != obstacles.end(); it++) {

    double minDist = 100000;

    // for each obstacle, store the min distance away from point p
    for (std::vector<Point>::iterator it2 = it->begin();
	 it2 != it->end(); it2++) {
      double currDist = dist(p,*it2);
      if (currDist < minDist)
	minDist = currDist;
    }

    obsDistances.push_back(minDist);

  }


  // add the enclosure edges to this array
  for (std::vector< std::vector<Point> >::iterator it = enclosureEdges.begin();
       it != enclosureEdges.end(); it++) {

    double minDist = 100000;

    // for each edge, store the min distance
    for (std::vector<Point>::iterator it2 = it->begin();
	 it2 != it->end(); it2++) {
      double currDist = dist(p,*it2);
      if (currDist < minDist)
	minDist = currDist;
    }

    obsDistances.push_back(minDist);

  }

  // narrow these down to the two closest points
  while (obsDistances.size() > 2) {

    double maxDist = -1000000;

    std::vector<double>::iterator loc;

    for (std::vector<double>::iterator it = obsDistances.begin();
	 it != obsDistances.end(); it++) {

      if (*it > maxDist) {
	maxDist = *it;
	loc = it;
      }

    }

     obsDistances.erase(loc);

  }

  if (fabs(obsDistances.at(0) - obsDistances.at(1)) < DIST_TOLERANCE)
    return true;
  else
    return false;

}

bool VoronoiBot::goToPoint(Point p)
{
  goToPoint(p.x,p.y);
}

bool VoronoiBot::goToPoint(double x, double y)
{

  std::cout<<"Going to point ("<<x<<","<<y<<")"<<std::endl;


  //initial location
  robot->Read();
  xpos = pp->GetXPos();
  ypos = pp->GetYPos();
  thetapos = pp->GetYaw();
  double xinit = xpos;
  double yinit = ypos;

  double xi = xpos;
  double yi = ypos;

  // desired location
  double xd=x;
  double yd=y;
  double td = atan2(yd-yi,xd-xi);


  // error in lateral and longitudinal distances
  double xerr=0;
  double yerr=0;
  double terr =0;

  double elapsedTime = TIMESTEP;

  // Parameters for the line we're trying to follow
  double slope = (yd - yinit)/(xd - xinit);
  double B = 1;
  double A = -slope;
  double C = slope * xd - yd;

  // output parameters
  double speed, turnrate;

  double distToPoint, distToObj;

  /* FIRST, TURN ON THE SPOT */
  while (fabs(thetapos - td) > THETA_TOLERANCE) {

    robot->Read();
    xpos = pp->GetXPos();
    ypos = pp->GetYPos();
    thetapos = pp->GetYaw();

    terr = td-thetapos;

    turnrate = LENGTH*A2*terr;
    if (turnrate > MAX_TURNRATE)
      turnrate = MAX_TURNRATE;
    if (turnrate < -MAX_TURNRATE)
      turnrate = -MAX_TURNRATE;

    pp->SetSpeed(0, turnrate);

    // Let's output this data to the terminal
    //    printf("xpos: %f  ypos: %f  thetapos: %f  td: %f  terr: % f ", xpos, ypos, thetapos,td,terr);

    //    printf("* speed: 0  turnrate: %f \n",turnrate);

    // wait a timestep before recalculating gains
    elapsedTime+=TIMESTEP;
    usleep(TIMESTEP*1000000);
      
  }

  //  pp->SetSpeed(0, 0);
  //  usleep(.5*1000000);

  /* NOW, MOVE TOWARDS POINT */
  while (fabs(xpos-xd) > TOLERANCE || fabs(ypos-yd) > TOLERANCE)
    {

      distToPoint = sqrt( (xpos-xd)*(xpos-xd) + (ypos-yd)*(ypos-yd) );

      // read from the proxie
      robot->Read();
      xpos = pp->GetXPos();
      ypos = pp->GetYPos();
      thetapos = pp->GetYaw();


      xerr = sqrt( (xpos - xd)*(xpos - xd) + (ypos - yd)*(ypos - yd) );
      yerr = (A*xpos + ypos + C)/(sqrt(A*A+B*B));
      terr = td - thetapos;

      speed =  LAMBDA * xerr;
      if (speed > MAX_SPEED)
	speed = MAX_SPEED;

      turnrate = - LENGTH * (A1 * yerr - A2 * terr);
      if (turnrate > MAX_TURNRATE)
	turnrate = MAX_TURNRATE;
      if (turnrate < -MAX_TURNRATE)
	turnrate = -MAX_TURNRATE;


      // Let's output this data to the terminal
      //      printf("xpos: %f  ypos: %f  thetapos: %f  xerr: %f  yerr: %f  terr: % f", xpos, ypos, thetapos,xerr,yerr,terr);

      //      printf("* speed: %f  turnrate: %f \n",speed,turnrate);

      // control the robot by setting motion commands here
      pp->SetSpeed(speed, turnrate);

      // wait a timestep before recalculating gains
      elapsedTime+=TIMESTEP;
      usleep(TIMESTEP*1000000);
    }

  return true;

}

Point VoronoiBot::getClosestPoint(Point p)
{

  double minDist = 10000;
  Point closestPoint = p;

  for (std::vector<Point>::iterator it = gvg.begin();
       it != gvg.end(); it++) {

    if (dist(p,*it) < minDist) {
      minDist = dist(p,*it);
      closestPoint = *it;
    }

  }

  return closestPoint;

}

Point VoronoiBot::getNextPoint(Point p)
{

  double distToGoal = dist(p,goal);

  double minDist = 10000;
  Point p1,p2;

  // a std::vector of points on the GVG that we have *not* visited
  std::vector<Point> possiblePts;
  for (std::vector<Point>::iterator it = gvg.begin();
       it != gvg.end(); it++) {

    bool visited = false;

    for (std::vector<Point>::iterator it2 = pointsVisited.begin();
	 it2 != pointsVisited.end(); it2++) {
      if ((fabs(it->x - it2->x) < TOLERANCE && fabs(it->y - it2->y) < TOLERANCE))
	visited = true;
    }

    if (!visited)
      possiblePts.push_back(*it);

  }

  if (possiblePts.size() ==0) {
    // FAILURE. goal unreachable
    failure = true;
    return p1;
  }

  for (std::vector<Point>::iterator it = possiblePts.begin();
       it != possiblePts.end(); it++) {
    if (dist(p,*it) < minDist) {
      minDist = dist(p,*it);
      p1 = *it;
    }
  }

  minDist = 10000;
  for (std::vector<Point>::iterator it = possiblePts.begin();
       it != possiblePts.end(); it++) {
    if (!(it->x == p1.x && it->y == p1.y) && 
	 (dist(p,*it) < minDist)) {
      minDist = dist(p,*it);
      p2 = *it;
    }
  }

  double distP1 = dist(p1,goal);
  double distP2 = dist(p2,goal);

  // if both closest points on the GVG are further away from the goal than we are now...
  if ((distP1 > distToGoal) && (distP2 > distToGoal)) {
    // we're ready to leave the GVG
    return goal;
  }

  // else go to the point that's closest to the goal
  if (distP1 < distP2)
    return p1;

  return p2;

}

int main(int argc, char **argv)
{
  double xdesired, ydesired;

  /* Calls the command line parser */
  parse_args(argc, argv);

  xdesired = 2.0;
  ydesired = 2.0;

  printf("desired location: (%f , %f)\n",xdesired,ydesired);

  // create the bot
  VoronoiBot bot = VoronoiBot(gHostname,gPort,gIndex);     

  bot.goal.x = xdesired;
  bot.goal.y = ydesired;

  // initial offset
  bot.pp->SetOdometry(-1,-2,0);  

  // robot pose information
  double xinit, yinit, thetainit;

  // get initial offset
  bot.robot->Read();
  xinit = bot.pp->GetXPos();
  yinit = bot.pp->GetYPos();
  thetainit = bot.pp->GetYaw();


  /* 
   * INPUT OBSTACLES AND ENCLOSURE 
   *
   * note that this should probably be read in from a file, but I'm just inputting
   * it directly for convenience.
   */

  std::vector<Point> edge;
  
  edge.push_back(Point(-3,-3));
  edge.push_back(Point(-3 ,  -2.9));
  edge.push_back(Point(-3 ,  -2.8));
  edge.push_back(Point(-3 ,  -2.7));
  edge.push_back(Point(-3 ,  -2.6));
  edge.push_back(Point(-3 ,  -2.5));
  edge.push_back(Point(-3 ,  -2.4));
  edge.push_back(Point(-3 ,  -2.3));
  edge.push_back(Point(-3 ,  -2.2));
  edge.push_back(Point(-3 ,  -2.1));
  edge.push_back(Point(-3 ,  -2.));
  edge.push_back(Point(-3 ,  -1.9));
  edge.push_back(Point(-3 ,  -1.8));
  edge.push_back(Point(-3 ,  -1.7));
  edge.push_back(Point(-3 ,  -1.6));
  edge.push_back(Point(-3 ,  -1.5));
  edge.push_back(Point(-3 ,  -1.4));
  edge.push_back(Point(-3 ,  -1.3));
  edge.push_back(Point(-3 ,  -1.2));
  edge.push_back(Point(-3 ,  -1.1));
  edge.push_back(Point(-3 ,  -1.));
  edge.push_back(Point(-3 ,  -0.9));
  edge.push_back(Point(-3 ,  -0.8));
  edge.push_back(Point(-3 ,  -0.7));
  edge.push_back(Point(-3 ,  -0.6));
  edge.push_back(Point(-3 ,  -0.5));
  edge.push_back(Point(-3 ,  -0.4));
  edge.push_back(Point(-3 ,  -0.3));
  edge.push_back(Point(-3 ,  -0.2));
  edge.push_back(Point(-3 ,  -0.1));
  edge.push_back(Point(-3 ,  0));
  edge.push_back(Point(-3 ,  0.1));
  edge.push_back(Point(-3 ,  0.2));
  edge.push_back(Point(-3 ,  0.3));
  edge.push_back(Point(-3 ,  0.4));
  edge.push_back(Point(-3 ,  0.5));
  edge.push_back(Point(-3 ,  0.6));
  edge.push_back(Point(-3 ,  0.7));
  edge.push_back(Point(-3 ,  0.8));
  edge.push_back(Point(-3 ,  0.9));
  edge.push_back(Point(-3 ,  1.));
  edge.push_back(Point(-3 ,  1.1));
  edge.push_back(Point(-3 ,  1.2));
  edge.push_back(Point(-3 ,  1.3));
  edge.push_back(Point(-3 ,  1.4));
  edge.push_back(Point(-3 ,  1.5));
  edge.push_back(Point(-3 ,  1.6));
  edge.push_back(Point(-3 ,  1.7));
  edge.push_back(Point(-3 ,  1.8));
  edge.push_back(Point(-3 ,  1.9));
  edge.push_back(Point(-3 ,  2.));
  edge.push_back(Point(-3 ,  2.1));
  edge.push_back(Point(-3 ,  2.2));
  edge.push_back(Point(-3 ,  2.3));
  edge.push_back(Point(-3 ,  2.4));
  edge.push_back(Point(-3 ,  2.5));
  edge.push_back(Point(-3 ,  2.6));
  edge.push_back(Point(-3 ,  2.7));
  edge.push_back(Point(-3 ,  2.8));
  edge.push_back(Point(-3 ,  2.9));

  bot.enclosureEdges.push_back(edge);

  for (std::vector<Point>::iterator it = edge.begin();
       it!=edge.end(); it++)
    bot.enclosure.push_back(*it);

  edge.clear();

  edge.push_back(Point(-3,3));
  edge.push_back(Point( -2.9,3));
  edge.push_back(Point( -2.8,3));
  edge.push_back(Point( -2.7,3));
  edge.push_back(Point( -2.6,3));
  edge.push_back(Point( -2.5,3));
  edge.push_back(Point( -2.4,3));
  edge.push_back(Point( -2.3,3));
  edge.push_back(Point( -2.2,3));
  edge.push_back(Point( -2.1,3));
  edge.push_back(Point( -2.,3));
  edge.push_back(Point( -1.9,3));
  edge.push_back(Point( -1.8,3));
  edge.push_back(Point( -1.7,3));
  edge.push_back(Point( -1.6,3));
  edge.push_back(Point( -1.5,3));
  edge.push_back(Point( -1.4,3));
  edge.push_back(Point( -1.3,3));
  edge.push_back(Point( -1.2,3));
  edge.push_back(Point( -1.1,3));
  edge.push_back(Point( -1.,3));
  edge.push_back(Point( -0.9,3));
  edge.push_back(Point( -0.8,3));
  edge.push_back(Point( -0.7,3));
  edge.push_back(Point( -0.6,3));
  edge.push_back(Point( -0.5,3));
  edge.push_back(Point( -0.4,3));
  edge.push_back(Point( -0.3,3));
  edge.push_back(Point( -0.2,3));
  edge.push_back(Point( -0.1,3));
  edge.push_back(Point( 0,3));
  edge.push_back(Point( 0.1,3));
  edge.push_back(Point( 0.2,3));
  edge.push_back(Point( 0.3,3));
  edge.push_back(Point( 0.4,3));
  edge.push_back(Point( 0.5,3));
  edge.push_back(Point( 0.6,3));
  edge.push_back(Point( 0.7,3));
  edge.push_back(Point( 0.8,3));
  edge.push_back(Point( 0.9,3));
  edge.push_back(Point( 1.,3));
  edge.push_back(Point( 1.1,3));
  edge.push_back(Point( 1.2,3));
  edge.push_back(Point( 1.3,3));
  edge.push_back(Point( 1.4,3));
  edge.push_back(Point( 1.5,3));
  edge.push_back(Point( 1.6,3));
  edge.push_back(Point( 1.7,3));
  edge.push_back(Point( 1.8,3));
  edge.push_back(Point( 1.9,3));
  edge.push_back(Point( 2.,3));
  edge.push_back(Point( 2.1,3));
  edge.push_back(Point( 2.2,3));
  edge.push_back(Point( 2.3,3));
  edge.push_back(Point( 2.4,3));
  edge.push_back(Point( 2.5,3));
  edge.push_back(Point( 2.6,3));
  edge.push_back(Point( 2.7,3));
  edge.push_back(Point( 2.8,3));
  edge.push_back(Point( 2.9,3));

  bot.enclosureEdges.push_back(edge);

  for (std::vector<Point>::iterator it = edge.begin();
       it!=edge.end(); it++)
    bot.enclosure.push_back(*it);

  edge.clear();

  edge.push_back(Point(3,3));
  edge.push_back(Point(3 ,  2.9));
  edge.push_back(Point(3 ,  2.8));
  edge.push_back(Point(3 ,  2.7));
  edge.push_back(Point(3 ,  2.6));
  edge.push_back(Point(3 ,  2.5));
  edge.push_back(Point(3 ,  2.4));
  edge.push_back(Point(3 ,  2.3));
  edge.push_back(Point(3 ,  2.2));
  edge.push_back(Point(3 ,  2.1));
  edge.push_back(Point(3 ,  2.));
  edge.push_back(Point(3 ,  1.9));
  edge.push_back(Point(3 ,  1.8));
  edge.push_back(Point(3 ,  1.7));
  edge.push_back(Point(3 ,  1.6));
  edge.push_back(Point(3 ,  1.5));
  edge.push_back(Point(3 ,  1.4));
  edge.push_back(Point(3 ,  1.3));
  edge.push_back(Point(3 ,  1.2));
  edge.push_back(Point(3 ,  1.1));
  edge.push_back(Point(3 ,  1.));
  edge.push_back(Point(3 ,  0.9));
  edge.push_back(Point(3 ,  0.8));
  edge.push_back(Point(3 ,  0.7));
  edge.push_back(Point(3 ,  0.6));
  edge.push_back(Point(3 ,  0.5));
  edge.push_back(Point(3 ,  0.4));
  edge.push_back(Point(3 ,  0.3));
  edge.push_back(Point(3 ,  0.2));
  edge.push_back(Point(3 ,  0.1));
  edge.push_back(Point(3 ,  0));
  edge.push_back(Point(3 , -0.1));
  edge.push_back(Point(3 , -0.2));
  edge.push_back(Point(3 , -0.3));
  edge.push_back(Point(3 , -0.4));
  edge.push_back(Point(3 , -0.5));
  edge.push_back(Point(3 , -0.6));
  edge.push_back(Point(3 , -0.7));
  edge.push_back(Point(3 , -0.8));
  edge.push_back(Point(3 , -0.9));
  edge.push_back(Point(3 , -1.));
  edge.push_back(Point(3 , -1.1));
  edge.push_back(Point(3 , -1.2));
  edge.push_back(Point(3 , -1.3));
  edge.push_back(Point(3 , -1.4));
  edge.push_back(Point(3 , -1.5));
  edge.push_back(Point(3 , -1.6));
  edge.push_back(Point(3 , -1.7));
  edge.push_back(Point(3 , -1.8));
  edge.push_back(Point(3 , -1.9));
  edge.push_back(Point(3 , -2.));
  edge.push_back(Point(3 , -2.1));
  edge.push_back(Point(3 , -2.2));
  edge.push_back(Point(3 , -2.3));
  edge.push_back(Point(3 , -2.4));
  edge.push_back(Point(3 , -2.5));
  edge.push_back(Point(3 , -2.6));
  edge.push_back(Point(3 , -2.7));
  edge.push_back(Point(3 , -2.8));
  edge.push_back(Point(3 , -2.9));

  bot.enclosureEdges.push_back(edge);

  for (std::vector<Point>::iterator it = edge.begin();
       it!=edge.end(); it++)
    bot.enclosure.push_back(*it);

  edge.clear();

  edge.push_back(Point(3,  -3));  
  edge.push_back(Point(  2.9,-3));
  edge.push_back(Point(  2.8,-3));
  edge.push_back(Point(  2.7,-3));
  edge.push_back(Point(  2.6,-3));
  edge.push_back(Point(  2.5,-3));
  edge.push_back(Point(  2.4,-3));
  edge.push_back(Point(  2.3,-3));
  edge.push_back(Point(  2.2,-3));
  edge.push_back(Point(  2.1,-3));
  edge.push_back(Point(  2.,-3));
  edge.push_back(Point(  1.9,-3));
  edge.push_back(Point(  1.8,-3));
  edge.push_back(Point(  1.7,-3));
  edge.push_back(Point(  1.6,-3));
  edge.push_back(Point(  1.5,-3));
  edge.push_back(Point(  1.4,-3));
  edge.push_back(Point(  1.3,-3));
  edge.push_back(Point(  1.2,-3));
  edge.push_back(Point(  1.1,-3));
  edge.push_back(Point(  1.,-3));
  edge.push_back(Point(  0.9,-3));
  edge.push_back(Point(  0.8,-3));
  edge.push_back(Point(  0.7,-3));
  edge.push_back(Point(  0.6,-3));
  edge.push_back(Point(  0.5,-3));
  edge.push_back(Point(  0.4,-3));
  edge.push_back(Point(  0.3,-3));
  edge.push_back(Point(  0.2,-3));
  edge.push_back(Point(  0.1,-3));
  edge.push_back(Point(  0,-3));
  edge.push_back(Point( -0.1,-3));
  edge.push_back(Point( -0.2,-3));
  edge.push_back(Point( -0.3,-3));
  edge.push_back(Point( -0.4,-3));
  edge.push_back(Point( -0.5,-3));
  edge.push_back(Point( -0.6,-3));
  edge.push_back(Point( -0.7,-3));
  edge.push_back(Point( -0.8,-3));
  edge.push_back(Point( -0.9,-3));
  edge.push_back(Point( -1.,-3));
  edge.push_back(Point( -1.1,-3));
  edge.push_back(Point( -1.2,-3));
  edge.push_back(Point( -1.3,-3));
  edge.push_back(Point( -1.4,-3));
  edge.push_back(Point( -1.5,-3));
  edge.push_back(Point( -1.6,-3));
  edge.push_back(Point( -1.7,-3));
  edge.push_back(Point( -1.8,-3));
  edge.push_back(Point( -1.9,-3));
  edge.push_back(Point( -2.,-3));
  edge.push_back(Point( -2.1,-3));
  edge.push_back(Point( -2.2,-3));
  edge.push_back(Point( -2.3,-3));
  edge.push_back(Point( -2.4,-3));
  edge.push_back(Point( -2.5,-3));
  edge.push_back(Point( -2.6,-3));
  edge.push_back(Point( -2.7,-3));
  edge.push_back(Point( -2.8,-3));
  edge.push_back(Point( -2.9,-3));

  bot.enclosureEdges.push_back(edge);

  for (std::vector<Point>::iterator it = edge.begin();
       it!=edge.end(); it++)
    bot.enclosure.push_back(*it);

  // edge.push_back(Point(-3,-3));

  std::vector<Point> obs;
  
  obs.push_back(Point(-1.8,.7));

  obs.push_back(Point(-1.8,.8));
  obs.push_back(Point(-1.8,.9));
  obs.push_back(Point(-1.8,1));
  obs.push_back(Point(-1.8,1.1));
  obs.push_back(Point(-1.8,1.2));
  obs.push_back(Point(-1.8,1.3));
  obs.push_back(Point(-1.8,1.4));

  obs.push_back(Point(-1.8,1.5));

  obs.push_back(Point(-1.7,1.5));
  obs.push_back(Point(-1.6,1.5));
  obs.push_back(Point(-1.5,1.5));
  obs.push_back(Point(-1.4,1.5));

  obs.push_back(Point(-1.3,1.5));

  obs.push_back(Point(-1.3,1.4));
  obs.push_back(Point(-1.3,1.3));
  obs.push_back(Point(-1.3,1.2));
  obs.push_back(Point(-1.3,1.1));
  obs.push_back(Point(-1.3,1.0));
  obs.push_back(Point(-1.3,0.9));
  obs.push_back(Point(-1.3,0.8));

  obs.push_back(Point(-1.3,.7));

  obs.push_back(Point(-1.4,.7));
  obs.push_back(Point(-1.5,.7));
  obs.push_back(Point(-1.6,.7));
  obs.push_back(Point(-1.7,.7));

  //  obs.push_back(Point(-1.8,.7));

  bot.obstacles.push_back(obs);

  obs.clear();

  
  obs.push_back(Point(.6,0));

  obs.push_back(Point(.7,0));
  obs.push_back(Point(.8,0));
  obs.push_back(Point(.9,0));
  obs.push_back(Point(1.0,0));
  obs.push_back(Point(1.1,0));
  obs.push_back(Point(1.2,0));

  obs.push_back(Point(1.3,0));

  obs.push_back(Point(1.3,-.1));
  obs.push_back(Point(1.3,-.2));
  obs.push_back(Point(1.3,-.3));
  obs.push_back(Point(1.3,-.4));
  obs.push_back(Point(1.3,-.5));
  obs.push_back(Point(1.3,-.6));

  obs.push_back(Point(1.3,-.7));

  obs.push_back(Point(1.2,-.7));
  obs.push_back(Point(1.1,-.7));
  obs.push_back(Point(1.0,-.7));
  obs.push_back(Point(.9,-.7));
  obs.push_back(Point(.8,-.7));
  obs.push_back(Point(.7,-.7));

  obs.push_back(Point(.6,-.7));

  obs.push_back(Point(.6,-.6));
  obs.push_back(Point(.6,-.5));
  obs.push_back(Point(.6,-.4));
  obs.push_back(Point(.6,-.3));
  obs.push_back(Point(.6,-.2));
  obs.push_back(Point(.6,-.1));

  //   obs.push_back(Point(.6,0));

  bot.obstacles.push_back(obs);
  
  /*
   * END INPUT OBSTACLES AND ENCLOSURE 
   *
   */


  double dist; // distance to goal

  bool pointReached = false;

  // variables for writing to a file
  std::ofstream outputFile;

  // open the files
  outputFile.open("plotVoronoi.m");

  std::cout<<"Creating GVG... ";

  // build the voronoi graph
  bot.createGVG();

  std::cout<<"done."<<std::endl;

  /*
   * Write the graph to a MATLAB file so we can plot it.
   */

  outputFile<<"enclosure = [";
  for (std::vector<Point>::iterator it = bot.enclosure.begin();
       it != bot.enclosure.end(); it++) {
    outputFile<<it->x<<" "<<it->y<<";"<<std::endl;
  }

  outputFile<<bot.enclosure.front().x<<" "<<bot.enclosure.front().y;
  outputFile<<"];"<<std::endl;

  int count = 0;
  for (std::vector< std::vector<Point> >::iterator it = bot.obstacles.begin();
       it != bot.obstacles.end(); it++) {

    outputFile<<"obs"<<count<<" = [";
    for (std::vector<Point>::iterator it2 = it->begin();
	 it2 != it->end(); it2++) {
      outputFile<<it2->x<<" "<<it2->y<<";"<<std::endl;
    }
    outputFile<<it->front().x<<" "<<it->front().y;
    outputFile<<"];"<<std::endl;
    count++;

  }

  outputFile<<"gvg = [";
  for (std::vector<Point>::iterator it = bot.gvg.begin();
       it != bot.gvg.end(); it++) {
    outputFile<<it->x<<" "<<it->y<<";"<<std::endl;
  }
  outputFile<<"];"<<std::endl;

  outputFile<<std::endl<<std::endl;

  outputFile<<"pos = [";

  outputFile<<xinit<<" "<<yinit<<";"<<std::endl;
  
  printf("Robot initialized. Goal: (%f , %f)\n",xdesired,ydesired);


  /* 
   * BEGIN ALGORITHM
   */


  // First, go to the closest poing on the GVG.
  Point nextPoint;
  nextPoint = bot.getClosestPoint(Point(xinit,yinit));
  bot.goToPoint(nextPoint);

  bot.pointsVisited.push_back(nextPoint);
  outputFile<<nextPoint.x<<" "<<nextPoint.y<<";"<<std::endl;

  // now move towards the goal, along the GVG
  bool goalReached = false;
  while (!goalReached) {

    // get our position
    bot.robot->Read();
    bot.xpos = bot.pp->GetXPos();
    bot.ypos = bot.pp->GetYPos();
    
    // find the next point on the gvg
    nextPoint = bot.getNextPoint(Point(bot.xpos,bot.ypos));

    // if we've gone to all points and haven't reached the goal, report failure
    if (bot.failure) {
      std::cout<<"*** FAILURE. GOAL NOT REACHABLE. ***"<<std::endl;
      break;
    }

    // go to the next point
    bot.goToPoint(nextPoint);

    bot.pointsVisited.push_back(bot.getClosestPoint(nextPoint));    
    outputFile<<nextPoint.x<<" "<<nextPoint.y<<";"<<std::endl;

    // check if we've made it to the goal
    if (fabs(bot.xpos - bot.goal.x) < TOLERANCE && fabs(bot.ypos - bot.goal.y) < TOLERANCE)
      goalReached = true;

    }


  // write stuff to a matlab file

  outputFile<<"];"<<std::endl;

  outputFile<<"figure"<<std::endl;
  outputFile<<"plot(enclosure(:,1),enclosure(:,2),'-ks','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',2);"<<std::endl;
  outputFile<<"hold on"<<std::endl;
  outputFile<<"plot(obs0(:,1),obs0(:,2),'-rs','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2);"<<std::endl;
  outputFile<<"plot(obs1(:,1),obs1(:,2),'-bs','LineWidth',1,'MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',2);"<<std::endl;
  outputFile<<"plot(gvg(:,1),gvg(:,2), '.co');"<<std::endl;
  outputFile<<"plot(pos(:,1),pos(:,2),'--rs','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',3)"<<std::endl;
  outputFile<<"plot("<<xdesired<<","<<ydesired<<",'gd','MarkerSize',15)"<<std::endl;
  outputFile<<"plot("<<xinit<<","<<yinit<<",'go','MarkerSize',10)"<<std::endl;
  outputFile<<"grid on"<<std::endl;
  outputFile<<"axis square"<<std::endl;
  outputFile<<"hold off"<<std::endl;

  outputFile.close();

  printf("Done.\n");

}
