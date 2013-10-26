#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main( int argc, char** argv )
{
  Mat bw_image;
  String filename = argv[1];
  bw_image = imread( filename, 0 );

  imwrite("bw_" + filename, bw_image);

//  namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
//  imshow( "Display Image", image );
//
//  waitKey(0);

  return 0;
}
