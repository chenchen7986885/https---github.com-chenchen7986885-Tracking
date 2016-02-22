#ifndef GAZE_TRACKER_H
#define GAZE_TRACKER_H

#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define kEyeLeft true
#define kEyeRight false

// Algorithm Parameters
const int kFastEyeWidth = 50;
const int kWeightBlurSize = 5;
const bool kEnableWeight = true;
const float kWeightDivisor = 1.0;
const double kGradientThreshold = 50.0;

bool rectInImage(Rect rect, Mat image);
bool inMat(Point p, int rows, int cols);
Mat matrixMagnitude(const Mat &matX, const Mat &matY);
double computeDynamicThreshold(const Mat &mat, double stdDevFactor);

// find eye-corner
void createCornerKernels();
void releaseCornerKernels();
Point2f findEyeCorner(Mat region, bool left, bool left2);
Point2f findSubpixelEyeCorner(Mat regionP);

// find eye-center
Point findEyeCenter(Mat face, Rect eye);

#endif
