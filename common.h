#ifndef COMMON_H
#define COMMON_H

#include <opencv/cv.h>
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/stitcher.hpp"

using namespace cv;
using namespace std;

#define INPUT_WEBCAM    0
#define INPUT_VIDEO     1

#define METHOD_TRACKING         0
#define METHOD_COUNTING         1
#define METHOD_SURVEILLANCE     2
#define METHOD_FACE             3
#define METHOD_GAZE             4
#define METHOD_HAND             5

#define SAMPLE_NUM						7
#define OBJECT_NOISE_SIZE               20
#define OBJECT_MIN_SIZE					20
#define OBJECT_MOVE_THRESHOLD_X			200
#define OBJECT_MOVE_THRESHOLD_Y			100
#define NUM_BINS                        256

const static cv::Scalar colors[] =  { CV_RGB(0,0,255),
	CV_RGB(0,128,255),
	CV_RGB(0,255,255),
	CV_RGB(0,255,0),
	CV_RGB(255,128,0),
	CV_RGB(255,255,0),
	CV_RGB(255,0,0),
	CV_RGB(255,0,255)} ;

typedef struct
{
    unsigned char hist_val[NUM_BINS];
} AMBPHistogram;

typedef struct {
    int objectCnt;
    vector<int> objectIDs;
    vector<Point> positions;
    vector<cv::Rect> objectRects;
} FRAME_OBJECTS;

typedef struct { 
	Point position;
	int objectIDS;
} OBJECT;

#endif // COMMON_H
