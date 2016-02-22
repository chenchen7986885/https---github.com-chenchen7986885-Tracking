#ifndef ISODATA_THRESHOLD_H
#define ISODATA_THRESHOLD_H

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class isodata_threshold
{
public:
    isodata_threshold();

    int process(Mat src);
};

#endif // ISODATA_THRESHOLD_H
