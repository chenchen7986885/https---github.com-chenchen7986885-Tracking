#ifndef FRAMEDIFF_ENGINE_H
#define FRAMEDIFF_ENGINE_H

#include "opencv2/opencv.hpp"
#include "isodata_threshold.h"

using namespace cv;
using namespace std;

class FrameDiff_Engine
{
public:
    FrameDiff_Engine();
    ~FrameDiff_Engine();

    void process(const Mat &img_input, Mat &img_output);

private:
    Mat img_input_prev, img_input_last, img_foreground;

    isodata_threshold *m_pThreshold;
};

#endif // FRAMEDIFF_ENGINE_H
