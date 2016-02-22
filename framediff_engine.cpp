#include "framediff_engine.h"

FrameDiff_Engine::FrameDiff_Engine()
{
    m_pThreshold = new isodata_threshold;
}

FrameDiff_Engine::~FrameDiff_Engine()
{
    delete(m_pThreshold);
}

void FrameDiff_Engine::process(const Mat &img_input, Mat &img_output)
{
    if(img_input.empty())
        return;

    if(img_input_prev.empty())
    {
        img_input.copyTo(img_input_prev);
        return;
    }

    if(img_input_last.empty())
    {
        img_input_prev.copyTo(img_input_last);
        return;
    }

    Mat temp1, temp2;
    absdiff(img_input, img_input_prev, temp1);
    absdiff(img_input, img_input_last, temp2);
    add(temp1, temp2, img_foreground);

    if(img_foreground.type() == CV_8UC3)
        cvtColor(img_foreground, img_foreground, CV_BGR2GRAY);

    int thresholdVal = m_pThreshold->process(img_foreground);
    threshold(img_foreground, img_foreground, 15, 255, THRESH_BINARY);

    img_foreground.copyTo(img_output);
    img_input_last = img_input_prev.clone();
    img_input_prev = img_input.clone();
}
