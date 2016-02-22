#ifndef _HAND_GESTURE_HPP
#define _HAND_GESTURE_HPP

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#define PI 3.14159

class MyImage
{
public:
    MyImage(int webCamera);
    MyImage();

public:
    void initWebCamera(int i);

public:
    Mat srcLR;
    Mat nonFaceSrc;
    Mat dst;
    Mat bw;
    vector<Mat> bwList;
    VideoCapture cap;
    int cameraSrc;
    Rect left_hand;
    Rect right_hand;
};

class My_ROI
{
public:
    My_ROI();
    My_ROI(Point upper_corner, Point lower_corner, Mat src);

public:
    void draw_rectangle(Mat src);

public:
    Point upper_corner, lower_corner;
    Mat roi_ptr;
    Scalar color;
    int border_thickness;
};

class HandDetector
{
public:
    HandDetector();

public:
    bool detectIfHand();
    void initVectors();
    int getFingerNumber(MyImage *m);
    void eleminateDefects(MyImage *m);
    void getFingerTips(MyImage *m);
    void drawFingerTips(MyImage *m);

private:
    void checkForOneFinger(MyImage *m);
    float getAngle(Point s,Point f, Point e);
    void analyzeContours();
    void computeFingerNumber();
    void drawNewNumber(MyImage *m);
    void addFingerNumberToVector();
    float distanceP2P(Point a,Point b);
    void removeRedundantEndPoints(vector<Vec4i> newDefects, MyImage *m);
    void removeRedundantFingerTips();

public:
    MyImage m;
    vector<vector<Point> > contours;
    vector<vector<int> >hullI;
    vector<vector<Point> >hullP;
    vector<vector<Vec4i> > defects;
    vector <Point> fingerTips;
    Rect rect;
    int cIdx;
    int frameNumber;
    int mostFrequentFingerNumber;
    int nrOfDefects;
    Rect bRect;
    double bRect_width;
    double bRect_height;
    bool isHand;

private:
    int fontFace;
    int prevNrFingerTips;
    vector<int> fingerNumbers;
    vector<int> numbers2Display;
    Scalar numberColor;
    int nrNoFinger;
};
#endif
