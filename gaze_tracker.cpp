#include <queue>
#include "opencv2/opencv.hpp"
#include "gaze_tracker.h"

// find eye-corner
Mat *leftCornerKernel;
Mat *rightCornerKernel;

// not constant because stupid opencv type signatures
float kEyeCornerKernel[4][6] = {
    {-1,-1,-1, 1, 1, 1},
    {-1,-1,-1,-1, 1, 1},
    {-1,-1,-1,-1, 0, 3},
    { 1, 1, 1, 1, 1, 1},
};

// --------------------------------------------------------------------------
// Function Name : rectInImage
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
bool rectInImage(Rect rect, Mat image)
{
    return rect.x > 0 && rect.y > 0 && rect.x+rect.width < image.cols && rect.y+rect.height < image.rows;
}

// --------------------------------------------------------------------------
// Function Name : inMat
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
bool inMat(Point p,int rows,int cols)
{
    return p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows;
}

// --------------------------------------------------------------------------
// Function Name : matrixMagnitude
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Mat matrixMagnitude(const Mat &matX, const Mat &matY)
{
    Mat mags(matX.rows, matX.cols, CV_64F);
    for (int y = 0; y < matX.rows; ++y)
    {
        const double *Xr = matX.ptr<double>(y), *Yr = matY.ptr<double>(y);
        double *Mr = mags.ptr<double>(y);
        for (int x = 0; x < matX.cols; ++x)
        {
            double gX = Xr[x], gY = Yr[x];
            double magnitude = sqrt((gX * gX) + (gY * gY));
            Mr[x] = magnitude;
        }
    }

    return mags;
}

// --------------------------------------------------------------------------
// Function Name : computeDynamicThreshold
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
double computeDynamicThreshold(const Mat &mat, double stdDevFactor)
{
    Scalar stdMagnGrad, meanMagnGrad;
    meanStdDev(mat, meanMagnGrad, stdMagnGrad);
    double stdDev = stdMagnGrad[0] / sqrt((double)(mat.rows*mat.cols));

    return stdDevFactor * stdDev + meanMagnGrad[0];
}

// --------------------------------------------------------------------------
// Function Name : floodShouldPushPoint
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
bool floodShouldPushPoint(const Point &np, const Mat &mat)
{
    return inMat(np, mat.rows, mat.cols);
}

// --------------------------------------------------------------------------
// Function Name : floodKillEdges
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Mat floodKillEdges(Mat &mat)
{
    rectangle(mat,Rect(0, 0, mat.cols, mat.rows),255);

    Mat mask(mat.rows, mat.cols, CV_8U, 255);
    std::queue<Point> toDo;
    toDo.push(Point(0, 0));
    while (!toDo.empty())
    {
        Point p = toDo.front();
        toDo.pop();
        if (mat.at<float>(p) == 0.0f)
            continue;

        // add in every direction
        Point np(p.x + 1, p.y); // right
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        np.x = p.x - 1; np.y = p.y; // left
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        np.x = p.x; np.y = p.y + 1; // down
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        np.x = p.x; np.y = p.y - 1; // up
        if (floodShouldPushPoint(np, mat)) toDo.push(np);

        // kill it
        mat.at<float>(p) = 0.0f;
        mask.at<uchar>(p) = 0;
    }

    return mask;
}

// --------------------------------------------------------------------------
// Function Name : unscalePoint
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Point unscalePoint(Point p, Rect origSize)
{
    float ratio = (((float)kFastEyeWidth)/origSize.width);
    int x = p.x / ratio;
    int y = p.y / ratio;

    return Point(x,y);
}

// --------------------------------------------------------------------------
// Function Name : scaleToFastSize
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void scaleToFastSize(const Mat &src, Mat &dst)
{
    resize(src, dst, Size(kFastEyeWidth,(((float)kFastEyeWidth)/src.cols) * src.rows));
}

// --------------------------------------------------------------------------
// Function Name : computeMatXGradient
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Mat computeMatXGradient(const Mat &mat)
{
    Mat out(mat.rows, mat.cols, CV_64F);

    for (int y = 0; y < mat.rows; ++y)
    {
        const uchar *Mr = mat.ptr<uchar>(y);
        double *Or = out.ptr<double>(y);

        Or[0] = Mr[1] - Mr[0];
        for (int x = 1; x < mat.cols - 1; ++x)
        {
            Or[x] = (Mr[x+1] - Mr[x-1])/2.0;
        }
        Or[mat.cols-1] = Mr[mat.cols-1] - Mr[mat.cols-2];
    }

    return out;
}

// --------------------------------------------------------------------------
// Function Name : testPossibleCentersFormula
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void testPossibleCentersFormula(int x, int y, const Mat &weight, double gx, double gy, Mat &out)
{
    // for all possible centers
    for (int cy = 0; cy < out.rows; ++cy)
    {
        double *Or = out.ptr<double>(cy);
        const unsigned char *Wr = weight.ptr<unsigned char>(cy);
        for (int cx = 0; cx < out.cols; ++cx)
        {
            if (x == cx && y == cy)
                continue;

            // create a vector from the possible center to the gradient origin
            double dx = x - cx;
            double dy = y - cy;

            // normalize d
            double magnitude = sqrt((dx * dx) + (dy * dy));
            dx = dx / magnitude;
            dy = dy / magnitude;
            double dotProduct = dx*gx + dy*gy;
            dotProduct = std::max(0.0,dotProduct);

            // square and multiply by the weight
            if (kEnableWeight)
                Or[cx] += dotProduct * dotProduct * (Wr[cx]/kWeightDivisor);
            else
                Or[cx] += dotProduct * dotProduct;
        }
    }
}

// --------------------------------------------------------------------------
// Function Name : findEyeCenter
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Point findEyeCenter(Mat face, Rect eye)
{
    Mat eyeROIUnscaled = face(eye);
    Mat eyeROI;
    scaleToFastSize(eyeROIUnscaled, eyeROI);

    // draw eye region
    rectangle(face, eye, 1234);

    //-- Find the gradient
    Mat gradientX = computeMatXGradient(eyeROI);
    Mat gradientY = computeMatXGradient(eyeROI.t()).t();

    //-- Normalize and threshold the gradient
    // compute all the magnitudes
    Mat mags = matrixMagnitude(gradientX, gradientY);
    double gradientThresh = computeDynamicThreshold(mags, kGradientThreshold);

    //normalize
    for (int y = 0; y < eyeROI.rows; ++y)
    {
        double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
        const double *Mr = mags.ptr<double>(y);
        for (int x = 0; x < eyeROI.cols; ++x)
        {
            double gX = Xr[x], gY = Yr[x];
            double magnitude = Mr[x];
            if (magnitude > gradientThresh)
            {
                Xr[x] = gX/magnitude;
                Yr[x] = gY/magnitude;
            }
            else
            {
                Xr[x] = 0.0;
                Yr[x] = 0.0;
            }
        }
    }

    Mat weight;
    GaussianBlur( eyeROI, weight, Size( kWeightBlurSize, kWeightBlurSize ), 0, 0 );
    for (int y = 0; y < weight.rows; ++y)
    {
        unsigned char *row = weight.ptr<unsigned char>(y);
        for (int x = 0; x < weight.cols; ++x)
        {
            row[x] = (255 - row[x]);
        }
    }

    Mat outSum = Mat::zeros(eyeROI.rows, eyeROI.cols, CV_64F);

    for (int y = 0; y < weight.rows; ++y)
    {
        const double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
        for (int x = 0; x < weight.cols; ++x)
        {
            double gX = Xr[x], gY = Yr[x];
            if (gX == 0.0 && gY == 0.0)
                continue;

            testPossibleCentersFormula(x, y, weight, gX, gY, outSum);
        }
    }

    // scale all the values down, basically averaging them
    double numGradients = (weight.rows*weight.cols);
    Mat out;
    outSum.convertTo(out, CV_32F, 1.0/numGradients);

    Point maxP;
    double maxVal;
    minMaxLoc(out, NULL, &maxVal, NULL, &maxP);

    // Flood fill the edges
    Mat floodClone;
    double floodThresh = maxVal * 0.97;
    threshold(out, floodClone, floodThresh, 0.0f, THRESH_TOZERO);
    Mat mask = floodKillEdges(floodClone);
    minMaxLoc(out, NULL, &maxVal, NULL, &maxP, mask);

    return unscalePoint(maxP, eye);
}

// --------------------------------------------------------------------------
// Function Name : createCornerKernels
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void createCornerKernels()
{
    rightCornerKernel = new Mat(4, 6, CV_32F, kEyeCornerKernel);
    leftCornerKernel = new Mat(4, 6, CV_32F);
    flip(*rightCornerKernel, *leftCornerKernel, 1);
}

// --------------------------------------------------------------------------
// Function Name : releaseCornerKernels
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void releaseCornerKernels()
{
    delete leftCornerKernel;
    delete rightCornerKernel;
}

// --------------------------------------------------------------------------
// Function Name : eyeCornerMap
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Mat eyeCornerMap(const Mat &region, bool left, bool left2)
{
    Mat cornerMap;
    Size sizeRegion = region.size();
    Range colRange(sizeRegion.width / 4, sizeRegion.width * 3 / 4);
    Range rowRange(sizeRegion.height / 4, sizeRegion.height * 3 / 4);
    Mat miRegion(region, rowRange, colRange);
    filter2D(miRegion, cornerMap, CV_32F, (left && !left2) || (!left && !left2) ? *leftCornerKernel : *rightCornerKernel);

    return cornerMap;
}

// --------------------------------------------------------------------------
// Function Name : findEyeCorner
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Point2f findEyeCorner(Mat region, bool left, bool left2)
{
    Mat cornerMap = eyeCornerMap(region, left, left2);

    Point maxP;
    minMaxLoc(cornerMap, NULL, NULL, NULL,&maxP);

    Point2f maxP2;
    maxP2 = findSubpixelEyeCorner(cornerMap);

    return maxP2;
}

// --------------------------------------------------------------------------
// Function Name : findSubpixelEyeCorner
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
Point2f findSubpixelEyeCorner(Mat region) {

    Size sizeRegion = region.size();

    Mat cornerMap(sizeRegion.height * 10, sizeRegion.width * 10, CV_32F);

    resize(region, cornerMap, cornerMap.size(), 0, 0, INTER_CUBIC);

    Point maxP2;
    minMaxLoc(cornerMap, NULL, NULL, NULL,&maxP2);

    return Point2f(sizeRegion.width / 2 + maxP2.x / 10, sizeRegion.height / 2 + maxP2.y / 10);
}
