#ifndef _KALMAN_TRACKER_H
#define _KALMAN_TRACKER_H

#include <vector>
#include <time.h>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class AssignmentProblemSolver
{
public:
    enum TMethod {
        optimal,
        many_forbidden_assignments,
        without_forbidden_assignments
    };

public:
    AssignmentProblemSolver();
    ~AssignmentProblemSolver();

public:
    double Solve(vector<vector<double> >& DistMatrix, vector<int>& Assignment, TMethod Method = optimal);

private:
    // --------------------------------------------------------------------------
    // Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
    // --------------------------------------------------------------------------
    void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
    void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
    void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
    void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step3 (int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step4 (int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
    void step5 (int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

    // --------------------------------------------------------------------------
    // Computes a suboptimal solution. Good for cases with many forbidden assignments.
    // --------------------------------------------------------------------------
    void assignmentsuboptimal1(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);

    // --------------------------------------------------------------------------
    // Computes a suboptimal solution. Good for cases with many forbidden assignments.
    // --------------------------------------------------------------------------
    void assignmentsuboptimal2(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
};

class TKalmanFilter
{
public:
    TKalmanFilter(Point2f p, float dt = 0.2, float Accel_noise_mag = 0.5);
    ~TKalmanFilter();

public:
    Point2f GetPrediction();
    Point2f Update(Point2f p, bool DataCorrect);

public:
    KalmanFilter* kalman;
    double deltatime;
    Point2f LastResult;
};

class KalmanTrack
{
public:
    KalmanTrack(Point2f p, float dt, float Accel_noise_mag);
    ~KalmanTrack();

public:
	vector<Point2d> trace;
	static size_t NextTrackID;
	size_t track_id;
	size_t skipped_frames; 
	Point2d prediction;
	TKalmanFilter* KF;
};

class KalmanTracker
{
public:
    KalmanTracker(float _dt, float _Accel_noise_mag, double _dist_thres = 60, int _maximum_allowed_skipped_frames = 10,int _max_trace_length = 10);
    ~KalmanTracker(void);

public:
    void Update(vector<Point2d>& detections);

public:
    vector<KalmanTrack*> tracks;
	float dt, Accel_noise_mag;
	double dist_thres;
    int maximum_allowed_skipped_frames, max_trace_length;
};
#endif

