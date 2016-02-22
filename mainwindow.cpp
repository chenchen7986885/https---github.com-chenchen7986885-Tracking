#include "mainwindow.h"
#include "ui_mainwindow.h"

bool g_bProcessing = false;

// For Hand
int square_len = 20;
int avgColor[SAMPLE_NUM][3] ;
int c_lower[SAMPLE_NUM][3];
int c_upper[SAMPLE_NUM][3];
int avgBGR[3];
int nrOfDefects;
struct dim{int w; int h;}boundingDim;
int handCandidateIdx;

Mat edges, grey_img, process_img, nonFace_img;
My_ROI roi1, roi2, roi3, roi4, roi5, roi6;
vector <My_ROI> roi;
vector <KalmanFilter> kf;
vector <Mat_<float> > measurement;
MyImage *webcam;
Rect face_region;

void gaze_tracking()
{
    Mat faceROI = grey_img(face_region);
    Mat faceOrg = process_img(face_region);

    //-- Find eye regions and draw them
    int eye_region_width = face_region.width * (35.0/100.0);
    int eye_region_height = face_region.width * (30.0/100.0);
    int eye_region_top = face_region.height * (25.0/100.0);
    Rect leftEyeRegion(face_region.width*(13.0/100.0),
        eye_region_top,eye_region_width,eye_region_height);
    Rect rightEyeRegion(face_region.width - eye_region_width - face_region.width*(13.0/100.0),
        eye_region_top,eye_region_width,eye_region_height);

    //-- Find Eye Centers
    Point leftPupil = findEyeCenter(faceROI,leftEyeRegion);
    Point rightPupil = findEyeCenter(faceROI,rightEyeRegion);

    // get corner regions
    Rect leftRightCornerRegion(leftEyeRegion);
    leftRightCornerRegion.width -= leftPupil.x;
    leftRightCornerRegion.x += leftPupil.x;
    leftRightCornerRegion.height /= 2;
    leftRightCornerRegion.y += leftRightCornerRegion.height / 2;
    Rect leftLeftCornerRegion(leftEyeRegion);
    leftLeftCornerRegion.width = leftPupil.x;
    leftLeftCornerRegion.height /= 2;
    leftLeftCornerRegion.y += leftLeftCornerRegion.height / 2;
    Rect rightLeftCornerRegion(rightEyeRegion);
    rightLeftCornerRegion.width = rightPupil.x;
    rightLeftCornerRegion.height /= 2;
    rightLeftCornerRegion.y += rightLeftCornerRegion.height / 2;
    Rect rightRightCornerRegion(rightEyeRegion);
    rightRightCornerRegion.width -= rightPupil.x;
    rightRightCornerRegion.x += rightPupil.x;
    rightRightCornerRegion.height /= 2;
    rightRightCornerRegion.y += rightRightCornerRegion.height / 2;

    rectangle(faceOrg, leftRightCornerRegion, Scalar(0, 255, 0));
    rectangle(faceOrg, leftLeftCornerRegion, Scalar(0, 255, 0));
    rectangle(faceOrg, rightLeftCornerRegion, Scalar(0, 255, 0));
    rectangle(faceOrg, rightRightCornerRegion, Scalar(0, 255, 0));

    // change eye centers to face coordinates
    rightPupil.x += rightEyeRegion.x;
    rightPupil.y += rightEyeRegion.y;
    leftPupil.x += leftEyeRegion.x;
    leftPupil.y += leftEyeRegion.y;

    // draw eye centers
    circle(faceOrg, rightPupil, 3, Scalar(255, 255, 0));
    circle(faceOrg, leftPupil, 3, Scalar(255, 255, 0));
}

// --------------------------------------------------------------------------
// Function Name : waitForPalmCover
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- MyImage* m : camera structure for hand detection -*
// This function intented to detect mouth.
// --------------------------------------------------------------------------
void waitForPalmCover(MyImage* m)
{
    m->cap >> m->nonFaceSrc;
    flip(m->nonFaceSrc, m->nonFaceSrc, 1);

    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/3, m->nonFaceSrc.rows/6), Point(m->nonFaceSrc.cols/3+square_len, m->nonFaceSrc.rows/6+square_len), m->nonFaceSrc));
    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/4, m->nonFaceSrc.rows/2), Point(m->nonFaceSrc.cols/4+square_len, m->nonFaceSrc.rows/2+square_len), m->nonFaceSrc));
    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/3, m->nonFaceSrc.rows/1.5), Point(m->nonFaceSrc.cols/3+square_len, m->nonFaceSrc.rows/1.5+square_len), m->nonFaceSrc));
    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/2, m->nonFaceSrc.rows/2), Point(m->nonFaceSrc.cols/2+square_len, m->nonFaceSrc.rows/2+square_len), m->nonFaceSrc));
    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/2.5, m->nonFaceSrc.rows/2.5), Point(m->nonFaceSrc.cols/2.5+square_len, m->nonFaceSrc.rows/2.5+square_len), m->nonFaceSrc));
    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/2, m->nonFaceSrc.rows/1.5), Point(m->nonFaceSrc.cols/2+square_len, m->nonFaceSrc.rows/1.5+square_len), m->nonFaceSrc));
    roi.push_back(My_ROI(Point(m->nonFaceSrc.cols/2.5, m->nonFaceSrc.rows/1.8), Point(m->nonFaceSrc.cols/2.5+square_len, m->nonFaceSrc.rows/1.8+square_len), m->nonFaceSrc));

    for(;;)
    {
        m->cap >> m->nonFaceSrc;
        flip(m->nonFaceSrc, m->nonFaceSrc,1);

        for(int j = 0; j < 7; j++)
        {
            roi[j].draw_rectangle(m->nonFaceSrc);
        }

        string imgText = string("If you are ready, press 'S'.");
        putText(m->nonFaceSrc, imgText, Point(m->nonFaceSrc.cols/2, m->nonFaceSrc.rows/10), FONT_HERSHEY_PLAIN, 1.2f, Scalar(200,0,0),2);

        imshow("result view", m->nonFaceSrc);
        int c = waitKey(30);
        if(c == 's')
            break;
    }
}

// --------------------------------------------------------------------------
// Function Name : getMedian
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- vector<int> val : integer array -*
// Output Data : *- median value of integer array, return type is int -*
// This function intented to get median value of integer array.
// --------------------------------------------------------------------------
int getMedian(vector<int> val)
{
    int median;
    size_t size = val.size();
    sort(val.begin(), val.end());
    if (size  % 2 == 0)
        median = val[size / 2 - 1];
    else
        median = val[size / 2];

    return median;
}

// --------------------------------------------------------------------------
// Function Name : getAvgColor
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- My_ROI roi : subimage structure for hand detection -*, *- int avg[3] : integer array for color value -*
// Output Data : *- median value of integer array, return type is int -*
// This function intented to calculate median color of image.
// --------------------------------------------------------------------------
void getAvgColor(My_ROI roi, int avg[3])
{
    Mat r;
    roi.roi_ptr.copyTo(r);
    vector<int> hm;
    vector<int> sm;
    vector<int> lm;

    // generate vectors
    for(int i = 2; i < r.rows-2; i++)
    {
        for(int j = 2; j < r.cols-2; j++)
        {
            hm.push_back(r.data[r.channels()*(r.cols*i + j) + 0]) ;
            sm.push_back(r.data[r.channels()*(r.cols*i + j) + 1]) ;
            lm.push_back(r.data[r.channels()*(r.cols*i + j) + 2]) ;
        }
    }

    avg[0] = getMedian(hm);
    avg[1] = getMedian(sm);
    avg[2] = getMedian(lm);
}

// --------------------------------------------------------------------------
// Function Name : average
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- MyImage *m : camera structure for hand detection -*
// This function intented to calculate median value of hand feature rects.
// --------------------------------------------------------------------------
void average(MyImage *m)
{
    for(int i = 0; i < 30; i++)
    {
        m->cap >> m->nonFaceSrc;
        flip(m->nonFaceSrc, m->nonFaceSrc,1);

        cvtColor(m->nonFaceSrc, m->nonFaceSrc, CV_BGR2HLS);
        for(int j = 0; j < SAMPLE_NUM; j++)
        {
            getAvgColor(roi[j], avgColor[j]);
            roi[j].draw_rectangle(m->nonFaceSrc);
        }

        cvtColor(m->nonFaceSrc, m->nonFaceSrc, CV_HLS2BGR);
        string imgText = string("Finding average color of hand");
        putText(m->nonFaceSrc, imgText, Point(m->nonFaceSrc.cols/2, m->nonFaceSrc.rows/10), FONT_HERSHEY_PLAIN, 1.2f, Scalar(200, 0, 0), 2);

        imshow("result view", m->nonFaceSrc);
        waitKey(30);
    }

    cvDestroyAllWindows();
}

// --------------------------------------------------------------------------
// Function Name : normalizeColors
// Auther : chen chen
// Data : 2016-02-15
// This function intented to calculate normalize value of hand feature rects.
// --------------------------------------------------------------------------
void normalizeColors()
{
    for(int i = 1; i < SAMPLE_NUM; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            c_lower[i][j] = c_lower[0][j];
            c_upper[i][j] = c_upper[0][j];
        }
    }

    for(int i = 0; i < SAMPLE_NUM; i++)
    {
        if((avgColor[i][0]-c_lower[i][0]) < 0)
            c_lower[i][0] = avgColor[i][0] ;

        if((avgColor[i][1]-c_lower[i][1]) < 0)
            c_lower[i][1] = avgColor[i][1] ;

        if((avgColor[i][2]-c_lower[i][2]) < 0)
            c_lower[i][2] = avgColor[i][2] ;

        if((avgColor[i][0]+c_upper[i][0]) > 255)
            c_upper[i][0] = 255-avgColor[i][0] ;

        if((avgColor[i][1]+c_upper[i][1]) > 255)
            c_upper[i][1] = 255-avgColor[i][1] ;

        if((avgColor[i][2]+c_upper[i][2]) > 255)
            c_upper[i][2] = 255-avgColor[i][2] ;
    }
}

// --------------------------------------------------------------------------
// Function Name : produceBinaries
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- MyImage *m : camera structure for hand detection -*
// This function intented to get binary images of hand feature rects.
// --------------------------------------------------------------------------
void produceBinaries(MyImage *m)
{
    Scalar lowerBound;
    Scalar upperBound;
    for(int i = 0; i < SAMPLE_NUM; i++)
    {
        normalizeColors();
        lowerBound = Scalar( avgColor[i][0] - c_lower[i][0] , avgColor[i][1] - c_lower[i][1], avgColor[i][2] - c_lower[i][2] );
        upperBound = Scalar( avgColor[i][0] + c_upper[i][0] , avgColor[i][1] + c_upper[i][1], avgColor[i][2] + c_upper[i][2] );
        m->bwList.push_back(Mat(m->srcLR.rows, m->srcLR.cols, CV_8U));
        inRange(m->srcLR, lowerBound, upperBound, m->bwList[i]);
    }

    m->bwList[0].copyTo(m->bw);
    for(int i = 1; i < SAMPLE_NUM; i++)
    {
        m->bw += m->bwList[i];
    }

    medianBlur(m->bw, m->bw, 7);
}

// --------------------------------------------------------------------------
// Function Name : myDrawContours
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- MyImage *m : camera structure for hand detection -*, *- HandDetector *hg : detector for hand detection -*
// This function intented to draw contour on frame.
// --------------------------------------------------------------------------
void myDrawContours(MyImage *m, HandDetector *hg)
{
    drawContours(m->dst, hg->hullP, hg->cIdx, cv::Scalar(200, 0, 0), 2, 8, vector<Vec4i>(), 0, Point());
    vector<Vec4i>::iterator d = hg->defects[hg->cIdx].begin();

    vector<Mat> channels;
    Mat result;
    for(int i = 0; i < 3;i++)
        channels.push_back(m->bw);

    merge(channels, result);
    drawContours(result, hg->hullP, hg->cIdx, cv::Scalar(0, 0, 250), 10, 8, vector<Vec4i>(), 0, Point());

    while( d != hg->defects[hg->cIdx].end() )
    {
        Vec4i& v = (*d);
        int faridx = v[2];
        Point ptFar(hg->contours[hg->cIdx][faridx] );
        circle( result, ptFar, 9, Scalar(0, 205, 0), 5 );
        d++;
    }
}

// --------------------------------------------------------------------------
// Function Name : findBiggestContour
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- vector<vector<Point> > contours : contour data -*
// Output Data : *- index of biggest contour, return type is int -*
// This function intented to get biggest contour index.
// --------------------------------------------------------------------------
int findBiggestContour(vector<vector<Point> > contours)
{
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 150;
    for (int i = 0; i < contours.size(); i++)
    {
        if(contours[i].size() > sizeOfBiggestContour)
        {
            sizeOfBiggestContour = contours[i].size();
            indexOfBiggestContour = i;
        }
    }

    handCandidateIdx = indexOfBiggestContour;
    return indexOfBiggestContour;
}

// --------------------------------------------------------------------------
// Function Name : makeFirstContours
// Auther : chen chen
// Data : 2016-02-15
// Input Data : *- MyImage *m : camera structure for hand detection -*, *- HandDetector *hg : detector for hand detection -*
// Output Data : *- if make first contour, return true, return type is bool -*
// This function intented to get biggest contour index.
// --------------------------------------------------------------------------
bool makeFirstContours(MyImage *m, HandDetector* hg){
    Mat aBw;
    pyrUp(m->bw, m->bw);
    m->bw.copyTo(aBw);

    findContours(aBw, hg->contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    hg->initVectors();
    hg->cIdx = findBiggestContour(hg->contours);
    if(hg->cIdx != -1)
    {
        hg->bRect = boundingRect(Mat(hg->contours[hg->cIdx]));
        convexHull(Mat(hg->contours[hg->cIdx]), hg->hullP[hg->cIdx], false, true);
        convexHull(Mat(hg->contours[hg->cIdx]), hg->hullI[hg->cIdx], false, false);
        approxPolyDP( Mat(hg->hullP[hg->cIdx]), hg->hullP[hg->cIdx], 18, true );
        if(hg->contours[hg->cIdx].size()>3 )
        {
            convexityDefects(hg->contours[hg->cIdx], hg->hullI[hg->cIdx], hg->defects[hg->cIdx]);
            hg->eleminateDefects(m);
        }

        bool isHand = hg->detectIfHand();
        if(isHand)
        {
            hg->getFingerTips(m);
            hg->drawFingerTips(m);
            myDrawContours(m,hg);
            return true;
        }
    }

    return false;
}

// processing thread
ProcessThread::ProcessThread(QObject* parent)
    : QObject(parent)
{
    
}

ProcessThread::~ProcessThread()
{
    
}

void ProcessThread::start()
{
    g_bProcessing = true;

	CascadeClassifier face_cascade;

	if((m_nProcessMode == METHOD_TRACKING)||(m_nProcessMode == METHOD_COUNTING)||(m_nProcessMode == METHOD_SURVEILLANCE))
	{
		createBgEngine();
	}
	else
	{
		if(!face_cascade.load("face.xml"))
		{
			g_bProcessing = false;
			isFinished();
			return;
		}
	}	

	if(m_nProcessMode == METHOD_HAND)
	{
		webcam = new MyImage(0);
		if(!webcam->cap.isOpened())
		{
			cout << "Cannot Find Camera!\n";
			return;
		}

		createCornerKernels();

		for(int i = 0; i < SAMPLE_NUM; i++)
		{
			c_lower[i][0] = 12;
			c_upper[i][0] = 7;
			c_lower[i][1] = 30;
			c_upper[i][1] = 40;
			c_lower[i][2] = 80;
			c_upper[i][2] = 80;
		}

		HandDetector hg;
		waitForPalmCover(webcam);
		average(webcam);

		for(;;)
		{
			vector<Rect> faces;
			Mat nonFace_img;
			bool bFaceDetected;
			handCandidateIdx = -1;
			hg.frameNumber++;
			webcam->cap.operator >> (m_currentFrame);

			if(m_currentFrame.empty())
				break;

			process_img = m_currentFrame.clone();
			flip(m_currentFrame, process_img, 1);

			process_img = m_currentFrame.clone();
			vector<Mat> rgbChannels(3);
			split(process_img, rgbChannels);
			grey_img = rgbChannels[2];
			face_region = Rect(0, 0, 0, 0);

			int maxarea = 10;
			int maxareai = -1;

			// detect face regions
			face_cascade.detectMultiScale(grey_img, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
			for (int i = 0; i < (int)faces.size(); i++)
			{
				maxareai = (faces[i].area() > maxarea) ? i : maxareai;
				maxarea = (faces[i].area() > maxarea) ? faces[i].area() : maxarea;
			}

			if((faces.size() != 0)&&(maxareai > -1))
			{
				face_region = faces[maxareai];
				bFaceDetected = true;
			}
			else
				bFaceDetected = false;

			Mat maskImg = Mat(process_img.size(), CV_8UC1);
			maskImg = Scalar::all(255);

			for (int i = 0; i < (int)faces.size(); i++)
			{
				rectangle(maskImg, faces[i], Scalar(0), -1);
			}

			nonFace_img = Mat(process_img.size(), process_img.type());
			nonFace_img = Scalar::all(0);
			process_img.copyTo(nonFace_img, maskImg);

			rectangle(process_img, face_region, Scalar(0, 0, 255));

			if(bFaceDetected)
			{
				webcam->dst = process_img;
				webcam->nonFaceSrc = nonFace_img.clone();
				pyrDown(webcam->nonFaceSrc, webcam->srcLR);
				blur(webcam->srcLR, webcam->srcLR, Size(3, 3));
				cvtColor(webcam->srcLR, webcam->srcLR,  CV_BGR2HLS);
				produceBinaries(webcam);
				cvtColor(webcam->srcLR, webcam->srcLR, CV_HLS2BGR);

				// check hand
				if(makeFirstContours(webcam, &hg))
				{
					hg.getFingerNumber(webcam);
				}
			}	

			if(process_img.type() == CV_8UC1)
				cvtColor(process_img, process_img, CV_GRAY2RGB);
			else
				cvtColor(process_img, process_img, CV_BGR2RGB);

			QImage result_img = QImage((const unsigned char*)(process_img.data), process_img.cols, process_img.rows, QImage::Format_RGB888);
			m_pVideoView->setPixmap(QPixmap::fromImage(result_img.scaled(m_pVideoView->width(), m_pVideoView->height())));
		}
	}
	else
	{
		if(m_nInputType == INPUT_WEBCAM)
		{
			m_capture = new VideoCapture(0);
			if(!m_capture->isOpened())
			{
				return;
			}        
		}
		else
		{
			QByteArray byteVideoName = m_strVideoName.toLocal8Bit();
			char* szVideoName = byteVideoName.data();
			m_capture = new VideoCapture(szVideoName);
		}

		FLANDMARK_Model *landmark_model;
		int *bbox; double *landmarks;

		if((m_nProcessMode == METHOD_FACE)||(m_nProcessMode == METHOD_GAZE))
		{
			landmark_model = flandmark_init("flandmark_model.dat");
			if(landmark_model == 0)
			{
				g_bProcessing = false;
				isFinished();
				return;
			}

			bbox = (int*)malloc(4*sizeof(int));
			landmarks = (double*)malloc(2*landmark_model->data.options.M*sizeof(double));
		}		

		m_nFrameNum = 0;

		for (;;)
		{
			if(!g_bProcessing)
				break;

			m_capture->operator >>(m_currentFrame);

			if (m_currentFrame.empty())
				break;

			process_img = m_currentFrame.clone();

			if((m_nProcessMode == METHOD_TRACKING)||(m_nProcessMode == METHOD_COUNTING)||(m_nProcessMode == METHOD_SURVEILLANCE))
			{
				backgroundModeling(process_img);
				detectObjects();

				QString strFrameNumber = "Frame Number : " + QString::number(m_nFrameNum+1);
				QByteArray byteFrameNumber = strFrameNumber.toLocal8Bit();
				char* szFrameNumber = byteFrameNumber.data();
				putText(process_img, szFrameNumber, Point(10, process_img.rows - 100), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 2);

				if(m_nProcessMode == METHOD_TRACKING)
					kalman_tracking();
				else if(m_nProcessMode == METHOD_COUNTING)
				{
					meanshift_tracking();
					checkState();
				}
			}
			else
			{
				vector<Rect> faces;
				Mat nonFace_img;
				bool bFaceDetected;

				process_img = m_currentFrame.clone();
				vector<Mat> rgbChannels(3);
				split(process_img, rgbChannels);
				grey_img = rgbChannels[2];
				face_region = Rect(0, 0, 0, 0);

				int maxarea = 10;
				int maxareai = -1;

				// detect face regions
				face_cascade.detectMultiScale(grey_img, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
				for (int i = 0; i < (int)faces.size(); i++)
				{
					maxareai = (faces[i].area() > maxarea) ? i : maxareai;
					maxarea = (faces[i].area() > maxarea) ? faces[i].area() : maxarea;
				}

				if((faces.size() != 0)&&(maxareai > -1))
				{
					face_region = faces[maxareai];
					bFaceDetected = true;
				}
				else
					bFaceDetected = false;

				if(bFaceDetected)
				{
					bbox[0] = face_region.x;
					bbox[1] = face_region.y;
					bbox[2] = face_region.x + face_region.width;
					bbox[3] = face_region.y + face_region.height;

					flandmark_detect(grey_img, bbox, landmark_model, landmarks);

					rectangle(process_img, Point(bbox[0], bbox[1]), Point(bbox[2], bbox[3]), Scalar(0, 0, 255));
					rectangle(process_img, Point(landmark_model->bb[0], landmark_model->bb[1]), Point(landmark_model->bb[2], landmark_model->bb[3]), Scalar(255, 0, 0));

					if(m_nProcessMode == METHOD_FACE)
					{
						circle(process_img, Point(landmarks[0], landmarks[1]), 3, Scalar(255, 0, 0), -1);

						for(int i = 1; i < 8; i++)
						{
							circle(process_img, Point(landmarks[i*2], landmarks[i*2+1]), 3, Scalar(0, 0, 255), -1);
						}
					}
					else
					{
						Mat faceROI = grey_img(face_region);
						Mat faceOrg = process_img(face_region);

						//-- Find eye regions and draw them
						int eye_region_width = face_region.width * (35.0/100.0);
						int eye_region_height = face_region.width * (30.0/100.0);
						int eye_region_top = face_region.height * (25.0/100.0);
						Rect leftEyeRegion(face_region.width*(13.0/100.0),
							eye_region_top,eye_region_width,eye_region_height);
						Rect rightEyeRegion(face_region.width - eye_region_width - face_region.width*(13.0/100.0),
							eye_region_top,eye_region_width,eye_region_height);

						//-- Find Eye Centers
						Point leftPupil = findEyeCenter(faceROI,leftEyeRegion);
						Point rightPupil = findEyeCenter(faceROI,rightEyeRegion);

						// get corner regions
						Rect leftRightCornerRegion(leftEyeRegion);
						leftRightCornerRegion.width -= leftPupil.x;
						leftRightCornerRegion.x += leftPupil.x;
						leftRightCornerRegion.height /= 2;
						leftRightCornerRegion.y += leftRightCornerRegion.height / 2;
						Rect leftLeftCornerRegion(leftEyeRegion);
						leftLeftCornerRegion.width = leftPupil.x;
						leftLeftCornerRegion.height /= 2;
						leftLeftCornerRegion.y += leftLeftCornerRegion.height / 2;
						Rect rightLeftCornerRegion(rightEyeRegion);
						rightLeftCornerRegion.width = rightPupil.x;
						rightLeftCornerRegion.height /= 2;
						rightLeftCornerRegion.y += rightLeftCornerRegion.height / 2;
						Rect rightRightCornerRegion(rightEyeRegion);
						rightRightCornerRegion.width -= rightPupil.x;
						rightRightCornerRegion.x += rightPupil.x;
						rightRightCornerRegion.height /= 2;
						rightRightCornerRegion.y += rightRightCornerRegion.height / 2;

						rectangle(faceOrg, leftRightCornerRegion, Scalar(0, 255, 0), 2);
						rectangle(faceOrg, leftLeftCornerRegion, Scalar(0, 255, 0), 2);
						rectangle(faceOrg, rightLeftCornerRegion, Scalar(0, 255, 0), 2);
						rectangle(faceOrg, rightRightCornerRegion, Scalar(0, 255, 0), 2);

						// change eye centers to face coordinates
						rightPupil.x += rightEyeRegion.x;
						rightPupil.y += rightEyeRegion.y;
						leftPupil.x += leftEyeRegion.x;
						leftPupil.y += leftEyeRegion.y;

						// draw eye centers
						circle(faceOrg, rightPupil, 5, Scalar(255, 255, 0), -1);
						circle(faceOrg, leftPupil, 5, Scalar(255, 255, 0), -1);
					}
				}            
			}

			if(process_img.type() == CV_8UC1)
				cvtColor(process_img, process_img, CV_GRAY2RGB);
			else
				cvtColor(process_img, process_img, CV_BGR2RGB);

			QImage result_img = QImage((const unsigned char*)(process_img.data), process_img.cols, process_img.rows, QImage::Format_RGB888);
			m_pVideoView->setPixmap(QPixmap::fromImage(result_img.scaled(m_pVideoView->width(), m_pVideoView->height())));

			m_nFrameNum++;
		}

		if((m_nProcessMode == METHOD_TRACKING)||(m_nProcessMode == METHOD_COUNTING)||(m_nProcessMode == METHOD_SURVEILLANCE))
			deleteBgEngine();
	}

    g_bProcessing = false;
    isFinished();
    m_capture->release();
}

// background modeling
void ProcessThread::createBgEngine()
{
    m_pFrameDiffEngine = new FrameDiff_Engine;
    m_pAMBPEngine = new AMBP_engine;

	if(m_nProcessMode == METHOD_TRACKING)
		kalman_tracker = new KalmanTracker(0.2, 0.5, 60.0, 10, 10);
	else if(m_nProcessMode == METHOD_COUNTING)
	{
		m_nObjectMaxID = 0;
		m_nUpCnt = 0;
		m_nDownCnt = 0;

		m_vecFirstDatas.clear();
		m_vecProcessedIDs.clear();
	}
}

void ProcessThread::deleteBgEngine()
{
    delete(m_pFrameDiffEngine);
    delete(m_pAMBPEngine);

	if(m_nProcessMode == METHOD_TRACKING)
		delete(kalman_tracker);
}

void ProcessThread::backgroundModeling(Mat procImg)
{
    m_pFrameDiffEngine->process(procImg, framediff_fg);
    if(framediff_fg.empty())
    {
        framediff_fg = Mat(procImg.size(), CV_8UC1);
        framediff_fg = Scalar::all(0);
    }
    if(framediff_fg.type() == CV_8UC3) cvtColor(framediff_fg, framediff_fg, CV_BGR2GRAY);

    Mat procAmbp;
    if(procImg.type() == CV_8UC1)
        cvtColor(procImg, procAmbp, CV_GRAY2BGR);
    else
        procAmbp = procImg.clone();

    m_pAMBPEngine->process(procAmbp, ambp_fg, m_currentBg, m_histogram);
    if(ambp_fg.empty())
    {
        ambp_fg = Mat(procImg.size(), CV_8UC1);
        ambp_fg = Scalar::all(0);
    }

    if(ambp_fg.type() == CV_8UC3) cvtColor(ambp_fg, ambp_fg, CV_BGR2GRAY);
    bitwise_and(ambp_fg, framediff_fg, m_currentFg);
    removeNoise();
}

void ProcessThread::removeNoise()
{
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(m_currentFg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    Mat img_foreground = Mat(m_currentFg.size(), CV_8UC1);
    img_foreground = Scalar::all(0);
    if(contours.size() != 0)
    {
        for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        {
            Rect contourRect = boundingRect(contours[idx]);
            if((contourRect.width < OBJECT_NOISE_SIZE)||(contourRect.height < OBJECT_NOISE_SIZE))
                continue;

            drawContours(img_foreground, contours, idx, Scalar(255), CV_FILLED);
        }
    }

    m_currentFg = img_foreground.clone();
}

// object detecting
void ProcessThread::detectObjects()
{
	if(m_nProcessMode == METHOD_TRACKING)
	{
		m_kalmanObjectCenters.clear();
		m_kalmanObjectRects.clear();
	}	
	else if(m_nProcessMode == METHOD_COUNTING)
	{
		releaseObjectBuffer(&m_currentObjects);
		m_nDoorPosition = process_img.rows / 2;
		line(process_img, Point(0, m_nDoorPosition), Point(process_img.cols, m_nDoorPosition), Scalar(0, 0, 255), 2);
	}
	else
	{
		m_nSurveilanceArea = Rect(100, 100, process_img.cols-200, process_img.rows-200);
		rectangle(process_img, m_nSurveilanceArea, Scalar(0, 0, 255), 2);
	}

	Mat procImg = m_currentFg.clone();
	if(procImg.type() == CV_8UC3)
		cvtColor(procImg, procImg, CV_BGR2GRAY);

	if(m_nFrameNum < 3)
		return;

	erode(procImg, procImg, Mat());
	dilate(procImg, procImg, Mat(), Point(-1, -1), 2);

	findContours(procImg, m_contours, m_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	int nSurveilanceCnt = 0;
	if( m_contours.size() != 0 )
	{
		int idx = 0;
		for( ; idx >= 0; idx = m_hierarchy[idx][0] )
		{
			Rect contourRect = boundingRect(m_contours[idx]);
			if ((contourRect.width < OBJECT_MIN_SIZE)||(contourRect.height < OBJECT_MIN_SIZE))
				continue;

			rectangle(process_img, contourRect, Scalar(0, 0, 255), 2);

			if(m_nProcessMode == METHOD_TRACKING)
			{
				m_kalmanObjectCenters.push_back(Point(contourRect.x+contourRect.width/2, contourRect.y+contourRect.height/2));
				m_kalmanObjectRects.push_back(contourRect);
			}
			else if(m_nProcessMode == METHOD_COUNTING)
			{
				m_currentObjects.objectCnt++;
				m_currentObjects.objectRects.push_back(contourRect);
				m_currentObjects.positions.push_back(Point(contourRect.x+contourRect.width/2, contourRect.y+contourRect.height/2));				
			}
			else
			{
				Point currentPt = Point(contourRect.x+contourRect.width/2, contourRect.y+contourRect.height/2);
				if((currentPt.x > m_nSurveilanceArea.x)&&(currentPt.x < m_nSurveilanceArea.x+m_nSurveilanceArea.width)&&(currentPt.y > m_nSurveilanceArea.y)&&(currentPt.y < m_nSurveilanceArea.y+m_nSurveilanceArea.height))
					nSurveilanceCnt++;
			}
		}
	}

	if(m_nProcessMode == METHOD_SURVEILLANCE)
	{
		setSurveilanceCnt(nSurveilanceCnt);
		QString strFrameNumber = "Object Count : " + QString::number(nSurveilanceCnt);
		QByteArray byteFrameNumber = strFrameNumber.toLocal8Bit();
		char* szFrameNumber = byteFrameNumber.data();
		putText(process_img, szFrameNumber, Point(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 2);
	}
}

void ProcessThread::releaseObjectBuffer(FRAME_OBJECTS *buffer)
{
	buffer->objectCnt = 0;
	buffer->objectIDs.clear();
	buffer->objectRects.clear();
	buffer->positions.clear();
}

/* kalman tracking */
void ProcessThread::kalman_tracking()
{
	if(m_kalmanObjectRects.size() > 0)
	{
		kalman_tracker->Update(m_kalmanObjectCenters);

		for (int i = 0; i < kalman_tracker->tracks.size(); i++)
		{
			QString strID = QString::number(kalman_tracker->tracks[i]->track_id+1);
			QByteArray byteID = strID.toLocal8Bit();
			char* szID = byteID.data();
			putText(process_img, szID, kalman_tracker->tracks[i]->prediction, FONT_HERSHEY_COMPLEX_SMALL, 1, colors[i%8], 2);
		}
	}

	QString strFrameNumber = "Frame Number : " + QString::number(m_nFrameNum+1);
	QByteArray byteFrameNumber = strFrameNumber.toLocal8Bit();
	char* szFrameNumber = byteFrameNumber.data();
	putText(process_img, szFrameNumber, Point(10, process_img.rows - 100), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 2);
}

/* meanshift tracking */
void ProcessThread::meanshift_tracking()
{
	if (m_currentObjects.objectCnt > 0)
	{
		getMeanshiftObjectID();

		m_thirdObjects = m_lastObjects;
		m_lastObjects = m_beforeObjects;
		m_beforeObjects = m_currentObjects;
	}
	else
	{
		m_thirdObjects = m_lastObjects;
		m_lastObjects = m_beforeObjects;
		releaseObjectBuffer(&m_beforeObjects);
	}

	for (int i = 0; i < m_currentObjects.objectCnt; i++)
	{
		Point position = Point(m_currentObjects.objectRects[i].x + m_currentObjects.objectRects[i].width/2,
			m_currentObjects.objectRects[i].y + m_currentObjects.objectRects[i].height/2);

		rectangle(process_img, m_currentObjects.objectRects[i], Scalar(0, 0, 255), 2);
	}
}

void ProcessThread::getMeanshiftObjectID()
{
	vector<int> currentObjectIDs;
	for (int i = 0; i < m_currentObjects.objectCnt; i++)
	{
		bool isNewObject = true;

		for (int j = 0; j < m_beforeObjects.objectCnt; j++)
		{
			bool bExistID = false;
			Point currentPt = Point(m_currentObjects.objectRects[i].x + m_currentObjects.objectRects[i].width/2,
				m_currentObjects.objectRects[i].y + m_currentObjects.objectRects[i].height);
			Point beforePt = Point(m_beforeObjects.objectRects[j].x + m_beforeObjects.objectRects[j].width/2,
				m_beforeObjects.objectRects[j].y + m_beforeObjects.objectRects[j].height);

			if (((abs(currentPt.x - beforePt.x)) < OBJECT_MOVE_THRESHOLD_X)&&((abs(currentPt.y - beforePt.y)) < OBJECT_MOVE_THRESHOLD_Y))
			{
				for (int k = 0; k < (int)currentObjectIDs.size(); k++)
				{
					if (m_beforeObjects.objectIDs[j] == currentObjectIDs[k])
					{
						bExistID = true;
					}
				}

				if(bExistID)
					continue;

				isNewObject = false;
				currentObjectIDs.push_back(m_beforeObjects.objectIDs[j]);
				m_currentObjects.objectIDs.push_back(m_beforeObjects.objectIDs[j]);
				break;
			}
		}

		if (isNewObject)
		{
			for (int j = 0; j < m_lastObjects.objectCnt; j++)
			{
				bool bExistID = false;
				Point currentPt = Point(m_currentObjects.objectRects[i].x + m_currentObjects.objectRects[i].width/2,
					m_currentObjects.objectRects[i].y + m_currentObjects.objectRects[i].height);
				Point beforePt = Point(m_lastObjects.objectRects[j].x + m_lastObjects.objectRects[j].width/2,
					m_lastObjects.objectRects[j].y + m_lastObjects.objectRects[j].height);

				if (((abs(currentPt.x - beforePt.x)) < OBJECT_MOVE_THRESHOLD_X)&&((abs(currentPt.y - beforePt.y)) < OBJECT_MOVE_THRESHOLD_Y))
				{
					for (int k = 0; k < (int)currentObjectIDs.size(); k++)
					{
						if (m_lastObjects.objectIDs[j] == currentObjectIDs[k])
						{
							bExistID = true;
						}
					}

					if(bExistID)
						continue;

					isNewObject = false;
					currentObjectIDs.push_back(m_lastObjects.objectIDs[j]);
					m_currentObjects.objectIDs.push_back(m_lastObjects.objectIDs[j]);
					break;
				}
			}
		}

		if (isNewObject)
		{
			for (int j = 0; j < m_thirdObjects.objectCnt; j++)
			{
				bool bExistID = false;
				Point currentPt = Point(m_currentObjects.objectRects[i].x + m_currentObjects.objectRects[i].width/2,
					m_currentObjects.objectRects[i].y + m_currentObjects.objectRects[i].height);
				Point beforePt = Point(m_thirdObjects.objectRects[j].x + m_thirdObjects.objectRects[j].width/2,
					m_thirdObjects.objectRects[j].y + m_thirdObjects.objectRects[j].height);

				if (((abs(currentPt.x - beforePt.x)) < OBJECT_MOVE_THRESHOLD_X)&&((abs(currentPt.y - beforePt.y)) < OBJECT_MOVE_THRESHOLD_Y))
				{
					for (int k = 0; k < (int)currentObjectIDs.size(); k++)
					{
						if (m_thirdObjects.objectIDs[j] == currentObjectIDs[k])
						{
							bExistID = true;
						}
					}

					if(bExistID)
						continue;

					isNewObject = false;
					currentObjectIDs.push_back(m_thirdObjects.objectIDs[j]);
					m_currentObjects.objectIDs.push_back(m_thirdObjects.objectIDs[j]);
					break;
				}
			}
		}

		if (isNewObject)
		{
			m_nObjectMaxID++;
			m_currentObjects.objectIDs.push_back(m_nObjectMaxID);

			OBJECT object;
			object.objectIDS = m_nObjectMaxID;
			object.position = Point(m_currentObjects.objectRects[i].x + m_currentObjects.objectRects[i].width/2,
				m_currentObjects.objectRects[i].y + m_currentObjects.objectRects[i].height);
			m_vecFirstDatas.push_back(object);
		}
	}
}

void ProcessThread::checkState()
{
	for(int i = 0; i < m_currentObjects.objectIDs.size(); i++)
	{
		bool bChecked = false;
		for(int j = 0; j < m_vecProcessedIDs.size(); j++)
		{
			if(m_vecProcessedIDs[j] == m_currentObjects.objectIDs[i])
			{
				bChecked = true;
				break;
			}
		}

		if(bChecked)
			continue;

		if((m_currentObjects.positions[i].y > m_nDoorPosition + 10)&&(m_currentObjects.positions[i].y < m_nDoorPosition + 30))
		{
			int currentId = m_currentObjects.objectIDs[i];
			if(m_vecFirstDatas[currentId-1].position.y < m_nDoorPosition)
			{
				m_vecProcessedIDs.push_back(currentId);
				m_nDownCnt++;
			}
		}

		if((m_currentObjects.positions[i].y > m_nDoorPosition - 30)&&(m_currentObjects.positions[i].y < m_nDoorPosition - 10))
		{
			int currentId = m_currentObjects.objectIDs[i];
			if(m_vecFirstDatas[currentId-1].position.y > m_nDoorPosition)
			{
				m_vecProcessedIDs.push_back(currentId);
				m_nUpCnt++;
			}
		}
	}

	setUpCnt(m_nUpCnt);
	QString strUpCnt = "Up : " + QString::number(m_nUpCnt);
	QByteArray byteUpCnt = strUpCnt.toLocal8Bit();
	char* szUpCnt = byteUpCnt.data();
	putText(process_img, szUpCnt, Point(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 2);

	setDownCnt(m_nDownCnt);
	QString strDownCnt = "Down : " + QString::number(m_nDownCnt);
	QByteArray byteDownCnt = strDownCnt.toLocal8Bit();
	char* szDownCnt = byteDownCnt.data();
	putText(process_img, szDownCnt, Point(10, 100), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 2);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_nMinSize = 10;
    m_nMaxSize = 50000;
    m_nSegmentThreshold = 0;
    m_nInputType = INPUT_WEBCAM;
    m_nProcessMode = METHOD_TRACKING;

    InitUI();

	m_pProcessThread = NULL;
	m_pThread = NULL;
}

MainWindow::~MainWindow()
{
	if(g_bProcessing)
		g_bProcessing = false;

	if(m_pProcessThread != NULL)
		delete(m_pProcessThread);

	if(m_pThread != NULL)
		delete(m_pThread);

    delete ui;
}

void MainWindow::on_webcam_radio_clicked(bool checked)
{
    if(checked)
    {
        m_nInputType = INPUT_WEBCAM;
        ui->process_btn->setDisabled(false);
        ui->video_open_btn->setDisabled(true);
        ui->open_video_label->setDisabled(true);
    }
}

void MainWindow::on_video_radio_clicked(bool checked)
{
    if(checked)
    {
        m_nInputType = INPUT_VIDEO;
        ui->video_open_btn->setDisabled(false);
        ui->open_video_label->setDisabled(false);
		ui->process_btn->setDisabled(true);

        if(!ui->open_video_label->text().isEmpty())
            ui->process_btn->setDisabled(false);
    }
}

void MainWindow::on_video_open_btn_clicked()
{
    ui->open_video_label->clear();
    QString strOpenVideoFile = QFileDialog::getOpenFileName( this, tr("Open File"), QDir::currentPath(), tr("avi files (*.avi)"));

    if(!strOpenVideoFile.isEmpty())
    {
        ui->open_video_label->setText(strOpenVideoFile);
        ui->process_btn->setDisabled(false);
    }
    else
    {
        ui->process_btn->setDisabled(true);
    }
}

void MainWindow::on_process_btn_clicked()
{
	if(m_pProcessThread != NULL)
		delete(m_pProcessThread);

	if(m_pThread != NULL)
		delete(m_pThread);

	if(m_nInputType == INPUT_WEBCAM)
	{
		VideoCapture camera(0);

		if(!camera.isOpened())
		{
			QMessageBox::warning(this, tr("Warning"), tr("No Camera!"));
			return;
        }
	}
    else
    {
        if(ui->open_video_label->text() == "")
        {
            QMessageBox::warning(this, tr("Warning"), tr("Please Open Video!"));
            return;
        }
    }

    m_pProcessThread = new ProcessThread;
    m_pThread = new QThread;

    m_pProcessThread->m_pVideoView = ui->show_video_view;

    m_pProcessThread->m_strVideoName = ui->open_video_label->text();
    m_pProcessThread->m_nInputType = m_nInputType;
    m_pProcessThread->m_nProcessMode = m_nProcessMode;
    m_pProcessThread->m_nMaxSize = m_nMaxSize;
    m_pProcessThread->m_nMinSize = m_nMinSize;
    m_pProcessThread->m_nSegmentThreshold = m_nSegmentThreshold;

	m_pProcessThread->moveToThread(m_pThread);

	connect(m_pThread, SIGNAL(started()), m_pProcessThread, SLOT(start()));
	connect(m_pProcessThread, SIGNAL(finished()), m_pThread, SLOT(quit()));
	connect(m_pProcessThread, SIGNAL(finished()), m_pProcessThread, SLOT(deleteLater()));
	connect(m_pProcessThread, SIGNAL(finished()), m_pThread, SLOT(deleteLater()));
    connect(m_pProcessThread, SIGNAL(requestFinish()), this, SLOT(on_thread_finished()));
	connect(m_pProcessThread, SIGNAL(requestSurveilanceCnt(int)), this, SLOT(on_surveilace_request(int)));
	connect(m_pProcessThread, SIGNAL(requestDownCnt(int)), this, SLOT(on_down_request(int)));;
	connect(m_pProcessThread, SIGNAL(requestUpCnt(int)), this, SLOT(on_up_request(int)));

	m_pThread->start();
	
	ui->process_btn->hide();
    ui->stop_btn->show();
    ui->open_video_label->clear();
	ui->video_group->setDisabled(false);
	ui->information_group->setDisabled(false);
    ui->segment_setting_group->setDisabled(false);
    ui->select_video_group->setDisabled(true);
    ui->process_method_combo->setDisabled(true);
}

void MainWindow::on_stop_btn_clicked()
{
	g_bProcessing = false;
	ui->information_group->setDisabled(true);
	ui->segment_setting_group->setDisabled(true);
	ui->stop_btn->hide();
}

void MainWindow::on_min_size_slider_valueChanged(int value)
{
    m_nMinSize = value;

    if(m_nMinSize > m_nMaxSize)
    {
        m_nMaxSize = m_nMinSize;
        ui->max_size_slider->setValue(m_nMaxSize);
    }
    ui->size_value_label->setText(QString::number(m_nMinSize) + " ~ " + QString::number(m_nMaxSize));
	m_pProcessThread->m_nMaxSize = m_nMaxSize;
	m_pProcessThread->m_nMinSize = m_nMinSize;
}

void MainWindow::on_max_size_slider_valueChanged(int value)
{
    m_nMaxSize = value;

    if(m_nMaxSize < m_nMinSize)
    {
        m_nMinSize = m_nMaxSize;
        ui->min_size_slider->setValue(m_nMinSize);
    }
    ui->size_value_label->setText(QString::number(m_nMinSize) + " ~ " + QString::number(m_nMaxSize));
	m_pProcessThread->m_nMaxSize = m_nMaxSize;
	m_pProcessThread->m_nMinSize = m_nMinSize;
}

void MainWindow::on_segment_threshold_slider_valueChanged(int value)
{
    m_nSegmentThreshold = value;
    ui->segment_threshold_label->setText(QString::number(m_nSegmentThreshold));
	m_pProcessThread->m_nSegmentThreshold = m_nSegmentThreshold;
}

void MainWindow::InitUI()
{
    ui->min_size_slider->setValue(m_nMinSize);
	ui->max_size_slider->setValue(m_nMaxSize);
	ui->segment_threshold_slider->setValue(m_nSegmentThreshold);
	ui->size_value_label->setText(QString::number(m_nMinSize) + " ~ " + QString::number(m_nMaxSize));
	ui->segment_threshold_label->setText(QString::number(m_nSegmentThreshold));

    ui->webcam_radio->setChecked(true);
    ui->video_open_btn->setDisabled(true);
    ui->open_video_label->setDisabled(true);
    ui->information_group->setDisabled(true);
    ui->segment_setting_group->setDisabled(true);
	ui->stop_btn->hide();
    ui->process_btn->show();
}

void MainWindow::on_process_method_combo_activated(int index)
{
    m_nProcessMode = index;

	if(m_nProcessMode == METHOD_HAND)
	{
		ui->open_video_label->clear();
		ui->video_open_btn->setDisabled(true);
		ui->video_radio->setChecked(false);
		ui->webcam_radio->setChecked(true);
		ui->video_radio->setDisabled(true);
		m_nInputType = INPUT_WEBCAM;
	}
	else
	{
		ui->video_open_btn->setDisabled(false);
		ui->video_radio->setDisabled(false);
	}
}

void MainWindow::on_thread_finished()
{
    ui->select_video_group->setDisabled(false);
    ui->information_group->setDisabled(true);
    ui->segment_setting_group->setDisabled(true);
	ui->process_method_combo->setDisabled(false);
    ui->stop_btn->hide();
    ui->process_btn->show();
	ui->down_objects_label->setText("0");
	ui->up_objects_label->setText("0");
	ui->exist_objects_label->setText("0");
	ui->show_video_view->clear();
}

void MainWindow::on_surveilace_request(int value)
{
	ui->exist_objects_label->setText(QString::number(value));
}

void MainWindow::on_down_request(int value)
{
	ui->down_objects_label->setText(QString::number(value));
}

void MainWindow::on_up_request(int value)
{
	ui->up_objects_label->setText(QString::number(value));
}
