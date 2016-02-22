#include "hand_detector.hpp"

// --------------------------------------------------------------------------
// Function Name : MyImage
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
MyImage::MyImage()
{
}

// --------------------------------------------------------------------------
// Function Name : MyImage
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
MyImage::MyImage(int webCamera)
{
    cameraSrc = webCamera;
    cap = VideoCapture(webCamera);
}

// --------------------------------------------------------------------------
// Function Name : My_ROI
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
My_ROI::My_ROI()
{
    upper_corner = Point(0,0);
    lower_corner = Point(0,0);
}

// --------------------------------------------------------------------------
// Function Name : My_ROI
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
My_ROI::My_ROI(Point u_corner, Point l_corner, Mat src)
{
    upper_corner = u_corner;
    lower_corner = l_corner;
    color = Scalar(0,255,0);
    border_thickness = 2;
    roi_ptr = src(Rect(u_corner.x, u_corner.y, l_corner.x-u_corner.x, l_corner.y-u_corner.y));
}

// --------------------------------------------------------------------------
// Function Name : draw_rectangle
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void My_ROI::draw_rectangle(Mat src)
{
    rectangle(src, upper_corner, lower_corner, color, border_thickness);
}

// --------------------------------------------------------------------------
// Function Name : HandDetector
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
HandDetector::HandDetector()
{
	frameNumber = 0;
	nrNoFinger = 0;
	fontFace = FONT_HERSHEY_PLAIN;
}

// --------------------------------------------------------------------------
// Function Name : initVectors
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::initVectors()
{
	hullI = vector<vector<int> >(contours.size());
	hullP = vector<vector<Point> >(contours.size());
	defects = vector<vector<Vec4i> > (contours.size());	
}

// --------------------------------------------------------------------------
// Function Name : analyzeContours
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::analyzeContours()
{
	bRect_height = bRect.height;
	bRect_width = bRect.width;
}

// --------------------------------------------------------------------------
// Function Name : detectIfHand
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
bool HandDetector::detectIfHand()
{
	analyzeContours();
	double h = bRect_height; 
	double w = bRect_width;
	isHand = true;
	if(fingerTips.size() > 5 )
		isHand = false;
	else if(h == 0 || w == 0)
		isHand = false;
	else if(h/w > 4 || w/h > 4)
		isHand = false;	
	else if(bRect.x < 20)
		isHand = false;

	return isHand;
}

// --------------------------------------------------------------------------
// Function Name : distanceP2P
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
float HandDetector::distanceP2P(Point a, Point b)
{
    float d = sqrt(fabs( std::pow((double)a.x-b.x, 2) + std::pow((double)(a.y-b.y), 2))) ;
	return d;
}

// --------------------------------------------------------------------------
// Function Name : removeRedundantFingerTips
// Auther : chen chen
// Data : 2016-02-15
// remove fingertips that are too close to each other
// --------------------------------------------------------------------------
void HandDetector::removeRedundantFingerTips()
{
	vector<Point> newFingers;
    for(int i = 0; i < fingerTips.size(); i++)
	{
        for(int j = i; j < fingerTips.size(); j++){
            if(!distanceP2P(fingerTips[i], fingerTips[j]) < 10 && i != j)
			{
				newFingers.push_back(fingerTips[i]);	
				break;
			}	
		}	
	}

	fingerTips.swap(newFingers);
}

// --------------------------------------------------------------------------
// Function Name : computeFingerNumber
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::computeFingerNumber()
{
	std::sort(fingerNumbers.begin(), fingerNumbers.end());
	int frequentNr;	
	int thisNumberFreq = 1;
	int highestFreq = 1;
	frequentNr = fingerNumbers[0];
	for(int i = 1; i < fingerNumbers.size(); i++)
	{
		if(fingerNumbers[i-1] != fingerNumbers[i])
		{
			if(thisNumberFreq > highestFreq)
			{
				frequentNr = fingerNumbers[i-1];	
				highestFreq = thisNumberFreq;
			}

			thisNumberFreq = 0;	
		}

		thisNumberFreq++;	
	}

	if(thisNumberFreq > highestFreq)
		frequentNr = fingerNumbers[fingerNumbers.size()-1];	

    mostFrequentFingerNumber = frequentNr;
}

// --------------------------------------------------------------------------
// Function Name : addFingerNumberToVector
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::addFingerNumberToVector()
{
	int i = fingerTips.size();	
	fingerNumbers.push_back(i);
}

// --------------------------------------------------------------------------
// Function Name : getFingerNumber
// Auther : chen chen
// Data : 2016-02-15
// calculate most frequent numbers of fingers over 20 frames
// --------------------------------------------------------------------------
int HandDetector::getFingerNumber(MyImage *m)
{
	int fingerCount = 0;
	removeRedundantFingerTips();
    if(bRect.height > m->dst.rows/2 && nrNoFinger > 12 && isHand )
	{
        numberColor = Scalar(0, 200, 0);
		addFingerNumberToVector();
		if(frameNumber > 12)
		{
			nrNoFinger = 0;
			frameNumber = 0;	
			computeFingerNumber();	
			numbers2Display.push_back(mostFrequentFingerNumber);
			fingerCount = fingerNumbers.size();
			fingerNumbers.clear();
		}		
		else
		{
			frameNumber++;
		}
	}
	else
	{
		nrNoFinger++;
		numberColor = Scalar(200,200,200);
	}

	return fingerTips.size();
}

// --------------------------------------------------------------------------
// Function Name : getAngle
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
float HandDetector::getAngle(Point s, Point f, Point e)
{
	float l1 = distanceP2P(f,s);
	float l2 = distanceP2P(f,e);
	float dot = (s.x-f.x)*(e.x-f.x) + (s.y-f.y)*(e.y-f.y);
	float angle = acos(dot/(l1*l2));
    angle = angle * 180/PI;

	return angle;
}

// --------------------------------------------------------------------------
// Function Name : eleminateDefects
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::eleminateDefects(MyImage *m)
{
	int tolerance =  bRect_height/5;
	float angleTol = 95;
	vector<Vec4i> newDefects;
	int startidx, endidx, faridx;
	vector<Vec4i>::iterator d = defects[cIdx].begin();
	while( d != defects[cIdx].end())
	{
   	    Vec4i& v = (*d);
	    startidx = v[0]; 
		Point ptStart(contours[cIdx][startidx] );
   		endidx = v[1]; 
		Point ptEnd(contours[cIdx][endidx] );
  	    faridx = v[2]; 
		Point ptFar(contours[cIdx][faridx] );
		 
		if(distanceP2P(ptStart, ptFar) > tolerance && distanceP2P(ptEnd, ptFar) > tolerance && getAngle(ptStart, ptFar, ptEnd  ) < angleTol )
		{
            if((ptEnd.y < (bRect.y + bRect.height -bRect.height/4)) && (ptStart.y < (bRect.y + bRect.height -bRect.height/4)))
			{
                newDefects.push_back(v);
            }
		}	

		d++;
	}

	nrOfDefects = newDefects.size();
	defects[cIdx].swap(newDefects);
	removeRedundantEndPoints(defects[cIdx], m);
}

// --------------------------------------------------------------------------
// Function Name : removeRedundantEndPoints
// Auther : chen chen
// Data : 2016-02-15
// remove endpoint of convexity defects if they are at the same fingertip
// --------------------------------------------------------------------------
void HandDetector::removeRedundantEndPoints(vector<Vec4i> newDefects, MyImage *m)
{
    float tolerance = bRect_width/6;
    int startidx, endidx;
	int startidx2, endidx2;
	for(int i = 0;i < newDefects.size(); i++)
	{
		for(int j = i; j < newDefects.size(); j++)
		{
	    	startidx = newDefects[i][0]; 
			Point ptStart(contours[cIdx][startidx] );
	   		endidx = newDefects[i][1]; 
			Point ptEnd(contours[cIdx][endidx] );
	    	startidx2 = newDefects[j][0]; 
			Point ptStart2(contours[cIdx][startidx2] );
	   		endidx2 = newDefects[j][1]; 
			Point ptEnd2(contours[cIdx][endidx2] );
			if(distanceP2P(ptStart, ptEnd2) < tolerance )
			{
				contours[cIdx][startidx] = ptEnd2;
				break;
			}
			if(distanceP2P(ptEnd,ptStart2) < tolerance )
			{
				contours[cIdx][startidx2] = ptEnd;
			}
		}
	}
}

// --------------------------------------------------------------------------
// Function Name : checkForOneFinger
// Auther : chen chen
// Data : 2016-02-15
// convexity defects does not check for one finger so another
// method has to check when there are no convexity defects
// --------------------------------------------------------------------------
void HandDetector::checkForOneFinger(MyImage *m)
{
	int yTol = bRect.height / 6;
	Point highestP;
	highestP.y = m->dst.rows;
	vector<Point>::iterator d = contours[cIdx].begin();
	while( d != contours[cIdx].end() ) 
	{
   	    Point v = (*d);
		if(v.y < highestP.y)
		{
			highestP = v;
		}

		d++;	
	}
	
	int n = 0;
	d = hullP[cIdx].begin();
	while( d != hullP[cIdx].end() ) {
   	    Point v = (*d);
		if(v.y < highestP.y+yTol && v.y != highestP.y && v.x != highestP.x)
			n++;
		d++;	
	}

	if(n == 0)
	{
		fingerTips.push_back(highestP);
	}
}

// --------------------------------------------------------------------------
// Function Name : drawFingerTips
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::drawFingerTips(MyImage *m)
{
    for(int i = 0; i < fingerTips.size(); i++)
    {
        Point p = fingerTips[i];
        stringstream ss;
        ss << i;
        string str = ss.str();
        putText(m->dst, str, p-Point(0,30), fontFace, 1.2f, Scalar(200,200,200), 2);
        circle( m->dst, p, 5, Scalar(100, 255 ,100), 4 );
     }
}

// --------------------------------------------------------------------------
// Function Name : getFingerTips
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void HandDetector::getFingerTips(MyImage *m)
{
    fingerTips.clear();
    int i = 0;
    vector<Vec4i>::iterator d = defects[cIdx].begin();
    while( d != defects[cIdx].end() )
    {
        Vec4i& v = (*d);
        int startidx = v[0];
        Point ptStart(contours[cIdx][startidx] );
        int endidx = v[1];
        Point ptEnd(contours[cIdx][endidx] );

        if(i == 0)
        {
            fingerTips.push_back(ptStart);
            i++;
        }

        fingerTips.push_back(ptEnd);
        d++;
        i++;
    }

    if(fingerTips.size() == 0)
    {
        checkForOneFinger(m);
    }
}
