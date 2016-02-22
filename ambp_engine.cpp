#include "ambp_engine.h"

/*********************************************************************************/
/* Median Binary Pattern   */
/*********************************************************************************/

MedianBinaryPattern::MedianBinaryPattern()
{
  m_ppOrgImgs = NULL;
  m_pRadiuses = NULL;
  m_fRobustWhiteNoise = 3.0f;
  m_pNeigPointsNums = NULL;
  m_pXYShifts = NULL;
  m_pShiftedImg = NULL;
}

MedianBinaryPattern::~MedianBinaryPattern()
{
  FreeMemories();
}

void MedianBinaryPattern::Initialization(IplImage **first_imgs, int imgs_num, int level_num, float *radius, int *neig_pt_num, float robust_white_noise, int type)
{

  m_nImgsNum = imgs_num;

  m_nAMBPLevelNum = level_num;

  m_pRadiuses = new float[m_nAMBPLevelNum];
  m_pNeigPointsNums= new int[m_nAMBPLevelNum];
  m_ppOrgImgs = first_imgs;

  int a, b;
  for ( a = 0 ; a < m_nImgsNum ; a++ ) {
    m_cvImgSize = cvGetSize(first_imgs[a]);

    if ( first_imgs[a]->nChannels > 1 ) {
      printf("Input image channel must be 1!");
      exit(1);
    }
  }

  int tot_neig_pts_num = 0;
  for ( a = 0 ; a < m_nAMBPLevelNum ; a++ ) {
    m_pRadiuses[a] = radius[a];
    m_pNeigPointsNums[a] = neig_pt_num[a];
    tot_neig_pts_num += neig_pt_num[a];
    if ( m_pNeigPointsNums[a] % 2 != 0 ) {
      exit(1);
    }
  }

  m_pShiftedImg = cvCloneImage(m_ppOrgImgs[0]);

  m_pXYShifts = new CvPoint[tot_neig_pts_num];

  m_nMaxShift.x = 0;
  m_nMaxShift.y = 0;
  int shift_idx = 0;
  for ( a = 0 ; a < m_nAMBPLevelNum ; a++ )
    for ( b = 0 ; b < m_pNeigPointsNums[a] ; b++ ) {
      // compute the offset of neig point
      CalNeigPixelOffset(m_pRadiuses[a], m_pNeigPointsNums[a], b, m_pXYShifts[shift_idx].x, m_pXYShifts[shift_idx].y);
      m_nMaxShift.x = MAX(m_nMaxShift.x, m_pXYShifts[shift_idx].x);
      m_nMaxShift.y = MAX(m_nMaxShift.y, m_pXYShifts[shift_idx].y);
      shift_idx++;
    }

    m_fRobustWhiteNoise = robust_white_noise;
}

void MedianBinaryPattern::SetNewImages(IplImage **new_imgs)
{
  m_ppOrgImgs = new_imgs;
}

void MedianBinaryPattern::ComputeAMBP(Mat &frame, PixelAMBPStruct *PAMBP, Mat &MBP_Params, CvRect *roi)
{
  float *dif_pattern;
  float *_dif_pattern;
  PixelAMBPStruct *_PAMBP;
  int data_length;
  float *cur_pattern;

  // allocate memories
  if ( roi )
    data_length = roi->height*roi->width;
  else
    data_length = m_cvImgSize.width*m_cvImgSize.height;

  dif_pattern = new float[data_length];

  for(int y = 2; y < frame.rows - 2; ++y)
  {
	  for(int x = 2; x < frame.cols - 2; ++x)
	  {
		  int taw, window_size = 3;
		  int k_max = window_size / 2;
		  int L = window_size*window_size;

		  int Smax, Smin, Smed;
		  int medianIdx = L/2;
		  vector<int> pixel_vals;
		  vector<int>::iterator iter;

		  if((x-k_max<0)||(x+k_max>frame.cols-1)||(y-k_max<0)||(y+k_max>frame.rows-1))
		  {
			  for(int i = x-1; i <= x+1; i++)
			  {
				  for(int j = y-1; j <= y+1; j++)
				  {
					  int val = (int)frame.at<uchar>(j, i);

					  bool bInsert = false;
					  iter = pixel_vals.begin();

					  for(int valIdx = 0; valIdx < pixel_vals.size(); valIdx++)
					  {
						  if(val < pixel_vals[valIdx])
						  {
							  pixel_vals.insert(iter, val);
							  bInsert = true;
							  break;
						  }

						  iter++;
					  }

					  if(!bInsert||pixel_vals.empty())
						  pixel_vals.push_back(val);
				  }
			  }

			  taw = pixel_vals[4];
			  break;
		  }

		  // calculate median, max, min value.
		  for(int i = x-k_max; i <= x+k_max; i++)
		  {
			  for(int j = y-k_max; j <= y+k_max; j++)
			  {
				  int val = (int)frame.at<uchar>(j, i);

				  bool bInsert = false;
				  iter = pixel_vals.begin();

				  for(int valIdx = 0; valIdx < pixel_vals.size(); valIdx++)
				  {
					  if(val < pixel_vals[valIdx])
					  {
						  pixel_vals.insert(iter, val);
						  bInsert = true;
						  break;
					  }

					  iter++;
				  }

				  if(!bInsert||pixel_vals.empty())
					  pixel_vals.push_back(val);
			  }
		  }

		  Smin = pixel_vals[0];
		  Smax = pixel_vals[L-1];
		  Smed = pixel_vals[medianIdx];


		  int center_val = (int)frame.at<uchar>(y, x);
		  if((center_val > Smin)&&(center_val < Smax))    // If the pixels within the squared region follows the relation shown in above as well,
			  taw = center_val;                           // then the threshold taw can be expressed as, taw=I(i,j)and the whole process act as LBP.
		  else                                            // If the squared region follows the relation in Equation a,
			  taw = Smed;

		  // calculate MBP element. (MBP_pr in paper)
		  unsigned char mbp_pr = 0;

		  if(taw  <= (int)frame.at<uchar>(y-1, x-1)) // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 1;

		  if(taw <= (int)frame.at<uchar>(y-1, x))    // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 2;

		  if(taw <= (int)frame.at<uchar>(y-1, x+1))  // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 4;

		  if(taw <= (int)frame.at<uchar>(y, x-1))    // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 8;

		  if(taw <= (int)frame.at<uchar>(y, x+1))    // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 16; 

		  if(taw <= (int)frame.at<uchar>(y+1, x-1))  // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 32;

		  if(taw <= (int)frame.at<uchar>(y+1, x))    // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 64;

		  if(taw < (int)frame.at<uchar>(y+1, x+1))   // calc s(x) and MBP_pr. In here, x = med - current_val.
			  mbp_pr += 128;

		  MBP_Params.at<uchar>(y, x) = mbp_pr;
	  }
  }

  int img_idx, pt_idx, yx, level;
  int pattern_idx = 0;
  for ( img_idx = 0 ; img_idx < m_nImgsNum ; img_idx++ ) {
    for ( level = 0 ; level < m_nAMBPLevelNum ; level++ ) {
      for ( pt_idx = 0 ; pt_idx < m_pNeigPointsNums[level] ; pt_idx++ ) {

        // computing the shifted image
        CalShiftedImage(m_ppOrgImgs[img_idx], m_pXYShifts[pattern_idx].x, m_pXYShifts[pattern_idx].y, m_pShiftedImg, roi);

        // computing the different binary images
        CalImageDifferenceMap(m_ppOrgImgs[img_idx], m_pShiftedImg, dif_pattern, roi);

        // set the binary values
        _PAMBP = PAMBP;
        _dif_pattern = dif_pattern;

        if ( roi ) {
          int x, y;
          for ( y = 0 ; y < roi->height ; y++ )  {
            _PAMBP = PAMBP + (y+roi->y)*m_cvImgSize.width + roi->x;
            for ( x = 0 ; x < roi->width ; x++ ) {
              cur_pattern = (*_PAMBP++).cur_pattern;
              cur_pattern[pattern_idx] = *_dif_pattern++;
            }
          }
        }
        else {
          for ( yx = 0 ; yx < data_length ; yx++ ) {
            cur_pattern = (*_PAMBP++).cur_pattern;
            cur_pattern[pattern_idx] = *_dif_pattern++;
          }
        }

        pattern_idx++;
      }
    }
  }

  // release memories
  delete [] dif_pattern;
}

void MedianBinaryPattern::FreeMemories()
{
  if ( m_pRadiuses != NULL )
    delete [] m_pRadiuses;
  if ( m_pNeigPointsNums != NULL )
    delete [] m_pNeigPointsNums;
  if ( m_pXYShifts )
    delete [] m_pXYShifts;
  if ( m_pShiftedImg )
    cvReleaseImage(&m_pShiftedImg);

  m_pXYShifts = NULL;
  m_pRadiuses = NULL;
  m_pNeigPointsNums = NULL;
  m_pShiftedImg = NULL;
}

void MedianBinaryPattern::SetShiftedMeshGrid(CvSize img_size, float offset_x, float offset_y, CvMat *grid_map_x, CvMat *grid_map_y)
{
  float *gX = (float*)(grid_map_x->data.ptr);
  float *gY = (float*)(grid_map_y->data.ptr);

  int x, y;
  for ( y = 0 ; y < img_size.height ; y++ ) {
    for ( x = 0 ; x < img_size.width ; x++ ) {
      *gX++ = (float)x + offset_x;
      *gY++ = (float)y + offset_y;
    }
  }
}

void MedianBinaryPattern::CalShiftedImage(IplImage *src, int offset_x, int offset_y, IplImage *dst, CvRect *roi)
{
  CvRect src_roi, dst_roi;
  int roi_width, roi_height;

  if ( roi ) {
    src_roi.x = MAX(offset_x+roi->x, 0);
    src_roi.y = MAX(offset_y+roi->y, 0);

    dst_roi.x = MAX(-(offset_x+roi->x), roi->x);
    dst_roi.y = MAX(-(offset_y+roi->y), roi->y);

    roi_width = MIN(MIN(roi->width+(int)fabsf((float)offset_x), src->width-src_roi.x), dst->width-dst_roi.x);
    roi_height = MIN(MIN(roi->height+(int)fabsf((float)offset_y), src->height-src_roi.y), dst->height-dst_roi.y);

    src_roi.width = roi_width;
    src_roi.height = roi_height;

    dst_roi.width = roi_width;
    dst_roi.height = roi_height;
  }
  else {
    roi_width = src->width-(int)fabsf((float)offset_x);
    roi_height = src->height-(int)fabsf((float)offset_y);

    src_roi.x = MAX(offset_x, 0);
    src_roi.y = MAX(offset_y, 0);
    src_roi.width = roi_width;
    src_roi.height = roi_height;

    dst_roi.x = MAX(-offset_x, 0);
    dst_roi.y = MAX(-offset_y, 0);
    dst_roi.width = roi_width;
    dst_roi.height = roi_height;
  }

  cvSet(dst,cvScalar(0));

  if ( roi_width <= 0 || roi_height <= 0 )
    return;

  cvSetImageROI(src, src_roi);
  cvSetImageROI(dst, dst_roi);
  cvCopy(src, dst);
  cvResetImageROI(src);
  cvResetImageROI(dst);
}

void MedianBinaryPattern::CalNeigPixelOffset(float radius, int tot_neig_pts_num, int neig_pt_idx, int &offset_x, int &offset_y)
{
  offset_x = radius;
  offset_y = -radius;
}

void MedianBinaryPattern::CalImageDifferenceMap(IplImage *cent_img, IplImage *neig_img, float *pattern, CvRect *roi)
{
  COpencvDataConversion<uchar, uchar> ODC;

  if ( roi ) {
    cvSetImageROI(cent_img, *roi);
    cvSetImageROI(neig_img, *roi);
  }

  uchar *_centI = ODC.GetImageData(cent_img);
  uchar *_neigI = ODC.GetImageData(neig_img);
  uchar *centI = _centI;
  uchar *neigI = _neigI;

  float *tmp_pattern = pattern;

  int xy;
  int length;

  if ( roi )
    length = roi->height*roi->width;
  else
    length = cent_img->height*cent_img->width;

  for ( xy = 0 ; xy < length ; xy++ ) {
    *tmp_pattern = (float)BINARY_PATTERM_ELEM(*neigI, *centI, m_fRobustWhiteNoise);
    tmp_pattern++;
    centI++;
    neigI++;
  }

  if ( roi ) {
    cvResetImageROI(cent_img);
    cvResetImageROI(neig_img);
  }

  // release memories
  delete [] _centI;
  delete [] _neigI;
}

/*********************************************************************************/
/* AMBP Modeling   */
/*********************************************************************************/

AMBP_Modeling::AMBP_Modeling() {
  m_nMaxAMBPModeNum = MAX_AMBP_MODE_NUM;

  m_fModeUpdatingLearnRate = MODE_UPDATING_LEARN_RATE;
  m_f1_ModeUpdatingLearnRate = 1.0f - m_fModeUpdatingLearnRate;

  m_fWeightUpdatingLearnRate = WEIGHT_UPDATING_LEARN_RATE;
  m_f1_WeightUpdatingLearnRate = 1.0f - m_fWeightUpdatingLearnRate;

  m_fRobustColorOffset = ROBUST_COLOR_OFFSET;

  m_fLowInitialModeWeight = LOW_INITIAL_MODE_WEIGHT;

  m_fPatternColorDistBgThreshold = PATTERN_COLOR_DIST_BACKGROUND_THRESHOLD;
  m_fPatternColorDistBgUpdatedThreshold = PATTERN_COLOR_DIST_BACKGROUND_THRESHOLD;

  m_fBackgroundModelPercent = BACKGROUND_MODEL_PERCENT;

  m_nPatternDistSmoothNeigHalfSize = PATTERN_DIST_SMOOTH_NEIG_HALF_SIZE;
  m_fPatternDistConvGaussianSigma = PATTERN_DIST_CONV_GAUSSIAN_SIGMA;

  m_fRobustShadowRate = ROBUST_SHADOW_RATE;
  m_fRobustHighlightRate = ROBUST_HIGHLIGHT_RATE;

  m_nCurImgFrameIdx = 0;

  m_pBkMaskImg = NULL;

  m_bUsedColorAMBP = false;
  m_bUsedGradImage = false;

  m_fMinAMBPBinaryProb = 0.1f;
  m_f1_MinAMBPBinaryProb = 1.0f - m_fMinAMBPBinaryProb;

  m_pOrgImg = m_pFgImg = m_pBgImg = m_pFgMaskImg = m_pBgDistImg = m_pEdgeImg = NULL;
  m_ppOrgAMBPImgs = NULL;

  m_disableLearning = false;
  m_fSigmaS = 3.0f;
  m_fSigmaR = 0.1f;

  m_fTextureWeight = 0.5f;
  m_fColorWeight = 1.0f - m_fTextureWeight;

  m_fWeightUpdatingConstant = 5.0f;

  m_fReliableBackgroundModeWeight = 0.9f;

  m_fMinBgLayerWeight = 0.0001f;

  m_fMinNoisedAngle = 3.0f / 180.0f * PI;
  m_fMinNoisedAngleSine = sinf(m_fMinNoisedAngle);

  m_fFrameDuration = 1.0f / 25.0f;

  m_fModeUpdatingLearnRatePerSecond = 0.2f;
  m_fWeightUpdatingLearnRatePerSecond = 0.2f;

  m_pROI = NULL;
}

AMBP_Modeling::~AMBP_Modeling() {
  int img_length = m_cvImgSize.height * m_cvImgSize.width;
  PixelAMBPStruct* PAMBP = m_pPixelAMBPs;
  for (int yx = 0; yx < img_length; yx++) {
    delete (*PAMBP).cur_intensity;
    delete (*PAMBP).cur_pattern;
    delete (*PAMBP).ambp_idxes;
    for (int a = 0; a < m_nMaxAMBPModeNum; a++) {
      delete (*PAMBP).AMBPs[a].bg_intensity;
      delete (*PAMBP).AMBPs[a].max_intensity;
      delete (*PAMBP).AMBPs[a].min_intensity;
      delete (*PAMBP).AMBPs[a].bg_pattern;
    }
    delete (*PAMBP).AMBPs;
    PAMBP++;
  }
  delete m_pPixelAMBPs;

  /* release memories */
  if (m_pFgImg != NULL)
    cvReleaseImage(&m_pFgImg);
  if (m_pBgImg != NULL)
    cvReleaseImage(&m_pBgImg);
  if (m_pBgDistImg != NULL)
    cvReleaseImage(&m_pBgDistImg);
  if (m_ppOrgAMBPImgs != NULL) {
    int a;
    for (a = 0; a < m_nAMBPImgNum; a++)
      cvReleaseImage(&m_ppOrgAMBPImgs[a]);
    delete [] m_ppOrgAMBPImgs;
  }
  if (m_pEdgeImg)
    cvReleaseImage(&m_pEdgeImg);
}

void AMBP_Modeling::ResetAllParameters() {
  m_f1_ModeUpdatingLearnRate = 1.0f - m_fModeUpdatingLearnRate;
  m_f1_WeightUpdatingLearnRate = 1.0f - m_fWeightUpdatingLearnRate;
  m_f1_MinAMBPBinaryProb = 1.0f - m_fMinAMBPBinaryProb;

  m_fColorWeight = 1.0f - m_fTextureWeight;

  m_fMinNoisedAngleSine = sinf(m_fMinNoisedAngle);

  m_fMinBgLayerWeight = 0.0001f;

  m_cAMBP.m_fRobustWhiteNoise = m_fRobustColorOffset;
}

void AMBP_Modeling::MergeImages(int num, ...) {
  if (num < 1 || num > 9) {
    exit(0);
  }

  int nCols = 0, nRows = 0;
  switch (num) {
  case 1: nCols = nRows = 1;
    break;
  case 2: nCols = 1;
    nRows = 2;
    break;
  case 3:
  case 4: nCols = 2;
    nRows = 2;
    break;
  case 5:
  case 6: nCols = 3;
    nRows = 2;
    break;
  case 7:
  case 8:
  case 9: nCols = 3;
    nRows = 3;
    break;
  }

  int a, b;

  IplImage** ppIplImg = new IplImage*[num + 1];

  va_list arg_ptr;
  va_start(arg_ptr, num);
  for (a = 0; a < num + 1; a++)
    ppIplImg[a] = va_arg(arg_ptr, IplImage*);
  va_end(arg_ptr);

  CvRect imgROIRect;
  CvSize imgSize = cvGetSize(ppIplImg[0]);
  if (ppIplImg[num] == NULL) {
    ppIplImg[num] = cvCreateImage(cvSize(imgSize.width*nCols, imgSize.height * nRows), IPL_DEPTH_8U, ppIplImg[0]->nChannels);
  }

  int img_idx = 0;
  for (a = 0; a < nRows; a++)
    for (b = 0; b < nCols; b++) {
      if (img_idx >= num)
        break;

      imgROIRect = cvRect(b * imgSize.width, a * imgSize.height, imgSize.width, imgSize.height);

      cvSetImageROI(ppIplImg[num], imgROIRect);
      cvCopyImage(ppIplImg[img_idx++], ppIplImg[num]);
      cvResetImageROI(ppIplImg[num]);
    }

    delete [] ppIplImg;
}

void AMBP_Modeling::Update_MAX_MIN_Intensity(unsigned char *cur_intensity, float *max_intensity, float *min_intensity) {
  int a;
  float curI;
  for (a = 0; a < m_nChannel; a++) {
    curI = (float) cur_intensity[a];

    min_intensity[a] = MIN(curI, min_intensity[a]);
    max_intensity[a] = MAX(curI, max_intensity[a]);
  }
}

void AMBP_Modeling::UpdateBgPixelColor(unsigned char *cur_intensity, float* bg_intensity) {
  int a;
  for (a = 0; a < m_nChannel; a++)
    bg_intensity[a] = m_f1_ModeUpdatingLearnRate * bg_intensity[a] + m_fModeUpdatingLearnRate * (float) cur_intensity[a];
}

void AMBP_Modeling::UpdateBgPixelPattern(float *cur_pattern, AMBPStruct* curAMBP, PixelAMBPStruct *PAMBP, AMBPStruct* AMBPs, unsigned int ambp_num, unsigned short* ambp_idxes) {
    if (m_fTextureWeight > 0)
    {
        for (int a = 0; a < m_nAMBPLength; a++)
        {
          curAMBP->bg_pattern[a] = m_f1_ModeUpdatingLearnRate * curAMBP->bg_pattern[a] + m_fModeUpdatingLearnRate * cur_pattern[a];
        }
    }

    bool removed_modes[10];
    float increasing_weight_factor = m_fWeightUpdatingLearnRate * (1.0f + m_fWeightUpdatingConstant * curAMBP->max_weight);
    curAMBP->weight = (1.0f - increasing_weight_factor) * curAMBP->weight + increasing_weight_factor; //*expf(-best_match_dist/m_fPatternColorDistBgThreshold);

    // update the maximal weight for the best matched mode
    curAMBP->max_weight = MAX(curAMBP->weight, curAMBP->max_weight);

    // calculate the number of background layer
    if (curAMBP->bg_layer_num > 0) {
      bool removed_bg_layers = false;
      if (curAMBP->weight > curAMBP->max_weight * 0.2f) {
        for (int a = 0; a < (int) ambp_num; a++) {
          removed_modes[a] = false;
          if (AMBPs[ambp_idxes[a]].bg_layer_num > curAMBP->bg_layer_num &&
            AMBPs[ambp_idxes[a]].weight < AMBPs[ambp_idxes[a]].max_weight * 0.9f) {
              removed_modes[a] = true;
              removed_bg_layers = true;
          }
        }
      }

      if (removed_bg_layers) {
        RemoveBackgroundLayers(PAMBP, removed_modes);
        ambp_num = (*PAMBP).num;
      }
    } else if (curAMBP->max_weight > m_fReliableBackgroundModeWeight && curAMBP->bg_layer_num == 0) {
      int max_bg_layer_num = AMBPs[ambp_idxes[0]].bg_layer_num;
      for (int a = 1; a < (int) ambp_num; a++)
        max_bg_layer_num = MAX(max_bg_layer_num, AMBPs[ambp_idxes[a]].bg_layer_num);
      curAMBP->bg_layer_num = max_bg_layer_num + 1;
      curAMBP->layer_time = m_nCurImgFrameIdx;
    }
}

void AMBP_Modeling::QuickSort(float *pData, unsigned short *pIdxes, long low, long high, bool bAscent) {
  long i = low;
  long j = high;
  float y = 0;
  int idx = 0;

  float z = pData[(low + high) / 2];

  do {
    if (bAscent) {
      while (pData[i] < z) i++;
      while (pData[j] > z) j--;
    } else {
      while (pData[i] > z) i++;
      while (pData[j] < z) j--;
    }

    if (i <= j) {
      y = pData[i];
      pData[i] = pData[j];
      pData[j] = y;

      idx = pIdxes[i];
      pIdxes[i] = pIdxes[j];
      pIdxes[j] = idx;

      i++;
      j--;
    }
  } while (i <= j);

  if (low < j)
    QuickSort(pData, pIdxes, low, j, bAscent);

  if (i < high)
    QuickSort(pData, pIdxes, i, high, bAscent);
}

float AMBP_Modeling::DistAMBP(AMBPStruct *AMBP1, AMBPStruct *AMBP2) {
  int a;

  float pattern_dist = 0;
  for (a = 0; a < m_nAMBPLength; a++) {
    pattern_dist = fabsf(AMBP1->bg_pattern[a] - AMBP1->bg_pattern[a]);
  }
  pattern_dist /= (float) m_nAMBPLength;

  float color_dist = 0;
  for (a = 0; a < m_nChannel; a++) {
    color_dist += fabsf((float) AMBP1->bg_intensity[a]-(float) AMBP2->bg_intensity[a]);
  }
  color_dist /= 3.0f * 125.0f;

  return color_dist;
}

void AMBP_Modeling::SetNewImage(IplImage *new_img, CvRect *roi) {
  m_pOrgImg = new_img;
  m_pROI = roi;
  if (roi && (roi->width <= 0 || roi->height <= 0))
    return;

  if (roi) {
    cvSetImageROI(m_pOrgImg, *roi);
    for (int a = 0; a < m_nAMBPImgNum; a++)
      cvSetImageROI(m_ppOrgAMBPImgs[a], *roi);
  }

  switch (m_nAMBPImgNum) {
  case 1:
    cvCvtColor(m_pOrgImg, m_ppOrgAMBPImgs[0], CV_BGR2GRAY);
    break;
  case 2:
    cvCvtColor(m_pOrgImg, m_ppOrgAMBPImgs[0], CV_BGR2GRAY);
    ComputeGradientImage(m_ppOrgAMBPImgs[0], m_ppOrgAMBPImgs[1], false);
    break;
  case 3:
    cvSplit(m_pOrgImg, m_ppOrgAMBPImgs[0], m_ppOrgAMBPImgs[1], m_ppOrgAMBPImgs[2], NULL);
    break;
  case 4:
    cvSplit(m_pOrgImg, m_ppOrgAMBPImgs[0], m_ppOrgAMBPImgs[1], m_ppOrgAMBPImgs[2], NULL);
    ComputeGradientImage(m_ppOrgAMBPImgs[0], m_ppOrgAMBPImgs[3], false);
    break;
  }

  if (roi) {
    cvResetImageROI(m_pOrgImg);
    for (int a = 0; a < m_nAMBPImgNum; a++)
      cvResetImageROI(m_ppOrgAMBPImgs[a]);
  }
  m_cAMBP.SetNewImages(m_ppOrgAMBPImgs);

  m_nCurImgFrameIdx++;
}

void AMBP_Modeling::SetBkMaskImage(IplImage *mask_img) {
  if (m_pBkMaskImg == NULL) {
    m_pBkMaskImg = cvCreateImage(cvGetSize(mask_img), mask_img->depth, mask_img->nChannels);
  }
  cvCopyImage(mask_img, m_pBkMaskImg);
}

Mat AMBP_Modeling::BackgroundSubtractionProcess() {
  CvRect *roi = m_pROI;

  Mat ambp_frame;
  if (roi && (roi->width <= 0 || roi->height <= 0))
    return ambp_frame;

  AMBPStruct* AMBPs;
  unsigned int bg_num;
  float* cur_pattern;
  unsigned char* cur_intensity;
  int a, b;
  unsigned int ambp_num;
  unsigned short* ambp_idxes;
  unsigned short cur_ambp_idx;
  bool bBackgroundUpdating;

  Mat procImg, mbp_params;
  resize(Mat(m_pOrgImg), procImg, Size(320, 240));
  mbp_params = procImg.clone();

  PixelAMBPStruct *PAMBP = m_pPixelAMBPs;

  bool bFirstFrame = (PAMBP[0].num == 0);
  float best_match_bg_dist, bg_pattern_dist, bg_color_dist, bg_pattern_color_dist;

  if (m_fTextureWeight > 0)
    m_cAMBP.ComputeAMBP(procImg, PAMBP, mbp_params, roi);

  AMBPStruct* curAMBP;

  int data_length;

  if (roi)
    data_length = roi->width * roi->height;
  else
    data_length = m_cvImgSize.width * m_cvImgSize.height;

  int best_match_idx;

  COpencvDataConversion<uchar, uchar> ODC1;
  if (roi) {
    cvSetImageROI(m_pBkMaskImg, *roi);
    cvSetImageROI(m_pOrgImg, *roi);
  }
  uchar *_mask = ODC1.GetImageData(m_pBkMaskImg);
  uchar *_org_intensity = ODC1.GetImageData(m_pOrgImg);

  if (roi) {
    cvResetImageROI(m_pBkMaskImg);
    cvResetImageROI(m_pOrgImg);
  }

  COpencvDataConversion<float, float> ODC2;
  float *_bg_dist = new float[data_length];

  uchar *mask = _mask;
  uchar *org_intensity = _org_intensity;
  float *bg_dist = _bg_dist;

  int x, y;

  for (y = 0; y < (roi ? roi->height : m_cvImgSize.height); y++) {
    if (roi)
      PAMBP = m_pPixelAMBPs + (roi->y + y) * m_cvImgSize.width + roi->x;
    else
      PAMBP = m_pPixelAMBPs + y * m_cvImgSize.width;

    for (x = 0; x < (roi ? roi->width : m_cvImgSize.width); x++) {
      if (*mask++ == 0) {
        PAMBP++;
        *bg_dist++ = 0.0f;
        org_intensity += m_nChannel;
        continue;
      }

      // removing the background layers
      if (!m_disableLearning) {
        RemoveBackgroundLayers(PAMBP);
      }

      // check whether the current image is the first image
      bFirstFrame = ((*PAMBP).num == 0);

      // get ambp information
      ambp_num = (*PAMBP).num;
      AMBPs = (*PAMBP).AMBPs;
      ambp_idxes = (*PAMBP).ambp_idxes;

      (*PAMBP).cur_bg_layer_no = 0;

      // set the current pixel's intensity
      cur_intensity = (*PAMBP).cur_intensity;
      for (a = 0; a < m_nChannel; a++)
        cur_intensity[a] = *org_intensity++;

      // get the current ambp pattern
      cur_pattern = (*PAMBP).cur_pattern;

      // first check whether the pixel is background or foreground and then update the background pattern model
      if (ambp_num == 0) {
        curAMBP = (&(AMBPs[0]));
        for (a = 0; a < m_nAMBPLength; a++) {
          curAMBP->bg_pattern[a] = (float) cur_pattern[a];
        }

        curAMBP->bg_layer_num = 0;
        curAMBP->weight = m_fLowInitialModeWeight;
        curAMBP->max_weight = m_fLowInitialModeWeight;

        curAMBP->first_time = m_nCurImgFrameIdx;
        curAMBP->last_time = m_nCurImgFrameIdx;
        curAMBP->freq = 1;

        (*PAMBP).matched_mode_first_time = (float) m_nCurImgFrameIdx;

        for (a = 0; a < m_nChannel; a++) {
          curAMBP->bg_intensity[a] = (float) cur_intensity[a];
          curAMBP->min_intensity[a] = (float) cur_intensity[a];
          curAMBP->max_intensity[a] = (float) cur_intensity[a];
        }

        ambp_idxes[0] = 0;

        ambp_num++;
        (*PAMBP).num = 1;
        (*PAMBP).bg_num = 1;

        PAMBP++;
        *bg_dist++ = 0.0f;
        continue;
      } else {
        best_match_idx = -1;
        best_match_bg_dist = 999.0f;

        // find the best match
        for (a = 0; a < (int) ambp_num; a++) {
          // get the current index for ambp pattern
          cur_ambp_idx = ambp_idxes[a];

          // get the current AMBP pointer
          curAMBP = &(AMBPs[cur_ambp_idx]);

          // compute the background probability based on ambp pattern
          bg_pattern_dist = 0.0f;
          if (m_fTextureWeight > 0)
            bg_pattern_dist = CalPatternBgDist(cur_pattern, curAMBP->bg_pattern);

          // compute the color invariant probability based on RGB color
          bg_color_dist = 0.0f;
          if (m_fColorWeight > 0)
            bg_color_dist = CalColorBgDist(cur_intensity, curAMBP->bg_intensity, curAMBP->max_intensity, curAMBP->min_intensity);

          bg_pattern_color_dist = m_fColorWeight * bg_color_dist + m_fTextureWeight*bg_pattern_dist;

          if (bg_pattern_color_dist < best_match_bg_dist) {
            best_match_bg_dist = bg_pattern_color_dist;
            best_match_idx = a;
          }
        }

        bg_num = (*PAMBP).bg_num;

        bBackgroundUpdating = ((best_match_bg_dist < m_fPatternColorDistBgUpdatedThreshold));

        // reset the weight of the mode
        if (best_match_idx >= (int) bg_num && AMBPs[ambp_idxes[best_match_idx]].max_weight < m_fReliableBackgroundModeWeight) // found not in the background models
          best_match_bg_dist = MAX(best_match_bg_dist, m_fPatternColorDistBgThreshold * 2.5f);

        *bg_dist = best_match_bg_dist;
      }
      if (m_disableLearning) {
        // no creation or update when learning is disabled
      } else if (!bBackgroundUpdating) { // no match

        for (a = 0; a < (int) ambp_num; a++) { // decrease the weights
          curAMBP = &(AMBPs[ambp_idxes[a]]);
          curAMBP->weight *= (1.0f - m_fWeightUpdatingLearnRate / (1.0f + m_fWeightUpdatingConstant * curAMBP->max_weight));
        }

        if ((int) ambp_num < m_nMaxAMBPModeNum) { // add a new pattern
          // find the pattern index for addition
          int add_ambp_idx = 0;
          bool bFound;
          for (a = 0; a < m_nMaxAMBPModeNum; a++) {
            bFound = true;
            for (b = 0; b < (int) ambp_num; b++)
              bFound &= (a != ambp_idxes[b]);
            if (bFound) {
              add_ambp_idx = a;
              break;
            }
          }
          curAMBP = &(AMBPs[add_ambp_idx]);

          curAMBP->first_time = m_nCurImgFrameIdx;
          curAMBP->last_time = m_nCurImgFrameIdx;
          curAMBP->freq = 1;
          curAMBP->layer_time = -1;

          (*PAMBP).matched_mode_first_time = (float) m_nCurImgFrameIdx;

          for (a = 0; a < m_nAMBPLength; a++) {
            curAMBP->bg_pattern[a] = (float) cur_pattern[a];
          }

          curAMBP->bg_layer_num = 0;
          curAMBP->weight = m_fLowInitialModeWeight;
          curAMBP->max_weight = m_fLowInitialModeWeight;

          for (a = 0; a < m_nChannel; a++) {
            curAMBP->bg_intensity[a] = (float) cur_intensity[a];
            curAMBP->min_intensity[a] = (float) cur_intensity[a];
            curAMBP->max_intensity[a] = (float) cur_intensity[a];
          }

          ambp_idxes[ambp_num] = add_ambp_idx;

          ambp_num++;
          (*PAMBP).num = ambp_num;
        } else {
          int rep_pattern_idx = ambp_idxes[m_nMaxAMBPModeNum - 1];

          curAMBP = &(AMBPs[rep_pattern_idx]);

          curAMBP->first_time = m_nCurImgFrameIdx;
          curAMBP->last_time = m_nCurImgFrameIdx;
          curAMBP->freq = 1;
          curAMBP->layer_time = -1;

          (*PAMBP).matched_mode_first_time = (float) m_nCurImgFrameIdx;

          for (a = 0; a < m_nAMBPLength; a++) {
            curAMBP->bg_pattern[a] = (float) cur_pattern[a];
          }

          curAMBP->bg_layer_num = 0;
          curAMBP->weight = m_fLowInitialModeWeight;
          curAMBP->max_weight = m_fLowInitialModeWeight;

          for (a = 0; a < m_nChannel; a++) {
            curAMBP->bg_intensity[a] = (float) cur_intensity[a];
            curAMBP->min_intensity[a] = (float) cur_intensity[a];
            curAMBP->max_intensity[a] = (float) cur_intensity[a];
          }
        }
      } else {
        // updating the background pattern model
        cur_ambp_idx = ambp_idxes[best_match_idx];
        curAMBP = &(AMBPs[cur_ambp_idx]);

        curAMBP->first_time = MAX(MIN(curAMBP->first_time, m_nCurImgFrameIdx), 0);
        (*PAMBP).matched_mode_first_time = curAMBP->first_time;

        curAMBP->last_time = m_nCurImgFrameIdx;
        curAMBP->freq++;

        if (m_fColorWeight > 0) {
          // update the color information
          UpdateBgPixelColor(cur_intensity, curAMBP->bg_intensity);
          // update the MAX and MIN color intensity
          Update_MAX_MIN_Intensity(cur_intensity, curAMBP->max_intensity, curAMBP->min_intensity);
        }

        // update the texture information
        UpdateBgPixelPattern(cur_pattern, curAMBP, PAMBP, AMBPs, ambp_num, ambp_idxes);
        (*PAMBP).cur_bg_layer_no = curAMBP->bg_layer_num;

        // decrease the weights of non-best matched modes
        for (a = 0; a < (int) ambp_num; a++) {
          if (a != best_match_idx) {
            curAMBP = &(AMBPs[ambp_idxes[a]]);
            curAMBP->weight *= (1.0f - m_fWeightUpdatingLearnRate / (1.0f + m_fWeightUpdatingConstant * curAMBP->max_weight));
          }
        }
      }

      // sort the list of modes based on the weights of modes
      if ((int) ambp_num > 1 && !m_disableLearning) {
        float weights[100], tot_weights = 0;
        for (a = 0; a < (int) ambp_num; a++) {
          weights[a] = AMBPs[ambp_idxes[a]].weight;
          tot_weights += weights[a];
        }

        // sort weights in the descent order
        QuickSort(weights, ambp_idxes, 0, (int) ambp_num - 1, false);

        // calculate the first potential background modes number, bg_num
        float threshold_weight = m_fBackgroundModelPercent*tot_weights;
        tot_weights = 0;
        for (a = 0; a < (int) ambp_num; a++) {
          tot_weights += AMBPs[ambp_idxes[a]].weight;
          if (tot_weights > threshold_weight) {
            bg_num = a + 1;
            break;
          }
        }
        (*PAMBP).bg_num = bg_num;
      }

      PAMBP++;
      bg_dist++;
    }
  }

  if (bFirstFrame) { // check whether it is the first frame for background modeling
    if (m_pFgMaskImg)
      cvSetZero(m_pFgMaskImg);
    cvSetZero(m_pBgDistImg);
  } else {
    // set the image data
    if (roi) {
      cvSetZero(m_pBgDistImg);
      cvSetImageROI(m_pBgDistImg, *roi);
    }
    ODC2.SetImageData(m_pBgDistImg, _bg_dist);

    // do gaussian smooth
    if (m_nPatternDistSmoothNeigHalfSize >= 0)
      cvSmooth(m_pBgDistImg, m_pBgDistImg, CV_GAUSSIAN, (2 * m_nPatternDistSmoothNeigHalfSize + 1), (2 * m_nPatternDistSmoothNeigHalfSize + 1), m_fPatternDistConvGaussianSigma);

    if (roi)
      cvResetImageROI(m_pBgDistImg);

    // get the foreground mask by thresholding
    if (m_pFgMaskImg)
      cvThreshold(m_pBgDistImg, m_pFgMaskImg, m_fPatternColorDistBgThreshold, 255, CV_THRESH_BINARY);

    // get the foreground probability image (uchar)
    if (m_pFgProbImg)
      GetForegroundProbabilityImage(m_pFgProbImg);
  }

  // release memories
  delete [] _mask;
  delete [] _bg_dist;
  delete [] _org_intensity;

  return mbp_params;
}

void AMBP_Modeling::GetBackgroundImage(IplImage *bk_img) {
  IplImage *bg_img = m_pBgImg;
  uchar *c1;
  float *c2;
  int bg_img_idx;
  int channel;
  int img_length = m_cvImgSize.height * m_cvImgSize.width;
  int yx;

  COpencvDataConversion<uchar, uchar> ODC;
  uchar *org_data = ODC.GetImageData(bg_img);
  c1 = org_data;

  PixelAMBPStruct* PAMBP = m_pPixelAMBPs;

  for (yx = 0; yx < img_length; yx++) {
    // the newest background image
    bg_img_idx = (*PAMBP).ambp_idxes[0];
    if ((*PAMBP).num == 0) {
      for (channel = 0; channel < m_nChannel; channel++)
        *c1++ = 0;
    } else {
      c2 = (*PAMBP).AMBPs[bg_img_idx].bg_intensity;
      for (channel = 0; channel < m_nChannel; channel++)
        *c1++ = cvRound(*c2++);
    }
    PAMBP++;
  }

  ODC.SetImageData(bg_img, org_data);
  delete [] org_data;

  cvCopyImage(m_pBgImg, bk_img);
}

void AMBP_Modeling::GetForegroundImage(IplImage *fg_img, CvScalar bg_color) {
  if (m_pROI && (m_pROI->width <= 0 || m_pROI->height <= 0))
    return;
  IplImage* org_img;
  IplImage* fg_mask_img;

  org_img = m_pOrgImg;
  fg_mask_img = m_pFgMaskImg;

  cvSet(fg_img, bg_color);
  if (m_pROI) {
    cvSetImageROI(org_img, *m_pROI);
    cvSetImageROI(fg_img, *m_pROI);
    cvSetImageROI(fg_mask_img, *m_pROI);
    cvCopy(org_img, fg_img, fg_mask_img);
    cvResetImageROI(org_img);
    cvResetImageROI(fg_img);
    cvResetImageROI(fg_mask_img);
  } else
    cvCopy(org_img, fg_img, fg_mask_img);
}

void AMBP_Modeling::GetForegroundMaskImage(IplImage *fg_mask_img) {
  if (m_pROI && (m_pROI->width <= 0 || m_pROI->height <= 0))
    return;

  if (m_pROI) {
    cvSetImageROI(m_pFgMaskImg, *m_pROI);
    cvSetImageROI(fg_mask_img, *m_pROI);
    cvThreshold(m_pFgMaskImg, fg_mask_img, 0, 255, CV_THRESH_BINARY);
    cvResetImageROI(m_pFgMaskImg);
    cvResetImageROI(fg_mask_img);
  } else
    cvThreshold(m_pFgMaskImg, fg_mask_img, 0, 255, CV_THRESH_BINARY);
}

void AMBP_Modeling::GetForegroundMaskMap(CvMat *fg_mask_mat) {
  COpencvDataConversion<uchar, uchar> ODC;
  ODC.ConvertData(m_pFgMaskImg, fg_mask_mat);
}

void AMBP_Modeling::GetCurrentBackgroundDistMap(CvMat *bk_dist_map) {
  cvCopy(m_pBgDistImg, bk_dist_map);
}

void AMBP_Modeling::Initialization(IplImage *first_img, int ambp_level_num, float *radiuses, int *neig_pt_nums) {
  int a;

  m_nAMBPLength = 0;
  m_nAMBPLevelNum = ambp_level_num;
  for (a = 0; a < ambp_level_num; a++) {
    m_nAMBPLength += neig_pt_nums[a];
    m_pAMBPRadiuses[a] = radiuses[a];
    m_pAMBPMeigPointNums[a] = neig_pt_nums[a];
  }

  m_pFgImg = NULL;
  m_pFgMaskImg = NULL;
  m_pBgDistImg = NULL;
  m_pOrgImg = NULL;
  m_pBgImg = NULL;
  m_ppOrgAMBPImgs = NULL;
  m_pFgProbImg = NULL;

  m_cvImgSize = cvGetSize(first_img);

  m_nChannel = first_img->nChannels;

  m_pOrgImg = first_img;

  if (m_bUsedColorAMBP && m_bUsedGradImage)
    m_nAMBPImgNum = 4;
  else if (m_bUsedColorAMBP && !m_bUsedGradImage)
    m_nAMBPImgNum = 3;
  else if (!m_bUsedColorAMBP && m_bUsedGradImage)
    m_nAMBPImgNum = 2;
  else
    m_nAMBPImgNum = 1;

  m_nAMBPLength *= m_nAMBPImgNum;

  m_ppOrgAMBPImgs = new IplImage*[m_nAMBPImgNum];
  for (a = 0; a < m_nAMBPImgNum; a++)
    m_ppOrgAMBPImgs[a] = cvCreateImage(m_cvImgSize, IPL_DEPTH_8U, 1);

  m_pBgImg = cvCreateImage(m_cvImgSize, IPL_DEPTH_8U, m_nChannel);
  m_pFgImg = cvCreateImage(m_cvImgSize, IPL_DEPTH_8U, m_nChannel);
  m_pEdgeImg = cvCreateImage(m_cvImgSize, IPL_DEPTH_32F, 1);
  m_pBgDistImg = cvCreateImage(m_cvImgSize, IPL_DEPTH_32F, 1);

  ResetAllParameters();

  int img_length = m_cvImgSize.height * m_cvImgSize.width;
  m_pPixelAMBPs = new PixelAMBPStruct[img_length];
  PixelAMBPStruct* PAMBP = m_pPixelAMBPs;
  int yx;
  for (yx = 0; yx < img_length; yx++) {
    (*PAMBP).cur_intensity = new unsigned char[m_nChannel];
    (*PAMBP).cur_pattern = new float[m_nAMBPLength];
    (*PAMBP).AMBPs = new AMBPStruct[m_nMaxAMBPModeNum];
    (*PAMBP).ambp_idxes = new unsigned short[m_nMaxAMBPModeNum];
    (*PAMBP).ambp_idxes[0] = 0;
    (*PAMBP).num = 0;
    (*PAMBP).cur_bg_layer_no = 0;
    (*PAMBP).matched_mode_first_time = 0;
    for (a = 0; a < m_nMaxAMBPModeNum; a++) {
      (*PAMBP).AMBPs[a].bg_intensity = new float[m_nChannel];
      (*PAMBP).AMBPs[a].max_intensity = new float[m_nChannel];
      (*PAMBP).AMBPs[a].min_intensity = new float[m_nChannel];
      (*PAMBP).AMBPs[a].bg_pattern = new float[m_nAMBPLength];
      (*PAMBP).AMBPs[a].first_time = -1;
      (*PAMBP).AMBPs[a].last_time = -1;
      (*PAMBP).AMBPs[a].freq = -1;
      (*PAMBP).AMBPs[a].layer_time = -1;
    }
    PAMBP++;
  }

  m_pBkMaskImg = cvCreateImage(m_cvImgSize, IPL_DEPTH_8U, 1);
  cvSet(m_pBkMaskImg, cvScalar(1));

  m_cAMBP.Initialization(m_ppOrgAMBPImgs, m_nAMBPImgNum, ambp_level_num, radiuses, neig_pt_nums, m_fRobustColorOffset);

#ifdef LINUX_BILATERAL_FILTER
  if (m_fSigmaS > 0 && m_fSigmaR > 0)
    m_cCrossBF.Initialization(m_pBgDistImg, m_pBgDistImg, m_fSigmaS, m_fSigmaR);
#endif
}

float AMBP_Modeling::CalPatternBgDist(float *cur_pattern, float *bg_pattern) {
  float bg_hamming_dist = 0;
  int a;
  for (a = 0; a < m_nAMBPLength; a++)
    bg_hamming_dist += fabsf(cur_pattern[a] - bg_pattern[a]) > m_f1_MinAMBPBinaryProb;

  bg_hamming_dist /= (float) m_nAMBPLength;

  return bg_hamming_dist;
}

float AMBP_Modeling::CalColorBgDist(uchar *cur_intensity, float *bg_intensity, float *max_intensity, float *min_intensity) {
  float noised_angle, range_dist, bg_color_dist;

  range_dist = CalColorRangeDist(cur_intensity, bg_intensity, max_intensity, min_intensity, m_fRobustShadowRate, m_fRobustHighlightRate);

  if (range_dist == 1.0f)
    bg_color_dist = range_dist;
  else {
    noised_angle = CalVectorsNoisedAngle(bg_intensity, cur_intensity, MAX(m_fRobustColorOffset, 5.0f), m_nChannel);
    bg_color_dist = (1.0f - expf(-100.0f * noised_angle * noised_angle));
  }

  return bg_color_dist;
}

void AMBP_Modeling::ComputeGradientImage(IplImage *src, IplImage *dst, bool bIsFloat) {
  if (src->nChannels != 1 || dst->nChannels != 1) {
    exit(1);
  }

  int a;

  IplImage* _dX = cvCreateImage(cvGetSize(dst), IPL_DEPTH_16S, 1);
  IplImage* _dY = cvCreateImage(cvGetSize(dst), IPL_DEPTH_16S, 1);

  int aperture_size = 3;

  cvSobel(src, _dX, 1, 0, aperture_size);
  cvSobel(src, _dY, 0, 1, aperture_size);

  COpencvDataConversion<short, short> ODC1;
  COpencvDataConversion<uchar, uchar> ODC2;
  COpencvDataConversion<float, float> ODC3;

  short* dX_data = ODC1.GetImageData(_dX);
  short* dY_data = ODC1.GetImageData(_dY);

  uchar* dst_u_data = NULL;
  float* dst_f_data = NULL;

  if (bIsFloat)
    dst_f_data = ODC3.GetImageData(dst);
  else
    dst_u_data = ODC2.GetImageData(dst);

  short* dX = dX_data;
  short* dY = dY_data;
  uchar *uSrc = dst_u_data;
  float *fSrc = dst_f_data;

  int length;
  if (src->roi)
    length = dst->width * dst->height;
  else
    length = dst->roi->width * dst->roi->height;

  if (bIsFloat) {
    for (a = 0; a < length; a++) {
      *fSrc = cvSqrt((float) ((*dX)*(*dX)+(*dY)*(*dY)) / (32.0f * 255.0f));
      fSrc++;
      dX++;
      dY++;
    }
    ODC3.SetImageData(dst, dst_f_data);
    delete [] dst_f_data;
  } else {
    for (a = 0; a < length; a++) {
      *uSrc = cvRound(cvSqrt((float) ((*dX)*(*dX)+(*dY)*(*dY)) / 32.0f));
      uSrc++;
      dX++;
      dY++;
    }
    ODC2.SetImageData(dst, dst_u_data);
    delete [] dst_u_data;
  }

  delete [] dX_data;
  delete [] dY_data;

  cvReleaseImage(&_dX);
  cvReleaseImage(&_dY);
}

float AMBP_Modeling::CalVectorsNoisedAngle(float *bg_color, unsigned char *noised_color, float offset, int length) {
  float org_angle = CalVectorsAngle(bg_color, noised_color, length);
  float norm_color = 0, elem, noised_angle;
  int a;
  for (a = 0; a < length; a++) {
    elem = bg_color[a];
    norm_color += elem*elem;
  }
  norm_color = sqrtf(norm_color);
  if (norm_color == 0)
    noised_angle = PI;
  else {
    float sin_angle = offset / norm_color;
    if (sin_angle < m_fMinNoisedAngleSine)
      noised_angle = m_fMinNoisedAngle;
    else
      noised_angle = (sin_angle >= 1 ? PI : sin_angle);
  }

  float angle = org_angle - noised_angle;
  if (angle < 0)
    angle = 0;
  return angle;
}

float AMBP_Modeling::CalVectorsAngle(float *c1, unsigned char *c2, int length) {
  float angle;
  float dot2, norm1, norm2, elem1, elem2;

  dot2 = norm1 = norm2 = 0;

  int a;
  for (a = 0; a < length; a++) {
    elem1 = (float) (c1[a]);
    elem2 = (float) (c2[a]);
    dot2 += elem1*elem2;
    norm1 += elem1*elem1;
    norm2 += elem2*elem2;
  }

  angle = (norm1 * norm2 == 0 ? 0 : sqrtf(std::max(1.0f - dot2 * dot2 / (norm1 * norm2), 0.f)));

  return angle;
}

float AMBP_Modeling::CalColorRangeDist(unsigned char *cur_intensity, float *bg_intensity, float *max_intensity, float *min_intensity, float shadow_rate, float highlight_rate) {
  float dist = 0.0f, minI, maxI, bgI, curI;
  int channel;

  for (channel = 0; channel < m_nChannel; channel++) {
    bgI = bg_intensity[channel];

    minI = MIN(min_intensity[channel], bgI * shadow_rate - 5.0f);
    maxI = MAX(max_intensity[channel], bgI * highlight_rate + 5.0f);
    curI = (float) (cur_intensity[channel]);

    if (curI > maxI || curI < minI) {
      dist = 1.0f;
      break;
    }
  }

  return dist;
}

void AMBP_Modeling::GetLayeredBackgroundImage(int layered_no, IplImage *layered_bg_img, CvScalar empty_color) {
  PixelAMBPStruct *PAMBP = m_pPixelAMBPs;
  AMBPStruct* AMBPs;
  unsigned short* ambp_idxes;

  int a, b, c;
  int img_length = m_pOrgImg->width * m_pOrgImg->height;

  cvSet(layered_bg_img, empty_color);

  COpencvDataConversion<uchar, uchar> ODC;

  uchar *bg_img_data = ODC.GetImageData(layered_bg_img);
  uchar *_bg_img_data = bg_img_data;
  float *cur_bg_intensity;
  int ambp_num;

  for (a = 0; a < img_length; a++) {
    // get ambp information
    AMBPs = (*PAMBP).AMBPs;
    ambp_idxes = (*PAMBP).ambp_idxes;
    ambp_num = (int) ((*PAMBP).num);
    bool found = false;
    for (b = 0; b < ambp_num; b++) {
      if (AMBPs[ambp_idxes[b]].bg_layer_num == layered_no) {
        cur_bg_intensity = AMBPs[ambp_idxes[b]].bg_intensity;
        for (c = 0; c < m_pOrgImg->nChannels; c++)
          *_bg_img_data++ = (uchar) * cur_bg_intensity++;
        found = true;
        break;
      }
    }
    if (!found)
      _bg_img_data += m_pOrgImg->nChannels;

    PAMBP++;
  }

  ODC.SetImageData(layered_bg_img, bg_img_data);

  delete [] bg_img_data;
}

void AMBP_Modeling::GetBgLayerNoImage(IplImage *bg_layer_no_img, CvScalar *layer_colors, int layer_num) {
  if (layer_num != 0 && layer_num != m_nMaxAMBPModeNum) {
    exit(1);
  }

  CvScalar *bg_layer_colors;
  int bg_layer_color_num = layer_num;
  if (bg_layer_color_num == 0)
    bg_layer_color_num = m_nMaxAMBPModeNum;
  bg_layer_colors = new CvScalar[bg_layer_color_num];
  if (layer_colors) {
    for (int l = 0; l < layer_num; l++)
      bg_layer_colors[l] = layer_colors[l];
  } else {
    int rgb[3];
    rgb[0] = rgb[1] = rgb[2] = 0;
    int rgb_idx = 0;
    for (int l = 0; l < bg_layer_color_num; l++) {
      bg_layer_colors[l] = CV_RGB(rgb[0], rgb[1], rgb[2]);
      rgb[rgb_idx] += 200;
      rgb[rgb_idx] %= 255;
      rgb_idx++;
      rgb_idx %= 3;
    }
  }

  int img_length = m_pOrgImg->width * m_pOrgImg->height;
  uchar *bg_layer_data = new uchar[img_length * bg_layer_no_img->nChannels];
  uchar *_bg_layer_data = bg_layer_data;

  PixelAMBPStruct *PAMBP = m_pPixelAMBPs;
  unsigned int cur_bg_layer_no;

  for (int a = 0; a < img_length; a++) {
    cur_bg_layer_no = (*PAMBP).cur_bg_layer_no;
    for (int b = 0; b < bg_layer_no_img->nChannels; b++) {
      *_bg_layer_data++ = (uchar) (bg_layer_colors[cur_bg_layer_no].val[b]);
    }
    PAMBP++;
  }

  COpencvDataConversion<uchar, uchar> ODC;
  ODC.SetImageData(bg_layer_no_img, bg_layer_data);

  delete [] bg_layer_data;
  delete [] bg_layer_colors;
}

void AMBP_Modeling::GetCurrentLayeredBackgroundImage(int layered_no, IplImage *layered_bg_img, IplImage *layered_fg_img, CvScalar layered_bg_bk_color, CvScalar layered_fg_color,
  int smooth_win, float smooth_sigma, float below_layer_noise, float above_layer_noise, int min_blob_size) {
    PixelAMBPStruct *PAMBP = m_pPixelAMBPs;
    AMBPStruct* AMBPs;
    unsigned short* ambp_idxes;

    int a;
    int img_length = m_pOrgImg->width * m_pOrgImg->height;

    float *bg_layer_mask = new float[img_length];
    float *_bg_layer_mask = bg_layer_mask;

    for (a = 0; a < img_length; a++) {
      // get ambp information
      AMBPs = (*PAMBP).AMBPs;
      ambp_idxes = (*PAMBP).ambp_idxes;
      *_bg_layer_mask++ = (float) (*PAMBP).cur_bg_layer_no;
      PAMBP++;
    }

    COpencvDataConversion<float, float> ODC;
    IplImage* bg_layer_float_mask_img = cvCreateImage(cvGetSize(m_pOrgImg), IPL_DEPTH_32F, 1);
    IplImage* bg_layer_low_mask_img = cvCreateImage(cvGetSize(m_pOrgImg), IPL_DEPTH_8U, 1);
    IplImage* bg_layer_high_mask_img = cvCreateImage(cvGetSize(m_pOrgImg), IPL_DEPTH_8U, 1);
    IplImage* bg_layer_mask_img = cvCreateImage(cvGetSize(m_pOrgImg), IPL_DEPTH_8U, 1);

    ODC.SetImageData(bg_layer_float_mask_img, bg_layer_mask);
    cvSmooth(bg_layer_float_mask_img, bg_layer_float_mask_img, CV_GAUSSIAN, smooth_win, smooth_win, smooth_sigma);

    cvThreshold(bg_layer_float_mask_img, bg_layer_low_mask_img, (float) layered_no - below_layer_noise, 1, CV_THRESH_BINARY);
    cvThreshold(bg_layer_float_mask_img, bg_layer_high_mask_img, (float) layered_no + above_layer_noise, 1, CV_THRESH_BINARY_INV);
    cvAnd(bg_layer_low_mask_img, bg_layer_high_mask_img, bg_layer_mask_img);

    cvDilate(bg_layer_mask_img, bg_layer_mask_img, 0, 2);
    cvErode(bg_layer_mask_img, bg_layer_mask_img, 0, 2);

    // Extract the blobs using a threshold of 100 in the image
    CBlobResult blobs = CBlobResult(bg_layer_mask_img, NULL, 0, true);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, min_blob_size);

    CBlob filtered_blob;
    cvSetZero(bg_layer_mask_img);
    for (a = 0; a < blobs.GetNumBlobs(); a++) {
      filtered_blob = blobs.GetBlob(a);
      filtered_blob.FillBlob(bg_layer_mask_img, cvScalar(1));
    }
    blobs.GetNthBlob(CBlobGetArea(), 0, filtered_blob);
    filtered_blob.FillBlob(bg_layer_mask_img, cvScalar(0));


    cvSet(layered_bg_img, layered_bg_bk_color);
    cvCopy(m_pBgImg, layered_bg_img, bg_layer_mask_img);

    if (layered_fg_img) {
      cvCopy(m_pOrgImg, layered_fg_img);
      cvSet(layered_fg_img, layered_fg_color, bg_layer_mask_img);
    }

    cvReleaseImage(&bg_layer_float_mask_img);
    cvReleaseImage(&bg_layer_low_mask_img);
    cvReleaseImage(&bg_layer_high_mask_img);
    cvReleaseImage(&bg_layer_mask_img);
    delete [] bg_layer_mask;
}

void AMBP_Modeling::GetColoredBgMultiLayeredImage(IplImage *bg_multi_layer_img, CvScalar *layer_colors) {
  cvCopyImage(m_pOrgImg, bg_multi_layer_img);

  COpencvDataConversion<uchar, uchar> ODC;

  uchar *bg_ml_imgD = ODC.GetImageData(bg_multi_layer_img);
  uchar *fg_maskD = ODC.GetImageData(m_pFgMaskImg);

  uchar *_bg_ml_imgD = bg_ml_imgD;
  uchar *_fg_maskD = fg_maskD;

  PixelAMBPStruct *PAMBP = m_pPixelAMBPs;
  AMBPStruct* AMBPs;
  unsigned short* ambp_idxes;
  unsigned int ambp_num;
  int bg_layer_num;

  int a, c;
  int img_length = m_pOrgImg->width * m_pOrgImg->height;
  int channels = m_pOrgImg->nChannels;
  bool bLayeredBg;

  for (a = 0; a < img_length; a++) {
    // get ambp information
    ambp_num = (*PAMBP).num;
    AMBPs = (*PAMBP).AMBPs;
    ambp_idxes = (*PAMBP).ambp_idxes;
    bLayeredBg = false;

    if ((*_fg_maskD == 0)) {
      bg_layer_num = AMBPs[ambp_idxes[0]].bg_layer_num;
      int first_layer_idx = 0;
      for (c = 0; c < (int) ambp_num; c++) {
        if (AMBPs[ambp_idxes[c]].bg_layer_num == 1) {
          first_layer_idx = c;
          break;
        }
      }
      if (bg_layer_num > 1 && DistAMBP(&(AMBPs[ambp_idxes[0]]), &(AMBPs[first_layer_idx])) > 0.1f) {
        for (c = 0; c < channels; c++)
          *_bg_ml_imgD++ = (uchar) (layer_colors[bg_layer_num].val[c]);
        bLayeredBg = true;
      }

      if (!bLayeredBg)
        _bg_ml_imgD += channels;
    } else {
      _bg_ml_imgD += channels;
    }

    PAMBP++;
    _fg_maskD++;
  }

  ODC.SetImageData(bg_multi_layer_img, bg_ml_imgD);

  delete [] fg_maskD;
  delete [] bg_ml_imgD;
}

void AMBP_Modeling::GetForegroundProbabilityImage(IplImage *fg_dist_img) {
  COpencvDataConversion<float, float> ODC1;
  COpencvDataConversion<uchar, uchar> ODC2;

  float *_fg_distD = ODC1.GetImageData(m_pBgDistImg);
  uchar *_fg_progI = ODC2.GetImageData(fg_dist_img);
  float *fg_distD = _fg_distD;
  uchar *fg_progI = _fg_progI;

  int channels = fg_dist_img->nChannels;

  int a, b;
  int img_length = fg_dist_img->width * fg_dist_img->height;
  uchar temp;
  for (a = 0; a < img_length; a++) {
    temp = cvRound(255.0f * ((*fg_distD++)));
    for (b = 0; b < channels; b++)
      *fg_progI++ = temp;
  }

  ODC2.SetImageData(fg_dist_img, _fg_progI);

  delete [] _fg_distD;
  delete [] _fg_progI;
}

void AMBP_Modeling::RemoveBackgroundLayers(PixelAMBPStruct *PAMBP, bool *removed_modes) {
  int a, b;
  int ambp_num = PAMBP->num;

  unsigned short* ambp_idxes = PAMBP->ambp_idxes;
  if (!removed_modes) {
    int removed_bg_layer_num = 0;
    for (a = 0; a < ambp_num; a++) {
      if (PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num && PAMBP->AMBPs[ambp_idxes[a]].weight < m_fMinBgLayerWeight) { // should be removed
        removed_bg_layer_num = PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num;
        ambp_num--;
        for (b = a; b < ambp_num; b++)
          ambp_idxes[b] = ambp_idxes[b + 1];
        break;
      }
    }
    if (removed_bg_layer_num) {
      for (a = 0; a < ambp_num; a++) {
        if (PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num > removed_bg_layer_num)
          PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num--;
      }
    }
  } else {
    int removed_bg_layer_nums[10];
    int removed_layer_num = 0;
    for (a = 0; a < ambp_num; a++) {
      if (removed_modes[a] && PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num) { // should be removed
        removed_bg_layer_nums[removed_layer_num++] = PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num;
      }
    }

    for (a = 0; a < ambp_num; a++) {
      if (removed_modes[a]) { // should be removed
        ambp_num--;
        for (b = a; b < ambp_num; b++)
          ambp_idxes[b] = ambp_idxes[b + 1];
      }
    }

    for (a = 0; a < ambp_num; a++) {
      for (b = 0; b < removed_layer_num; b++) {
        if (PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num > removed_bg_layer_nums[b])
          PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num--;
      }
    }
  }

  // sort the list of modes based on the weights of modes
  if (ambp_num != (int) PAMBP->num) {
    float weights[100], tot_weights = 0;
    for (a = 0; a < (int) ambp_num; a++) {
      weights[a] = PAMBP->AMBPs[ambp_idxes[a]].weight;
      tot_weights += weights[a];
    }

    // sort weights in the descent order
    QuickSort(weights, ambp_idxes, 0, (int) ambp_num - 1, false);

    // calculate the first potential background modes number, bg_num
    float threshold_weight = m_fBackgroundModelPercent*tot_weights;
    int bg_num = 0;
    tot_weights = 0;
    for (a = 0; a < (int) ambp_num; a++) {
      tot_weights += PAMBP->AMBPs[ambp_idxes[a]].weight;
      if (tot_weights > threshold_weight) {
        bg_num = a + 1;
        break;
      }
    }
    (*PAMBP).bg_num = bg_num;

  }

  PAMBP->num = ambp_num;

  float bg_layer_data[10];
  unsigned short bg_layer_idxes[10];
  int bg_layer_num;
  int tot_bg_layer_num = 0;
  for (a = 0; a < ambp_num; a++) {
    bg_layer_num = PAMBP->AMBPs[ambp_idxes[a]].bg_layer_num;
    if (bg_layer_num) {
      bg_layer_data[tot_bg_layer_num] = (float) bg_layer_num;
      bg_layer_idxes[tot_bg_layer_num++] = ambp_idxes[a];
    }
  }
  if (tot_bg_layer_num == 1) {
    PAMBP->AMBPs[bg_layer_idxes[0]].bg_layer_num = 1;
  } else if (tot_bg_layer_num) {
    // sort weights in the descent order
    QuickSort(bg_layer_data, bg_layer_idxes, 0, tot_bg_layer_num - 1, true);
    for (a = 0; a < tot_bg_layer_num; a++)
      PAMBP->AMBPs[bg_layer_idxes[a]].bg_layer_num = a + 1;
  }
}

void AMBP_Modeling::Postprocessing() {
  // post-processing for background subtraction results
  cvDilate(m_pFgMaskImg, m_pFgMaskImg, 0, 2);
  cvErode(m_pFgMaskImg, m_pFgMaskImg, 0, 2);

  /** Example of extracting and filtering the blobs of an image */

  // object that will contain blobs of inputImage
  CBlobResult blobs;

  IplImage *inputImage = m_pFgMaskImg;

  // Extract the blobs using a threshold of 100 in the image
  blobs = CBlobResult(inputImage, NULL, 0, true);

  // discard the blobs with less area than 100 pixels
  blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 100);

  // build an output image equal to the input but with 3 channels (to draw the coloured blobs)
  IplImage *outputImage;
  outputImage = cvCreateImage(cvSize(inputImage->width, inputImage->height), IPL_DEPTH_8U, 1);
  cvSet(outputImage, cvScalar(0));

  // plot the selected blobs in a output image
  CBlob filtered_blob;
  int a;
  for (a = 0; a < blobs.GetNumBlobs(); a++) {
    filtered_blob = blobs.GetBlob(a);
    filtered_blob.FillBlob(outputImage, cvScalar(255));
  }
  blobs.GetNthBlob(CBlobGetArea(), 0, filtered_blob);
  filtered_blob.FillBlob(outputImage, cvScalar(0));

  cvReleaseImage(&outputImage);
}

void AMBP_Modeling::GetFloatEdgeImage(IplImage *src, IplImage *dst) {
  if (src->nChannels > 1) {
    printf("Error: the input source image must be single-channel image!\n");
    exit(1);
  }
  if (dst->depth != IPL_DEPTH_32F) {
    printf("Error: the output edge image must be float image ranging in [0,1]!\n");
    exit(1);
  }

  uchar *src_x_data;
  float *dst_x_data;

  int x, y;
  for (y = 0; y < dst->height; y++) {
    src_x_data = (uchar*) (src->imageData + src->widthStep * y);
    dst_x_data = (float*) (dst->imageData + dst->widthStep * y);
    for (x = 0; x < dst->width; x++) {
      *dst_x_data++ = (float) (*src_x_data++) / 255.0f;
    }
  }
}

void AMBP_Modeling::UpdatePatternColorDistWeights(float *cur_pattern, float *bg_pattern) {
  return;

  int cur_true_num = 0, cur_false_num = 0, bg_true_num = 0, bg_false_num = 0;
  int a;

  for (a = 0; a < m_nAMBPLength; a++) {
    cur_true_num += (cur_pattern[a] > 0.5f ? 1 : 0);
    cur_false_num += (cur_pattern[a] < 0.5f ? 0 : 1);
    bg_true_num += (bg_pattern[a] > 0.5f);
    bg_false_num += (bg_pattern[a] < 0.5f);
  }
  m_fTextureWeight = expf(-(fabsf(cur_true_num - cur_false_num) + fabsf(bg_true_num - bg_false_num) + 0.8f) / (float) m_nAMBPLength);
  m_fTextureWeight = MAX(MIN(m_fTextureWeight, 0.5f), 0.1f);
  m_fColorWeight = 1.0f - m_fTextureWeight;
}

void AMBP_Modeling::SetValidPointMask(IplImage *maskImage, int mode) {
  if (mode == 1)
    SetBkMaskImage(maskImage);
  else
    cvAnd(m_pBkMaskImg, maskImage, m_pBkMaskImg);
}

void AMBP_Modeling::SetFrameRate(float frameDuration) {
  m_fModeUpdatingLearnRate = m_fModeUpdatingLearnRatePerSecond*frameDuration;
  m_fWeightUpdatingLearnRate = m_fWeightUpdatingLearnRatePerSecond*frameDuration;

  m_fFrameDuration = frameDuration;

  m_f1_ModeUpdatingLearnRate = 1.0f - m_fModeUpdatingLearnRate;
  m_f1_WeightUpdatingLearnRate = 1.0f - m_fWeightUpdatingLearnRate;
}

void AMBP_Modeling::Init(int width, int height) {
  IplImage* first_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  int ambp_level_num = 1;
  float radiuses[] = {2.0f};
  int neig_pt_nums[] = {6};
  Initialization(first_img, ambp_level_num, radiuses, neig_pt_nums);
  cvReleaseImage(&first_img);
}

int AMBP_Modeling::SetRGBInputImage(IplImage *inputImage, CvRect *roi) {
  if (!inputImage) {
    return 0;
  }
  if (inputImage->width != m_cvImgSize.width ||
    inputImage->height != m_cvImgSize.height ||
    inputImage->depth != IPL_DEPTH_8U ||
    inputImage->nChannels != 3) {
      return 0;
  }
  SetNewImage(inputImage, roi);
  return 1;
}

void AMBP_Modeling::SetParameters(int max_ambp_mode_num, float mode_updating_learn_rate_per_second, float weight_updating_learn_rate_per_second, float low_init_mode_weight) {
  m_nMaxAMBPModeNum = max_ambp_mode_num;
  m_fModeUpdatingLearnRate = mode_updating_learn_rate_per_second*m_fFrameDuration;
  m_fWeightUpdatingLearnRate = weight_updating_learn_rate_per_second*m_fFrameDuration;
  m_fLowInitialModeWeight = low_init_mode_weight;

  m_fModeUpdatingLearnRatePerSecond = mode_updating_learn_rate_per_second;
  m_fWeightUpdatingLearnRatePerSecond = weight_updating_learn_rate_per_second;

  m_f1_ModeUpdatingLearnRate = 1.0f - m_fModeUpdatingLearnRate;
  m_f1_WeightUpdatingLearnRate = 1.0f - m_fWeightUpdatingLearnRate;
}

Mat AMBP_Modeling::Process() {
  return BackgroundSubtractionProcess();
}

int AMBP_Modeling::SetForegroundMaskImage(IplImage* fg_mask_img) {
  if (!fg_mask_img) {
    return 0;
  }
  if (fg_mask_img->width != m_cvImgSize.width ||
    fg_mask_img->height != m_cvImgSize.height ||
    fg_mask_img->depth != IPL_DEPTH_8U ||
    fg_mask_img->nChannels != 1) {
      return 0;
  }

  m_pFgMaskImg = fg_mask_img;

  return 1;
}

int AMBP_Modeling::SetForegroundProbImage(IplImage* fg_prob_img) {
  if (!fg_prob_img) {
    return 0;
  }
  if (fg_prob_img->width != m_cvImgSize.width ||
    fg_prob_img->height != m_cvImgSize.height ||
    fg_prob_img->depth != IPL_DEPTH_8U) {
      return 0;
  }

  m_pFgProbImg = fg_prob_img;

  return 1;
}

void AMBP_Modeling::SetCurrentFrameNumber(unsigned long cur_frame_no) {
  m_nCurImgFrameIdx = cur_frame_no;
}

/*********************************************************************************/
/* AMBP Engine   */
/*********************************************************************************/

AMBP_engine::AMBP_engine() : firstTime(true), showOutput(true),
	bg_model_preload(""), saveModel(false), disableLearning(false), disableDetectMode(true), loadDefaultParams(true), 
	detectAfter(0), frameNumber(0)
{
}

AMBP_engine::~AMBP_engine()
{
	finish();
}

void AMBP_engine::setStatus(Status _status)
{
	status = _status;
}

void AMBP_engine::finish(void)
{
	cvReleaseImage(&fg_img);
	cvReleaseImage(&bg_img);
	cvReleaseImage(&fg_prob_img);
	cvReleaseImage(&fg_mask_img);
	cvReleaseImage(&fg_prob_img3);
	cvReleaseImage(&merged_img);

	delete BGS;
}

void AMBP_engine::process(const cv::Mat &img_input, cv::Mat &img_output, cv::Mat &bg_output, AMBPHistogram &ambp_hist)
{
	if(img_input.empty())
		return;

	CvSize img_size = cvSize(cvCeil((double) img_input.size().width), cvCeil((double) img_input.size().height));

	if(firstTime)
	{
		status = MLBGS_LEARN;
		org_img = new IplImage(img_input);

		fg_img = cvCreateImage(img_size, org_img->depth, org_img->nChannels);
		bg_img = cvCreateImage(img_size, org_img->depth, org_img->nChannels);
		fg_prob_img = cvCreateImage(img_size, org_img->depth, 1);
		fg_mask_img = cvCreateImage(img_size, org_img->depth, 1);
		fg_prob_img3 = cvCreateImage(img_size, org_img->depth, org_img->nChannels);
		merged_img = cvCreateImage(cvSize(img_size.width * 2, img_size.height * 2), org_img->depth, org_img->nChannels);

        BGS = new AMBP_Modeling();
		BGS->Init(img_size.width, img_size.height);
		BGS->SetForegroundMaskImage(fg_mask_img);
		BGS->SetForegroundProbImage(fg_prob_img);

		frame_duration = 1.0 / 10.0;
		mode_learn_rate_per_second = 0.5;
		weight_learn_rate_per_second = 0.5;
		init_mode_weight = 0.05;
		max_mode_num = 5;
		weight_updating_constant = 5.0;
		texture_weight = 0.5;
		bg_mode_percent = 0.6;
		pattern_neig_half_size = 4;
		pattern_neig_gaus_sigma = 3.0;
		bg_prob_threshold = 0.2;
		bg_prob_updating_threshold = 0.2;
		robust_AMBP_constant = 3;
		min_noised_angle = 10.0 / 180.0 * PI;
		shadow_rate = 0.6;
		highlight_rate = 1.2;
		bilater_filter_sigma_s = 3.0;
		bilater_filter_sigma_r = 0.1;

        BGS->m_nMaxAMBPModeNum = max_mode_num;
		BGS->m_fWeightUpdatingConstant = weight_updating_constant;
		BGS->m_fTextureWeight = texture_weight;
		BGS->m_fBackgroundModelPercent = bg_mode_percent;
		BGS->m_nPatternDistSmoothNeigHalfSize = pattern_neig_half_size;
		BGS->m_fPatternDistConvGaussianSigma = pattern_neig_gaus_sigma;
		BGS->m_fPatternColorDistBgThreshold = bg_prob_threshold;
		BGS->m_fPatternColorDistBgUpdatedThreshold = bg_prob_updating_threshold;
        BGS->m_fRobustColorOffset = robust_AMBP_constant;
		BGS->m_fMinNoisedAngle = min_noised_angle;
		BGS->m_fRobustShadowRate = shadow_rate;
		BGS->m_fRobustHighlightRate = highlight_rate;
		BGS->m_fSigmaS = bilater_filter_sigma_s;
		BGS->m_fSigmaR = bilater_filter_sigma_r;

		BGS->SetFrameRate(frame_duration);
		BGS->SetParameters(max_mode_num, mode_learn_rate_per_second, weight_learn_rate_per_second, init_mode_weight);

		delete org_img;
    }

	if(detectAfter > 0 && detectAfter == frameNumber)
	{
        status = MLBGS_DETECT;

		mode_learn_rate_per_second = 0.01;
		weight_learn_rate_per_second = 0.01;
		init_mode_weight = 0.001;

		BGS->SetParameters(max_mode_num, mode_learn_rate_per_second, weight_learn_rate_per_second, init_mode_weight);

        BGS->m_disableLearning = disableLearning;
	}

	IplImage* img = new IplImage(img_input);

	BGS->SetRGBInputImage(img);
    Mat ambpImg = BGS->Process();
    if(!ambpImg.empty())
        Histogram(ambpImg, &ambp_hist);

	BGS->GetBackgroundImage(bg_img);
	BGS->GetForegroundImage(fg_img);
	BGS->GetForegroundProbabilityImage(fg_prob_img3);
	BGS->GetForegroundMaskImage(fg_mask_img);
	BGS->MergeImages(4, img, bg_img, fg_prob_img3, fg_img, merged_img);

	img_merged = cv::Mat(merged_img);
	img_foreground = cv::Mat(fg_mask_img);
	img_background = cv::Mat(bg_img);

    img_foreground.copyTo(img_output);
	img_background.copyTo(bg_output);

    delete img;

	firstTime = false;
	frameNumber++;
}

void AMBP_engine::Histogram(Mat texture, AMBPHistogram* curTextureHist)
{
    for(int i = 0; i < NUM_BINS; i++)
    {
        curTextureHist->hist_val[i] = 0;
    }

    for(int y = 0; y < texture.rows; ++y)
    {
        for(int x = 0; x < texture.cols; ++x)
        {
            curTextureHist->hist_val[texture.at<uchar>(y, x)]++;
        }
    }
}
