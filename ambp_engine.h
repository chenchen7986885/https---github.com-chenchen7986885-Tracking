#ifndef AMBP_ENGINE_H
#define AMBP_ENGINE_H

#include "opencv/cv.h"
#include "opencv2/opencv.hpp"

#include <fstream>
#include <iostream>

#include "common.h"
#include "ambp_objectextraction.h"

using namespace cv;
using namespace std;
using namespace AMBP_OBJECT;

/*********************************************************************************/
/* AMBP PARAM   */
/*********************************************************************************/

#define MAX_AMBP_MODE_NUM	5
#define ROBUST_COLOR_OFFSET	6.0f
#define LOW_INITIAL_MODE_WEIGHT	0.01f
#define MODE_UPDATING_LEARN_RATE	0.01f
#define WEIGHT_UPDATING_LEARN_RATE	0.01f
#define COLOR_MAX_MIN_OFFSET		5
#define BACKGROUND_MODEL_PERCENT	0.6f
#define PATTERN_COLOR_DIST_BACKGROUND_THRESHOLD	0.2f
#define PATTERN_DIST_SMOOTH_NEIG_HALF_SIZE	6
#define PATTERN_DIST_CONV_GAUSSIAN_SIGMA	2.5f
#define ROBUST_SHADOW_RATE	0.6f
#define ROBUST_HIGHLIGHT_RATE	1.20f
#define BINARY_PATTERM_ELEM(c1, c2, offset)	\
  ((float)(c2)-(float)(c1)+offset>0)
#define PI 3.141592653589793f

typedef struct _AMBP
{
  float* bg_pattern;                /* the average median binary pattern of background mode */
  float* bg_intensity;              /* the average color intensity of background mode */
  float* max_intensity;             /* the maximal color intensity of background mode */
  float* min_intensity;             /* the minimal color intensity of background mode */
  float weight;                     /* the weight of background mode, i.e. probability that the background mode belongs to background */
  float max_weight;                 /* the maximal weight of background mode */
  int bg_layer_num;                 /* the background layer number of background mode */
  unsigned long first_time;         /* the first time of background mode appearing */
  unsigned long last_time;          /* the last time of background model appearing */
  int freq;                         /* the appearing frequency */
  unsigned long layer_time;         /* the first time of background mode becoming a background layer */
}
AMBPStruct;

typedef struct _PixelAMBP
{
  AMBPStruct* AMBPs;                  /* the background modes */
  unsigned short* ambp_idxes;		/* the indices of background modes */
  unsigned int cur_bg_layer_no;
  unsigned int num;                 /* the total number of background modes */
  unsigned int bg_num;              /* the number of the first background modes for foreground detection */
  unsigned char* cur_intensity;		/* the color intensity of current pixel */
  float* cur_pattern;               /* the median binary pattern of current pixel */
  float matched_mode_first_time;	/* the index of currently matched pixel mode */
}
PixelAMBPStruct;

class BG_PIXEL_MODE
{
public:
  float* bg_ambp_pattern;			/* the average median binary pattern of background mode */
  float* bg_intensity;              /* the average color intensity of background mode */
  float* max_intensity;             /* the maximal color intensity of background mode */
  float* min_intensity;             /* the minimal color intensity of background mode */
  float weight;                     /* the weight of background mode, i.e. probability that the background mode belongs to background */
  float max_weight;                 /* the maximal weight of background mode */
  int bg_layer_num;                 /* the background layer number of background mode */

  int ambp_pattern_length;
  int color_channel;

  BG_PIXEL_MODE(int _ambp_pattern_length, int _color_channel=3) {
    ambp_pattern_length = _ambp_pattern_length;
    color_channel = _color_channel;

    bg_ambp_pattern = new float[ambp_pattern_length];
    bg_intensity = new float[color_channel];
    max_intensity = new float[color_channel];
    min_intensity = new float[color_channel];
  };

  virtual ~BG_PIXEL_MODE() {
    delete [] bg_ambp_pattern;
    delete [] bg_intensity;
    delete [] max_intensity;
    delete [] min_intensity;
  };
};

class BG_PIXEL_PATTERN
{
public:
  BG_PIXEL_MODE** pixel_MODEs;          /* the background modes */
  unsigned short* ambp_pattern_idxes;	/* the indices of background modes */
  unsigned int cur_bg_layer_no;
  unsigned int num;                     /* the total number of background modes */
  unsigned int bg_num;                  /* the number of the first background modes for foreground detection */
  unsigned char* cur_intensity;         /* the color intensity of current pixel */
  float* cur_ambp_pattern;               /* the median binary pattern of current pixel */

  int ambp_pattern_length;
  int color_channel;
  int pixel_mode_num;

  BG_PIXEL_PATTERN(int _pixel_mode_num, int ambp_pattern_length, int _color_channel=3) {
    pixel_mode_num = _pixel_mode_num;
    ambp_pattern_length = ambp_pattern_length;
    color_channel = _color_channel;

    pixel_MODEs = new BG_PIXEL_MODE*[pixel_mode_num];

    for ( int i = 0 ; i < pixel_mode_num ; i++ ) {
      pixel_MODEs[i] = new BG_PIXEL_MODE(ambp_pattern_length, _color_channel);
    }

    ambp_pattern_idxes = new unsigned short[pixel_mode_num];
    cur_intensity = new unsigned char[color_channel];
    cur_ambp_pattern = new float[ambp_pattern_length];
  };

  virtual ~BG_PIXEL_PATTERN() {
    delete [] ambp_pattern_idxes;
    delete [] cur_intensity;
    delete [] cur_ambp_pattern;

    for ( int i = 0 ; i < pixel_mode_num ; i++ )
      delete pixel_MODEs[i];
    delete [] pixel_MODEs;
  };
};

class IMAGE_BG_MODEL
{
  int pixel_length;

  BG_PIXEL_PATTERN** pixel_PATTERNs;

  IMAGE_BG_MODEL(int _pixel_length, int _pixel_mode_num, int _ambp_pattern_length, int _color_channel=3) {
    pixel_length = _pixel_length;

    pixel_PATTERNs = new BG_PIXEL_PATTERN*[pixel_length];
    for ( int i = 0 ; i < pixel_length ; i++ )
      pixel_PATTERNs[i] = new BG_PIXEL_PATTERN(_pixel_mode_num, _ambp_pattern_length, _color_channel);
  }
  virtual ~IMAGE_BG_MODEL() {
    for ( int i = 0 ; i < pixel_length ; i++ )
      delete pixel_PATTERNs[i];
    delete [] pixel_PATTERNs;
  }
};

/*********************************************************************************/
/* OpenCV Data Conversion   */
/*********************************************************************************/

template <class TI, class TM>
class COpencvDataConversion
{
public:
  TI * GetImageData(IplImage *img)
  {
    if ( !img->roi ) {
      int y;
      TI* img_data = new TI[img->width*img->height*img->nChannels];
      TI* temp = img_data;
      TI* x_data;

      for ( y = 0 ; y < img->height ; y++ ) {
        x_data = (TI*)(img->imageData + img->widthStep*y);
        int row_length = img->width*img->nChannels;
        memcpy(temp, x_data, sizeof(TI)*row_length);
        temp += row_length;
      }

      return img_data;
    }
    else {
      int y;
      TI* img_data = new TI[img->roi->width*img->roi->height*img->nChannels];
      TI* temp = img_data;
      TI* x_data;
      for ( y = img->roi->yOffset ; y < img->roi->yOffset+img->roi->height ; y++ ) {
        x_data = (TI*)(img->imageData + img->widthStep*y + img->roi->xOffset*sizeof(TI)*img->nChannels);
        int row_length = img->roi->width*img->nChannels;
        memcpy(temp, x_data, sizeof(TI)*row_length);
        temp += row_length;
      }
      return img_data;
    }
  };

  void SetImageData(IplImage *img, TI *img_data)
  {
    if ( !img->roi ) {
      int y;
      TI* temp = img_data;
      TI* x_data;
      for ( y = 0 ; y < img->height ; y++ ) {
        x_data = (TI*)(img->imageData + img->widthStep*y);
        int row_length = img->width*img->nChannels;
        memcpy(x_data, temp, sizeof(TI)*row_length);
        temp += row_length;
      }
    }
    else {
      int y;
      TI* temp = img_data;
      TI* x_data;
      for ( y = img->roi->yOffset ; y < img->roi->yOffset+img->roi->height ; y++ ) {
        x_data = (TI*)(img->imageData + img->widthStep*y + img->roi->xOffset*sizeof(TI)*img->nChannels);
        int row_length = img->roi->width*img->nChannels;
        memcpy(x_data, temp, sizeof(TI)*row_length);
        temp += row_length;
      }
    }
  }

  TM * GetMatData(CvMat *mat)
  {
    TM* mat_data = new TM[mat->width*mat->height];
    memcpy(mat_data, mat->data.ptr, sizeof(TM)*mat->width*mat->height);
    return mat_data;
  };

  void SetMatData(CvMat *mat, TM *mat_data)
  {
    memcpy(mat->data.ptr, mat_data, sizeof(TM)*mat->width*mat->height);
  }

  void ConvertData(IplImage *img_src, CvMat *mat_dst)
  {
    if ( img_src->nChannels > 1 ) {
      printf("Must be one-channel image for ConvertImageData!\n");
      exit(1);
    }

    TI* _img_data = GetImageData(img_src);
    TM* _mat_data = new TM[img_src->width*img_src->height];

    TI* img_data = _img_data;
    TM* mat_data = _mat_data;
    int i;
    for ( i = 0 ; i < img_src->width*img_src->height ; i++ )
      *mat_data++ = (TM)(*img_data++);

    SetMatData(mat_dst, _mat_data);

    delete [] _img_data;
    delete [] _mat_data;
  }

  void ConvertData(CvMat *mat_src, IplImage *img_dst)
  {
    if ( img_dst->nChannels > 1 ) {
      printf("Must be one-channel image for ConvertImageData!\n");
      exit(1);
    }

    TM* _mat_data = GetMatData(mat_src);
    TI* _img_data = new TI[mat_src->width*mat_src->height];

    TM* mat_data = _mat_data;
    TI* img_data = _img_data;

    int i;
    for ( i = 0 ; i < mat_src->width*mat_src->height ; i++ )
      *img_data++ = (TI)(*mat_data++);

    SetImageData(img_dst, _img_data);

    delete [] _img_data;
    delete [] _mat_data;
  }

  COpencvDataConversion() {};
  virtual ~COpencvDataConversion() {};
};

/*********************************************************************************/
/* Median Binary Pattern   */
/*********************************************************************************/

#define	GENERAL_AMBP	0
#define SYMMETRIC_AMBP	1

class MedianBinaryPattern
{
public:
  void CalImageDifferenceMap(IplImage *cent_img, IplImage *neig_img, float *pattern, CvRect *roi=NULL);
  void CalNeigPixelOffset(float radius, int tot_neig_pts_num, int neig_pt_idx, int &offset_x, int &offset_y);
  void CalShiftedImage(IplImage *src, int offset_x, int offset_y, IplImage *dst, CvRect *roi=NULL);
  void FreeMemories();
  void ComputeAMBP(Mat &frame, PixelAMBPStruct *PAMBP, Mat &MBP_Params, CvRect *roi=NULL);
  void SetNewImages(IplImage **new_imgs);

  IplImage** m_ppOrgImgs;			/* the original images used for computing the AMBP operators */

  void Initialization(IplImage **first_imgs, int imgs_num,
    int level_num, float *radius, int *neig_pt_num,
    float robust_white_noise = 3.0f, int type = GENERAL_AMBP);

  MedianBinaryPattern();
  virtual ~MedianBinaryPattern();

  float	m_fRobustWhiteNoise;		/* the robust noise value for computing the AMBP operator in each channel */

private:
  void SetShiftedMeshGrid(CvSize img_size, float offset_x, float offset_y, CvMat *grid_map_x, CvMat *grid_map_y);

  float*	m_pRadiuses;			/* the circle radiuses for the AMBP operator */
  int	m_nAMBPType;                 /* the type of computing AMBP operator */
  int*	m_pNeigPointsNums;          /* the numbers of neighboring pixels on multi-level circles */
  int	m_nImgsNum;                 /* the number of multi-channel image */
  int	m_nAMBPLevelNum;             /* the number of multi-level AMBP operator */
  CvSize	m_cvImgSize;			/* the image size (width, height) */

  CvPoint* m_pXYShifts;
  CvPoint	m_nMaxShift;

  IplImage* m_pShiftedImg;
};

/*********************************************************************************/
/* AMBP Modeling   */
/*********************************************************************************/

class AMBP_Modeling
{
public:
    void   Init(int width,int height);
    void   SetValidPointMask(IplImage* maskImage, int mode);
    void   SetFrameRate(float    frameDuration);
    void SetParameters(int max_ambp_mode_num,           // maximal AMBP mode number
    float mode_updating_learn_rate_per_second,          // background mode updating learning rate per second
    float weight_updating_learn_rate_per_second,        // mode's weight updating learning rate per second
    float low_init_mode_weight);
    int   SetRGBInputImage(IplImage  *  inputImage, CvRect *roi=NULL);
    int SetForegroundMaskImage(IplImage* fg_mask_img);
    int SetForegroundProbImage(IplImage* fg_prob_img);
    Mat   Process();

    void SetCurrentFrameNumber(unsigned long cur_frame_no);

    void GetForegroundMaskImage(IplImage *fg_mask_img);
    void GetForegroundImage(IplImage *fg_img, CvScalar bg_color=CV_RGB(0,255,0));
    void GetBackgroundImage(IplImage *bk_img);
    void GetForegroundProbabilityImage(IplImage* fg_prob_img);

    void GetBgLayerNoImage(IplImage *bg_layer_no_img, CvScalar* layer_colors=NULL, int layer_num=0);
    void GetLayeredBackgroundImage(int layered_no, IplImage *layered_bg_img, CvScalar empty_color=CV_RGB(0,0,0));
    void GetCurrentLayeredBackgroundImage(int layered_no, IplImage *layered_bg_img, IplImage *layered_fg_img=NULL,
    CvScalar layered_bg_bk_color=CV_RGB(0,0,0), CvScalar layered_fg_color=CV_RGB(255,0,0),
    int smooth_win=13, float smooth_sigma=3.0f, float below_layer_noise=0.5f, float above_layer_noise=0.3f, int min_blob_size=50);
    float DistAMBP(AMBPStruct *AMBP1, AMBPStruct *AMBP2);
    void GetColoredBgMultiLayeredImage(IplImage *bg_multi_layer_img, CvScalar *layer_colors);
    void UpdatePatternColorDistWeights(float *cur_pattern, float *bg_pattern);
    void Postprocessing();
    void GetFloatEdgeImage(IplImage *src, IplImage *dst);
    void RemoveBackgroundLayers(PixelAMBPStruct *PAMBP, bool *removed_modes=NULL);
    float CalColorRangeDist(unsigned char *cur_intensity, float *bg_intensity, float *max_intensity,
    float *min_intensity, float shadow_rate, float highlight_rate);
    float CalVectorsAngle(float *c1, unsigned char *c2, int length);
    float CalVectorsNoisedAngle(float *bg_color, unsigned char *noised_color, float offset, int length);
    void ComputeGradientImage(IplImage *src, IplImage *dst, bool bIsFloat);
    float CalColorBgDist(uchar *cur_intensity, float *bg_intensity, float *max_intensity, float *min_intensity);
    float CalPatternBgDist(float *cur_pattern, float *bg_pattern);

    void GetForegroundMaskMap(CvMat *fg_mask_mat);
    void Initialization(IplImage *first_img, int ambp_level_num, float *radiuses, int *neig_pt_nums);
    void GetCurrentBackgroundDistMap(CvMat *bk_dist_map);
    Mat BackgroundSubtractionProcess();
    void SetBkMaskImage(IplImage *mask_img);
    void SetNewImage(IplImage *new_img, CvRect *roi=NULL);

    void ResetAllParameters();
    void QuickSort(float *pData, unsigned short *pIdxes, long low, long high, bool bAscent);
    void UpdateBgPixelPattern(float *cur_pattern, AMBPStruct* curAMBP, PixelAMBPStruct *PAMBP, AMBPStruct* AMBPs, unsigned int ambp_num, unsigned short* ambp_idxes);
    void UpdateBgPixelColor(unsigned char* cur_intensity, float* bg_intensity);
    void Update_MAX_MIN_Intensity(unsigned char *cur_intensity, float *max_intensity, float *min_intensity);
    void MergeImages(int num, ...);

    int	m_nChannel;                                     /* most of opencv functions support 1,2,3 or 4 channels, for the input images */

    PixelAMBPStruct*	m_pPixelAMBPs;                  /* the AMBP texture patterns for each image */
    int	m_nMaxAMBPModeNum;                              /* the maximal number for the used AMBP pattern models */
    float	m_fModeUpdatingLearnRate;                   /* the background mode learning rate */
    float	m_fWeightUpdatingLearnRate;                 /* the background mode weight updating rate */
    float	m_f1_ModeUpdatingLearnRate;                 /* 1 - background_mode_learning_rate */
    float	m_f1_WeightUpdatingLearnRate;               /* 1 - background_mode_weight_updating_rate */
    float	m_fRobustColorOffset;                       /* the intensity offset robust to noise */
    float	m_fLowInitialModeWeight;                    /* the lowest weight of initial background mode */
    int	m_nAMBPLength;                                  /* the length of texture AMBP operator */
    float	m_fPatternColorDistBgThreshold;             /* the threshold value used to classify background and foreground */
    float	m_fPatternColorDistBgUpdatedThreshold;      /* the threshold value used to update the background modeling */
    float	m_fMinBgLayerWeight;                        /* the minimal weight to remove background layers */

    int	m_nPatternDistSmoothNeigHalfSize;               /* the neighboring half size of gaussian window to remove the noise on the distance map */
    float	m_fPatternDistConvGaussianSigma;            /* the gaussian sigma used to remove the noise on the distance map */

    float	m_fBackgroundModelPercent;                  /* the background mode percent, the first several background modes with high mode weights should be regarded as reliable background modes */

    float	m_fRobustShadowRate;                        /* the minimal shadow rate, [0.4, 0.7] */
    float	m_fRobustHighlightRate;                     /* the maximal highlight rate, [1.1, 1.4] */

    int	m_nAMBPImgNum;                                  /* the number of images used for texture AMBP feature */

    float	m_fMinAMBPBinaryProb;                       /* the minimal AMBP binary probability */
    float	m_f1_MinAMBPBinaryProb;                     /* 1 - minimal_AMBP_binary_probability */

    CvSize	m_cvImgSize;                                /* the image size (width, height) */

    unsigned long	m_nCurImgFrameIdx;                  /* the frame index of current image */

    bool	m_bUsedGradImage;                           /* the boolean variable signaling whether the gradient image is used or not for computing AMBP operator */

    bool	m_bUsedColorAMBP;                           /* true - multi-channel color image for AMBP operator, false - gray-scale image for AMBP operator  */

    MedianBinaryPattern	m_cAMBP;                    /* the class instant for computing AMBP (median binary pattern) texture feature */

    IplImage* m_pBkMaskImg;                             /* the mask image corresponding to the input image, i.e. all the masked pixels should be processed  */

    IplImage* m_pOrgImg;                                /* the original image */
    IplImage** m_ppOrgAMBPImgs;                         /* the multi-layer images used for AMBP feature extraction */
    IplImage* m_pFgImg;                                 /* the foreground image */
    IplImage* m_pBgImg;                                 /* the background image */
    IplImage* m_pFgMaskImg;                             /* the foreground mask image */
    IplImage* m_pBgDistImg;                             /* the background distance image (float) */
    IplImage* m_pEdgeImg;                               /* the edge image used for cross bilateral filter */
    IplImage* m_pFgProbImg;                             /* the foreground probability image (uchar) */

    IplImage* m_pFirstAppearingTimeMap;

    bool	m_disableLearning;
    float	m_fSigmaS;                                  /* sigma in the spatial domain for cross bilateral filter */
    float	m_fSigmaR;                                  /* sigma in the normalized intensity domain for cross bilateral filter */

    float	m_fTextureWeight;                           /* the weight value of texture AMBP feature for background modeling & foreground detection */

    float	m_fColorWeight;                             /* the weight value of color invariant feature for background modeling & foreground detection */

    float	m_fWeightUpdatingConstant;                  /* the constant ( >= 1 ) for 'hysteries' weight updating scheme(increase when matched, decrease when un-matched */

    float	m_fReliableBackgroundModeWeight;            /* the weight value for background mode which should be regarded as a reliable background mode, which is useful for multi-layer scheme */

    float	m_fMinNoisedAngle;                          /* the minimal angle value between the background color and the noised observed color */

    float	m_fMinNoisedAngleSine;                      /* the minimal angle sine value between the background color and the noised observed color */

    float	m_fFrameDuration;                           /* frame duration */

    float	m_fModeUpdatingLearnRatePerSecond, m_fWeightUpdatingLearnRatePerSecond;

    int m_nAMBPLevelNum;
    float m_pAMBPRadiuses[10];
    int m_pAMBPMeigPointNums[10];

    CvRect* m_pROI;
    AMBP_Modeling();
    virtual ~AMBP_Modeling();

};

/*********************************************************************************/
/* AMBP Engine   */
/*********************************************************************************/

class AMBP_engine
{
public:
	enum Status
	{
		MLBGS_NONE   = -1,
		MLBGS_LEARN  = 0,
		MLBGS_DETECT = 1
	};

public:
    AMBP_engine();
    ~AMBP_engine();

    void process(const Mat &img_input, Mat &img_foreground, Mat &img_background, AMBPHistogram &ambp_hist);
	void setStatus(Status status);

private:
	void finish(void);
    void Histogram(Mat texture, AMBPHistogram* curTextureHist);

private:
	bool firstTime;
	long long frameNumber;
	cv::Mat img_foreground;
	cv::Mat img_background;
	cv::Mat img_merged;	
	bool showOutput;
	bool saveModel;
	bool disableDetectMode;
	bool disableLearning;
	int detectAfter;
    AMBP_Modeling* BGS;
	Status status;
	IplImage* img;
	IplImage* org_img;
	IplImage* fg_img;
	IplImage* bg_img;
	IplImage* fg_prob_img;
	IplImage* fg_mask_img;
	IplImage* fg_prob_img3;
	IplImage* merged_img;
	std::string bg_model_preload;

	bool loadDefaultParams;

	int max_mode_num;
	float weight_updating_constant;
	float texture_weight;
	float bg_mode_percent;
	int pattern_neig_half_size;
	float pattern_neig_gaus_sigma;
	float bg_prob_threshold;
	float bg_prob_updating_threshold;
    int robust_AMBP_constant;
	float min_noised_angle;
	float shadow_rate;
	float highlight_rate;
	float bilater_filter_sigma_s;
	float bilater_filter_sigma_r;

	float frame_duration;

	float mode_learn_rate_per_second;
	float weight_learn_rate_per_second;
	float init_mode_weight;

	float learn_mode_learn_rate_per_second;
	float learn_weight_learn_rate_per_second;
	float learn_init_mode_weight;

	float detect_mode_learn_rate_per_second;
	float detect_weight_learn_rate_per_second;
	float detect_init_mode_weight;
};

#endif // AMBP_ENGINE_H
