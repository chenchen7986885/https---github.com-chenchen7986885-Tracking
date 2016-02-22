#ifndef LANDMARK_DETECTOR_H
#define LANDMARK_DETECTOR_H

#include "opencv/cvaux.h"

using namespace cv;
using namespace std;

#ifdef _MSC_VER
typedef unsigned char uint8_t;
typedef char int8_t;
typedef unsigned __int16 uint16_t;
typedef __int16 int16_t;
typedef unsigned __int32 uint32_t;
typedef __int32 int32_t;
typedef unsigned __int64 uint64_t;
typedef __int64 int64_t;
#else
#include <stdint.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

// LBP feature
#define LIBLBP_INDEX(ROW, COL, NUM_ROWS) ((COL)*(NUM_ROWS)+(ROW))
#define LIBLBP_MIN(A,B) ((A) > (B) ? (B) : (A))

typedef uint32_t t_index;
extern void liblbp_pyr_features_sparse(t_index *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols );
extern void liblbp_pyr_features(char *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols );
extern double liblbp_pyr_dotprod(double *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols);
extern void liblbp_pyr_addvec(int64_t *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols);
extern void liblbp_pyr_subvec(int64_t *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols);
extern uint32_t liblbp_pyr_get_dim(uint16_t img_nRows, uint16_t img_nCols, uint16_t nPyramids);

// face landmark detect
#define INDEX(ROW, COL, NUM_ROWS) ((COL)*(NUM_ROWS)+(ROW))
#define ROW(IDX, ROWS) (((IDX)-1) % (ROWS))
#define COL(IDX, ROWS) (((IDX)-1) / (ROWS))

typedef struct psig_struct {
    int * disp;
    int ROWS, COLS;
} FLANDMARK_PSIG;

typedef struct options_struct {
    uint8_t M;
    int * S;
    int bw[2], bw_margin[2];
    FLANDMARK_PSIG *PsiGS0, *PsiGS1, *PsiGS2;
    int PSIG_ROWS[3], PSIG_COLS[3];
} FLANDMARK_Options;

typedef struct lbp_struct {
    int winSize[2];
    uint8_t hop;
    uint32_t *wins;
    int WINS_ROWS, WINS_COLS;
} FLANDMARK_LBP;

typedef struct data_struct {
    FLANDMARK_LBP * lbp;
    int imSize[2];
    int * mapTable;
    FLANDMARK_Options options;
} FLANDMARK_Data;

typedef struct model_struct {
    double * W;
    int W_ROWS, W_COLS;
    FLANDMARK_Data data;
    uint8_t *normalizedImageFrame;
    double *bb;
    float *sf;
} FLANDMARK_Model;

typedef struct psi_struct {
    char * data;
    uint32_t PSI_ROWS, PSI_COLS;
} FLANDMARK_PSI;

typedef struct psi_sparse {
    uint32_t * idxs;
    uint32_t PSI_ROWS, PSI_COLS;
} FLANDMARK_PSI_SPARSE;

enum EError_T {
    NO_ERR = 0,
    ERROR_M = 1,
    ERROR_BW = 2,
    ERROR_BW_MARGIN = 3,
    ERROR_W = 4,
    ERROR_DATA_IMAGES = 5,
    ERROR_DATA_MAPTABLE = 6,
    ERROR_DATA_LBP = 7,
    ERROR_DATA_OPTIONS_S = 8,
    ERROR_DATA_OPTIONS_PSIG = 9,
    UNKNOWN_ERROR = 100
};

/*-- read / write structure Model from / to file procedures --*/
// Given the path to the file containing the model in binary form, this function will return a pointer to this model. It returns null pointer in the case of failure
FLANDMARK_Model * flandmark_init(const char* filename);

// This function dealocates the FLANDMARK_Model data structure
void flandmark_free(FLANDMARK_Model* model);

// Computes LBP features representing it as sparse matrix (i.e. only inices with ones are stored in connected list)
void flandmark_get_psi_mat_sparse(FLANDMARK_PSI_SPARSE* Psi, FLANDMARK_Model* model, int lbpidx);

/*-- dot product maximization with max-index return --*/
void flandmark_maximize_gdotprod(double *maximum, double *idx, const double *first, const double *second, const int *third, const int cols, const int tsize);

int flandmark_get_normalized_image_frame(Mat src, const int bbox[], double *bb, uint8_t *face_img, FLANDMARK_Model *model);

int flandmark_imcrop(Mat src, Mat &dst, const CvRect region);

void flandmark_argmax(double *smax, FLANDMARK_Options *options, const int *mapTable, FLANDMARK_PSI_SPARSE *Psi_sparse, double **q, double **g);

// Estimates positions of facial landmarks in the normalized image frame.
int flandmark_detect_base(uint8_t *face_image, FLANDMARK_Model *model, double *landmarks);

// Estimates positions of facial landmarks in the normalized image frame.
int flandmark_detect_base(uint8_t *face_image, FLANDMARK_Model *model, double *landmarks);

// Estimates positions of facial landmarks given the image and the bounding box of the detected face
int flandmark_detect(Mat src, int * bbox, FLANDMARK_Model *model, double *landmarks, int * bw_margin = 0);

#endif // LANDMARK_DETECTOR_H
