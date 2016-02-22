#include "landmark_detector.h"

// --------------------------------------------------------------------------
// Function Name : liblbp_pyr_features_sparse
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void liblbp_pyr_features_sparse(t_index* vec, uint32_t vec_nDim, uint32_t* img, uint16_t img_nRows, uint16_t img_nCols)
{
    uint32_t offset, ww, hh, x, y, center, j, idx;
    uint8_t pattern;

    idx = 0;
    offset = 0;
    ww = img_nCols;
    hh = img_nRows;
    while(1)
    {
        for(x = 1; x < ww-1; x++)
        {
            for(y = 1; y< hh-1; y++)
            {
                pattern = 0;
                center = img[LIBLBP_INDEX(y, x, img_nRows)];
                if(img[LIBLBP_INDEX(y-1, x-1, img_nRows)] < center) pattern = pattern | 0x01;
                if(img[LIBLBP_INDEX(y-1, x, img_nRows)] < center)   pattern = pattern | 0x02;
                if(img[LIBLBP_INDEX(y-1, x+1, img_nRows)] < center) pattern = pattern | 0x04;
                if(img[LIBLBP_INDEX(y, x-1, img_nRows)] < center)   pattern = pattern | 0x08;
                if(img[LIBLBP_INDEX(y, x+1, img_nRows)] < center)   pattern = pattern | 0x10;
                if(img[LIBLBP_INDEX(y+1, x-1, img_nRows)] < center) pattern = pattern | 0x20;
                if(img[LIBLBP_INDEX(y+1, x, img_nRows)] < center)   pattern = pattern | 0x40;
                if(img[LIBLBP_INDEX(y+1, x+1, img_nRows)] < center) pattern = pattern | 0x80;

                vec[idx++] = offset+pattern;
                offset += 256;
            }
        }

        if(vec_nDim <= idx)
          return;

        if(ww % 2 == 1) ww--;
        if(hh % 2 == 1) hh--;

        ww = ww/2;

        for(x = 0; x < ww; x++)
          for(j = 0; j < hh; j++)
            img[LIBLBP_INDEX(j,x,img_nRows)] = img[LIBLBP_INDEX(j,2*x,img_nRows)] +
              img[LIBLBP_INDEX(j, 2*x+1, img_nRows)];

        hh = hh/2;

        for(y = 0; y < hh; y++)
          for(j = 0; j < ww; j++)
            img[LIBLBP_INDEX(y, j, img_nRows)] = img[LIBLBP_INDEX(2*y, j, img_nRows)] +
              img[LIBLBP_INDEX(2*y+1, j, img_nRows)];
    }

    return;
}

// --------------------------------------------------------------------------
// Function Name : liblbp_pyr_features
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void liblbp_pyr_features(char *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols )
{
    uint32_t offset, ww, hh, x, y,center,j ;
    uint8_t pattern;

    offset=0;
    ww=img_nCols;
    hh=img_nRows;

    while(1)
    {
        for(x = 1; x < ww-1; x++)
        {
            for(y = 1; y < hh-1; y++)
            {
                pattern = 0;
                center = img[LIBLBP_INDEX(y, x, img_nRows)];
                if(img[LIBLBP_INDEX(y-1, x-1, img_nRows)] < center) pattern = pattern | 0x01;
                if(img[LIBLBP_INDEX(y-1, x, img_nRows)] < center)   pattern = pattern | 0x02;
                if(img[LIBLBP_INDEX(y-1, x+1, img_nRows)] < center) pattern = pattern | 0x04;
                if(img[LIBLBP_INDEX(y, x-1, img_nRows)] < center)   pattern = pattern | 0x08;
                if(img[LIBLBP_INDEX(y, x+1, img_nRows)] < center)   pattern = pattern | 0x10;
                if(img[LIBLBP_INDEX(y+1, x-1, img_nRows)] < center) pattern = pattern | 0x20;
                if(img[LIBLBP_INDEX(y+1, x, img_nRows)] < center)   pattern = pattern | 0x40;
                if(img[LIBLBP_INDEX(y+1, x+1, img_nRows)] < center) pattern = pattern | 0x80;

                vec[offset+pattern]++;
                offset += 256;
            }
        }

        if(vec_nDim <= offset)
        return;

        if(ww % 2 == 1) ww--;
        if(hh % 2 == 1) hh--;

        ww = ww/2;
        for(x = 0; x < ww; x++)
            for(j = 0; j < hh; j++)
                img[LIBLBP_INDEX(j, x, img_nRows)] = img[LIBLBP_INDEX(j, 2*x, img_nRows)] +
                  img[LIBLBP_INDEX(j, 2*x+1, img_nRows)];

        hh = hh/2;
        for(y = 0; y < hh; y++)
            for(j = 0; j < ww; j++)
                img[LIBLBP_INDEX(y, j, img_nRows)] = img[LIBLBP_INDEX(2*y, j, img_nRows)] +
                  img[LIBLBP_INDEX(2*y+1, j, img_nRows)];
    }
}

// --------------------------------------------------------------------------
// Function Name : liblbp_pyr_dotprod
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
double liblbp_pyr_dotprod(double *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols)
{
    double dot_prod = 0;
    uint32_t offset=0;
    uint32_t ww, hh, center, x, y, j;
    uint8_t pattern;

    ww = img_nCols;
    hh = img_nRows;

    while(1)
    {
        for(x = 1; x < ww-1; x++)
        {
            for(y = 1; y < hh-1; y++)
            {
                pattern = 0;
                center = img[LIBLBP_INDEX(y, x, img_nRows)];
                if(img[LIBLBP_INDEX(y-1, x-1, img_nRows)] < center) pattern = pattern | 0x01;
                if(img[LIBLBP_INDEX(y-1, x, img_nRows)] < center)   pattern = pattern | 0x02;
                if(img[LIBLBP_INDEX(y-1, x+1, img_nRows)] < center) pattern = pattern | 0x04;
                if(img[LIBLBP_INDEX(y, x-1, img_nRows)] < center)   pattern = pattern | 0x08;
                if(img[LIBLBP_INDEX(y, x+1, img_nRows)] < center)   pattern = pattern | 0x10;
                if(img[LIBLBP_INDEX(y+1, x-1, img_nRows)] < center) pattern = pattern | 0x20;
                if(img[LIBLBP_INDEX(y+1, x, img_nRows)] < center)   pattern = pattern | 0x40;
                if(img[LIBLBP_INDEX(y+1, x+1, img_nRows)] < center) pattern = pattern | 0x80;

                dot_prod += vec[offset+pattern];
                offset += 256;
            }
        }
        if(vec_nDim <= offset)
          return(dot_prod);

        if(ww % 2 == 1) ww--;
        if(hh % 2 == 1) hh--;

        ww = ww/2;
        for(x = 0; x < ww; x++)
          for(j = 0; j < hh; j++)
            img[LIBLBP_INDEX(j, x, img_nRows)] = img[LIBLBP_INDEX(j, 2*x, img_nRows)] +
                                              img[LIBLBP_INDEX(j, 2*x+1, img_nRows)];

        hh = hh/2;
        for(y = 0; y < hh; y++)
          for(j = 0; j < ww; j++)
            img[LIBLBP_INDEX(y, j, img_nRows)] = img[LIBLBP_INDEX(2*y, j, img_nRows)] +
                                               img[LIBLBP_INDEX(2*y+1, j, img_nRows)];
    }
}

// --------------------------------------------------------------------------
// Function Name : liblbp_pyr_addvec
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void liblbp_pyr_addvec(int64_t *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols)
{
    uint32_t offset, ww, hh, x, y, center,j ;
    uint8_t pattern;

    offset = 0;
    ww = img_nCols;
    hh = img_nRows;

    while(1)
    {
        for(x = 1; x < ww-1; x++)
        {
            for(y = 1; y < hh-1; y++)
            {
                pattern = 0;
                center = img[LIBLBP_INDEX(y, x, img_nRows)];
                if(img[LIBLBP_INDEX(y-1, x-1, img_nRows)] < center) pattern = pattern | 0x01;
                if(img[LIBLBP_INDEX(y-1, x, img_nRows)] < center)   pattern = pattern | 0x02;
                if(img[LIBLBP_INDEX(y-1, x+1, img_nRows)] < center) pattern = pattern | 0x04;
                if(img[LIBLBP_INDEX(y, x-1, img_nRows)] < center)   pattern = pattern | 0x08;
                if(img[LIBLBP_INDEX(y, x+1, img_nRows)] < center)   pattern = pattern | 0x10;
                if(img[LIBLBP_INDEX(y+1, x-1, img_nRows)] < center) pattern = pattern | 0x20;
                if(img[LIBLBP_INDEX(y+1, x, img_nRows)] < center)   pattern = pattern | 0x40;
                if(img[LIBLBP_INDEX(y+1, x+1, img_nRows)] < center) pattern = pattern | 0x80;

                vec[offset+pattern]++;
                offset += 256;
            }
        }

        if(vec_nDim <= offset)
          return;

        if(ww % 2 == 1) ww--;
        if(hh % 2 == 1) hh--;

        ww = ww/2;
        for(x = 0; x < ww; x++)
          for(j = 0; j < hh; j++)
            img[LIBLBP_INDEX(j, x, img_nRows)] = img[LIBLBP_INDEX(j, 2*x, img_nRows)] +
                 img[LIBLBP_INDEX(j,2*x+1,img_nRows)];

        hh = hh/2;
        for(y = 0; y < hh; y++)
          for(j = 0; j < ww; j++)
            img[LIBLBP_INDEX(y, j, img_nRows)] = img[LIBLBP_INDEX(2*y, j, img_nRows)] +
              img[LIBLBP_INDEX(2*y+1, j, img_nRows)];

    }
}

// --------------------------------------------------------------------------
// Function Name : liblbp_pyr_subvec
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void liblbp_pyr_subvec(int64_t *vec, uint32_t vec_nDim, uint32_t *img, uint16_t img_nRows, uint16_t img_nCols)
{
    uint32_t offset, ww, hh, x, y,center,j ;
    uint8_t pattern;

    offset = 0;
    ww = img_nCols;
    hh = img_nRows;

    while(1)
    {
        for(x = 1; x < ww-1; x++)
        {
            for(y = 1; y < hh-1; y++)
            {
                pattern = 0;
                center = img[LIBLBP_INDEX(y,x,img_nRows)];
                if(img[LIBLBP_INDEX(y-1, x-1, img_nRows)] < center) pattern = pattern | 0x01;
                if(img[LIBLBP_INDEX(y-1, x, img_nRows)] < center)   pattern = pattern | 0x02;
                if(img[LIBLBP_INDEX(y-1, x+1, img_nRows)] < center) pattern = pattern | 0x04;
                if(img[LIBLBP_INDEX(y, x-1, img_nRows)] < center)   pattern = pattern | 0x08;
                if(img[LIBLBP_INDEX(y, x+1, img_nRows)] < center)   pattern = pattern | 0x10;
                if(img[LIBLBP_INDEX(y+1, x-1, img_nRows)] < center) pattern = pattern | 0x20;
                if(img[LIBLBP_INDEX(y+1, x, img_nRows)] < center)   pattern = pattern | 0x40;
                if(img[LIBLBP_INDEX(y+1, x+1, img_nRows)] < center) pattern = pattern | 0x80;

                vec[offset+pattern]--;
                offset += 256;

            }
        }

        if(vec_nDim <= offset)
          return;

        if(ww % 2 == 1) ww--;
        if(hh % 2 == 1) hh--;

        ww = ww/2;
        for(x = 0; x < ww; x++)
          for(j = 0; j < hh; j++)
            img[LIBLBP_INDEX(j, x, img_nRows)] = img[LIBLBP_INDEX(j, 2*x, img_nRows)] +
              img[LIBLBP_INDEX(j, 2*x+1, img_nRows)];

        hh = hh/2;
        for(y = 0; y < hh; y++)
          for(j = 0; j < ww; j++)
            img[LIBLBP_INDEX(y, j, img_nRows)] = img[LIBLBP_INDEX(2*y, j, img_nRows)] +
              img[LIBLBP_INDEX(2*y+1, j, img_nRows)];

    }
}

// --------------------------------------------------------------------------
// Function Name : liblbp_pyr_get_dim
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
uint32_t liblbp_pyr_get_dim(uint16_t img_nRows, uint16_t img_nCols, uint16_t nPyramids)
{
    uint32_t w, h, N, i;

    for(w = img_nCols, h = img_nRows, N = 0, i = 0; i < nPyramids; i++)
    {
        if(LIBLBP_MIN(w,h) < 3)
          break;

        N += (w-2)*(h-2);

        if(w % 2) w--;
        if(h % 2) h--;
        w = w/2;
        h = h/2;
    }

    return(256*N);
}

// --------------------------------------------------------------------------
// Function Name : flandmark_init
// Auther : chen chen
// Data : 2016-02-15
// Face Landmark detector
// --------------------------------------------------------------------------
FLANDMARK_Model * flandmark_init(const char* filename)
{
    int *p_int = 0, tsize = -1, tmp_tsize = -1;
    uint8_t *p_uint8 = 0;

    FILE *fin;
    if ((fin = fopen(filename, "rb")) == NULL)
        return 0;

    // allocate memory for FLANDMARK_Model
    FLANDMARK_Model * tst = (FLANDMARK_Model*)malloc(sizeof(FLANDMARK_Model));

    //int fscan_ret = -1;
    if (fscanf(fin, " %c ", &tst->data.options.M) < 1)
    {
        return 0;
    }

    if (fscanf(fin, " %d %d ", &tst->data.options.bw[0], &tst->data.options.bw[1]) < 2)
    {
        return 0;
    }

    if (fscanf(fin, " %d %d ", &tst->data.options.bw_margin[0], &tst->data.options.bw_margin[1]) < 2)
    {
        return 0;
    }

    if (fscanf(fin, " %d %d ", &tst->W_ROWS, &tst->W_COLS) < 2)
    {
        return 0;
    }

    if (fscanf(fin, " %d %d ", &tst->data.imSize[0], &tst->data.imSize[1]) < 2)
    {
        return 0;
    }

    int M = tst->data.options.M;

    tst->data.lbp = (FLANDMARK_LBP*)malloc(M*sizeof(FLANDMARK_LBP));
    for (int idx = 0; idx < M; ++idx)
    {
        if (fscanf(fin, " %d %d ", &tst->data.lbp[idx].WINS_ROWS, &tst->data.lbp[idx].WINS_COLS) < 2)
        {
            return 0;
        }
    }

    for (int idx = 0; idx < 3; ++idx)
    {
        if (fscanf(fin, " %d %d ", &tst->data.options.PSIG_ROWS[idx], &tst->data.options.PSIG_COLS[idx]) < 2)
        {
            return 0;
        }
    }

    // load model.W -----------------------------------------------------------
    tst->W = (double*)malloc(tst->W_ROWS * sizeof(double));
    if (fread(tst->W, tst->W_ROWS * sizeof(double), 1, fin) != 1)
    {
        printf( "Error reading file %s\n", filename);
        return 0;
    }

    // load model.data.mapTable -----------------------------------------------
    p_int = (int*)malloc(M*4*sizeof(int));
    tst->data.mapTable = (int*)malloc(M*4*sizeof(int));
    if (fread(p_int, M*4*sizeof(int), 1, fin) != 1)
    {
        printf( "Error reading file %s\n", filename);
        return 0;
    }
    for (int i = 0; i < M*4; ++i)
    {
        tst->data.mapTable[i] = p_int[i];
    }
    free(p_int);

    // load model.data.lbp ---------------------------------------------------
    for (int idx = 0; idx < M; ++idx)
    {
        // lbp{idx}.winSize
        p_int = (int*)malloc(2*sizeof(int));
        if (fread(p_int, 2*sizeof(int), 1, fin) != 1)
        {
            printf( "Error reading file %s\n", filename);
            return 0;
        }
        for (int i = 0; i < 2; ++i)
        {
            tst->data.lbp[idx].winSize[i] = p_int[i];
        }
        free(p_int);

        // lbp{idx}.hop
        p_uint8 = (uint8_t*)malloc(sizeof(uint8_t));
        if (fread(p_uint8, sizeof(uint8_t), 1, fin) != 1)
        {
            printf( "Error reading file %s\n", filename);
            return 0;
        }
        tst->data.lbp[idx].hop = p_uint8[0];
        free(p_uint8);

        // lbp{idx}.wins
        tsize = tst->data.lbp[idx].WINS_ROWS*tst->data.lbp[idx].WINS_COLS;
        tst->data.lbp[idx].wins = (uint32_t*)malloc(tsize * sizeof(uint32_t));
        if (fread(tst->data.lbp[idx].wins, tsize * sizeof(uint32_t), 1, fin) != 1)
        {
            printf( "Error reading file %s\n", filename);
            return 0;
            //exit(1);
        }
    }

    // load model.options.S --------------------------------------------------
    p_int = (int*)malloc(4*M*sizeof(int));
    tst->data.options.S = (int*)malloc(4*M*sizeof(int));
    if (fread(p_int, 4*M*sizeof(int), 1, fin) != 1)
    {
        printf( "Error reading file %s\n", filename);
        return 0;
        //exit(1);
    }
    for (int i = 0; i < 4*M; ++i)
    {
        tst->data.options.S[i] = p_int[i];
    }
    free(p_int);

    // load model.options.PsiG -----------------------------------------------
    FLANDMARK_PSIG * PsiGi = NULL;
    for (int psigs_idx = 0; psigs_idx < 3; ++psigs_idx)
    {
        tsize = tst->data.options.PSIG_ROWS[psigs_idx]*tst->data.options.PSIG_COLS[psigs_idx];

        switch (psigs_idx)
        {
            case 0:
                tst->data.options.PsiGS0 = (FLANDMARK_PSIG*)malloc(tsize*sizeof(FLANDMARK_PSIG));
                PsiGi = tst->data.options.PsiGS0;
                break;
            case 1:
                tst->data.options.PsiGS1 = (FLANDMARK_PSIG*)malloc(tsize*sizeof(FLANDMARK_PSIG));
                PsiGi = tst->data.options.PsiGS1;
                break;
            case 2:
                tst->data.options.PsiGS2 = (FLANDMARK_PSIG*)malloc(tsize*sizeof(FLANDMARK_PSIG));
                PsiGi = tst->data.options.PsiGS2;
                break;
        }

        for (int idx = 0; idx < tsize; ++idx)
        {
            // disp ROWS
            p_int = (int*)malloc(sizeof(int));
            if (fread(p_int, sizeof(int), 1, fin) != 1)
            {
                printf( "Error reading file %s\n", filename);
                return 0;
                //exit(1);
            }
            PsiGi[idx].ROWS = p_int[0];
            free(p_int);
            // disp COLS
            p_int = (int*)malloc(sizeof(int));
            if (fread(p_int, sizeof(int), 1, fin) != 1)
            {
                printf( "Error reading file %s\n", filename);
                return 0;
                //exit(1);
            }
            PsiGi[idx].COLS = p_int[0];
            free(p_int);
            // disp
            tmp_tsize = PsiGi[idx].ROWS*PsiGi[idx].COLS;
            PsiGi[idx].disp = (int*)malloc(tmp_tsize*sizeof(int));
            if (fread(PsiGi[idx].disp, tmp_tsize*sizeof(int), 1, fin) != 1)
            {
                printf( "Error reading file %s\n", filename);
                return 0;
                //exit(1);
            }
        }
    }

    fclose(fin);

    tst->normalizedImageFrame = (uint8_t*)calloc(tst->data.options.bw[0]*tst->data.options.bw[1], sizeof(uint8_t));

    tst->bb = (double*)calloc(4, sizeof(double));

    tst->sf = (float*)calloc(2, sizeof(float));

    return tst;
}

// --------------------------------------------------------------------------
// Function Name : flandmark_free
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void flandmark_free(FLANDMARK_Model* model)
{
    FLANDMARK_PSIG *PsiGi = NULL;
    for (int psig_idx = 0; psig_idx < 3; ++psig_idx)
    {
        switch(psig_idx)
        {
            case 0:
                PsiGi = (model->data.options.PsiGS0);
                break;
            case 1:
                PsiGi = (model->data.options.PsiGS1);
                break;
            case 2:
                PsiGi = (model->data.options.PsiGS2);
                break;
        }

        int tsize = model->data.options.PSIG_ROWS[psig_idx] * model->data.options.PSIG_COLS[psig_idx];
        for (int i = 0; i < tsize; ++i)
        {
            free(PsiGi[i].disp);
        }
        free(PsiGi);
    }

    free(model->W);
    for (int i = 0; i < model->data.options.M; ++i)
    {
        free(model->data.lbp[i].wins);
    }

    free(model->data.lbp);
    free(model->data.options.S);
    free(model->data.mapTable);

    if (model->normalizedImageFrame)
        free(model->normalizedImageFrame);

    if (model->bb)
        free(model->bb);

    if (model->sf)
        free(model->sf);

    free(model);
}

// --------------------------------------------------------------------------
// Function Name : flandmark_get_psi_mat_sparse
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void flandmark_get_psi_mat_sparse(FLANDMARK_PSI_SPARSE* Psi, FLANDMARK_Model* model, int lbpidx)
{
    t_index * Features;
    uint8_t * Images = model->normalizedImageFrame;
    uint32_t im_H = (uint32_t)model->data.imSize[0];
    uint32_t im_W = (uint32_t)model->data.imSize[1];
    uint32_t * Wins = model->data.lbp[lbpidx].wins;
    uint16_t win_H = (uint16_t)model->data.lbp[lbpidx].winSize[0];
    uint16_t win_W = (uint16_t)model->data.lbp[lbpidx].winSize[1];
    uint16_t nPyramids = model->data.lbp[lbpidx].hop;
    uint32_t nDim = liblbp_pyr_get_dim(win_H, win_W, nPyramids)/256;
    uint32_t nData = model->data.lbp[lbpidx].WINS_COLS;

    uint32_t cnt0, mirror, x, x1, y, y1, idx, *win;
    uint8_t *img_ptr;

    Features = (t_index*)calloc(nDim*nData, sizeof(t_index));
    if (Features == NULL)
    {
        printf( "Not enough memory for LBP features.\n");
        exit(1);
    }

    win = (uint32_t*)calloc(win_H*win_W, sizeof(uint32_t));
    if(win == NULL)
    {
        printf( "Not enough memory for cropped_window.\n");
        exit(1);
    }

    for(uint32_t i = 0; i < nData; ++i)
    {
        idx = Wins[INDEX(0,i,4)]-1;
        x1  = Wins[INDEX(1,i,4)]-1;
        y1  = Wins[INDEX(2,i,4)]-1;
        mirror = Wins[INDEX(3,i,4)];

        img_ptr = &Images[idx*im_H*im_W];

        cnt0 = 0;

        if(mirror == 0)
        {
            for(x = x1; x < x1+win_W; x++)
                for(y = y1; y < y1+win_H; y++)
                    win[cnt0++] = img_ptr[INDEX(y,x,im_H)];
        } else {
            for(x = x1+win_W-1; x >= x1; x--)
                for(y = y1; y < y1+win_H; y++)
                    win[cnt0++] = img_ptr[INDEX(y,x,im_H)];
        }
        liblbp_pyr_features_sparse(&Features[nDim*i], nDim, win, win_H, win_W);
    }

    Psi->PSI_COLS = nData;
    Psi->PSI_ROWS = nDim;
    Psi->idxs = Features;

    free(win);
}

// --------------------------------------------------------------------------
// Function Name : flandmark_argmax
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void flandmark_argmax(double *smax, FLANDMARK_Options *options, const int *mapTable, FLANDMARK_PSI_SPARSE *Psi_sparse, double **q, double **g)
{
    uint8_t M = options->M;

    // compute argmax
    int * indices = (int*)malloc(M*sizeof(int));
    int tsize = mapTable[INDEX(1, 3, M)] - mapTable[INDEX(1, 2, M)] + 1;

    // left branch - store maximum and index of s5 for all positions of s1
    int q1_length = Psi_sparse[1].PSI_COLS;

    double * s1 = (double *)calloc(2*q1_length, sizeof(double));
    double * s1_maxs = (double *)calloc(q1_length, sizeof(double));
    for (int i = 0; i < q1_length; ++i)
    {
        // dot product <g_5, PsiGS1>
        flandmark_maximize_gdotprod(
                //s2_maxs, s2_idxs,
                &s1[INDEX(0, i, 2)], (double*)&s1[INDEX(1, i, 2)],
                q[5], g[4], options->PsiGS1[INDEX(i, 0, options->PSIG_ROWS[1])].disp,
                options->PsiGS1[INDEX(i, 0, options->PSIG_ROWS[1])].COLS, tsize
                );
        s1[INDEX(0, i, 2)] += q[1][i];
    }
    for (int i = 0; i < q1_length; ++i)
    {
        s1_maxs[i] = s1[INDEX(0, i, 2)];
    }

    // right branch (s2->s6) - store maximum and index of s6 for all positions of s2
    int q2_length = Psi_sparse[2].PSI_COLS;
    double * s2 = (double *)calloc(2*q2_length, sizeof(double));
    double * s2_maxs = (double *)calloc(q2_length, sizeof(double));
    for (int i = 0; i < q2_length; ++i)
    {
        // dot product <g_6, PsiGS2>
        flandmark_maximize_gdotprod(
                //s2_maxs, s2_idxs,
                &s2[INDEX(0, i, 2)], (double*)&s2[INDEX(1, i, 2)],
                q[6], g[5], options->PsiGS2[INDEX(i, 0, options->PSIG_ROWS[2])].disp,
                options->PsiGS2[INDEX(i, 0, options->PSIG_ROWS[2])].COLS, tsize);
        s2[INDEX(0, i, 2)] += q[2][i];
    }
    for (int i = 0; i < q2_length; ++i)
    {
        s2_maxs[i] = s2[INDEX(0, i, 2)];
    }

    // the root s0 and its connections
    int q0_length = Psi_sparse[0].PSI_COLS;
    double maxs0 = -FLT_MAX; int maxs0_idx = -1;
    double maxq10 = -FLT_MAX, maxq20 = -FLT_MAX, maxq30 = -FLT_MAX, maxq40 = -FLT_MAX, maxq70 = -FLT_MAX;
    double * s0 = (double *)calloc(M*q0_length, sizeof(double));
    for (int i = 0; i < q0_length; ++i)
    {
        // q10
        maxq10 = -FLT_MAX;
        flandmark_maximize_gdotprod(
                &maxq10, &s0[INDEX(1, i, M)],
                s1_maxs, g[0], options->PsiGS0[INDEX(i, 0, options->PSIG_ROWS[0])].disp,
                options->PsiGS0[INDEX(i, 0, options->PSIG_ROWS[0])].COLS, tsize);
        s0[INDEX(5, i, M)] = s1[INDEX(1, (int)s0[INDEX(1, i, M)], 2)];
        // q20
        maxq20 = -FLT_MAX;
        flandmark_maximize_gdotprod(
                &maxq20, &s0[INDEX(2, i, M)],
                s2_maxs, g[1], options->PsiGS0[INDEX(i, 1, options->PSIG_ROWS[0])].disp,
                options->PsiGS0[INDEX(i, 1, options->PSIG_ROWS[0])].COLS, tsize);
        s0[INDEX(6, i, M)] = s2[INDEX(1, (int)s0[INDEX(2, i, M)], 2)];
        // q30
        maxq30 = -FLT_MAX;
        flandmark_maximize_gdotprod(
                &maxq30, &s0[INDEX(3, i, M)],
                q[3], g[2], options->PsiGS0[INDEX(i, 2, options->PSIG_ROWS[0])].disp,
                options->PsiGS0[INDEX(i, 2, options->PSIG_ROWS[0])].COLS, tsize);
        // q40
        maxq40 = -FLT_MAX;
        flandmark_maximize_gdotprod(
                &maxq40, &s0[INDEX(4, i, M)],
                q[4], g[3], options->PsiGS0[INDEX(i, 3, options->PSIG_ROWS[0])].disp,
                options->PsiGS0[INDEX(i, 3, options->PSIG_ROWS[0])].COLS, tsize);
        // q70
        maxq70 = -FLT_MAX;
        flandmark_maximize_gdotprod(
                &maxq70, &s0[INDEX(7, i, M)],
                q[7], g[6], options->PsiGS0[INDEX(i, 4, options->PSIG_ROWS[0])].disp,
                options->PsiGS0[INDEX(i, 4, options->PSIG_ROWS[0])].COLS, tsize);
        // sum q10+q20+q30+q40+q70
        if (maxs0 < maxq10+maxq20+maxq30+maxq40+maxq70+q[0][i])
        {
            maxs0_idx = i;
            s0[INDEX(0, i, M)] = i;
            maxs0 = maxq10+maxq20+maxq30+maxq40+maxq70+q[0][i];
        }
    }

    // get indices
    for (int i = 0; i < M; ++i)
    {
        indices[i] = (int)s0[INDEX(0, maxs0_idx, M)+i]+1;
    }

    // cleanup temp variables
    free(s0);
    free(s1); free(s1_maxs);
    free(s2); free(s2_maxs);

    // convert 1D indices to 2D coordinates of estimated positions
    //int * optionsS = &options->S[0];
    const int * optionsS = options->S;
    for (int i = 0; i < M; ++i)
    {
        int rows = optionsS[INDEX(3, i, 4)] - optionsS[INDEX(1, i, 4)] + 1;
        smax[INDEX(0, i, 2)] = float(COL(indices[i], rows) + optionsS[INDEX(0, i, 4)]);
        smax[INDEX(1, i, 2)] = float(ROW(indices[i], rows) + optionsS[INDEX(1, i, 4)]);
    }
    free(indices);
}

// --------------------------------------------------------------------------
// Function Name : flandmark_detect_base
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
int flandmark_detect_base(uint8_t* face_image, FLANDMARK_Model* model, double * landmarks)
{
    const int M = model->data.options.M;
    const double * W = model->W;
    int tsize = -1, cols = -1, rows = -1;
    const int * mapTable = model->data.mapTable;

    if (!model->normalizedImageFrame)
    {
        model->normalizedImageFrame = face_image;
    }

    // get PSI matrix
    FLANDMARK_PSI_SPARSE * Psi_sparse = (FLANDMARK_PSI_SPARSE*)malloc(M*sizeof(FLANDMARK_PSI_SPARSE));
    for (int idx = 0; idx < M; ++idx)
    {
        flandmark_get_psi_mat_sparse(&Psi_sparse[idx], model, idx);
    }

    // get Q and G
    double ** q = (double**)calloc(M, sizeof(double*));
    double ** g = (double**)calloc((M-1), sizeof(double*));

    int idx_qtemp = 0;

    for (int idx = 0; idx < M; ++idx)
    {
        // Q
        tsize = mapTable[INDEX(idx, 1, M)] - mapTable[INDEX(idx, 0, M)] + 1;

        double * q_temp = (double*)calloc(tsize, sizeof(double));
        memcpy(q_temp, W+mapTable[INDEX(idx, 0, M)]-1, tsize*sizeof(double));

        // sparse dot product <W_q, PSI_q>
        cols = Psi_sparse[idx].PSI_COLS; rows = Psi_sparse[idx].PSI_ROWS;
        uint32_t *psi_temp = Psi_sparse[idx].idxs;
        q[idx] = (double*)malloc(cols*sizeof(double));
        for (int i = 0; i < cols; ++i)
        {
            double dotprod = 0.0f;
            for (int j = 0; j < rows; ++j)
            {
                idx_qtemp = psi_temp[(rows*i) + j];
                dotprod += q_temp[ idx_qtemp ];
            }
            q[idx][i] = dotprod;
        }
        free(q_temp);

        // G
        if (idx > 0)
        {
            tsize = mapTable[INDEX(idx, 3, M)] - mapTable[INDEX(idx, 2, M)] + 1;
            g[idx - 1] = (double*)malloc(tsize*sizeof(double));
            memcpy(g[idx - 1], W+mapTable[INDEX(idx, 2, M)]-1, tsize*sizeof(double));
        }
    }

    // argmax
    flandmark_argmax(landmarks, &model->data.options, mapTable, Psi_sparse, q, g);

    // cleanup Psi_sparse[].idxs
    for (int i = 0; i < M; ++i)
    {
        free(Psi_sparse[i].idxs);
    }
    free(Psi_sparse);

    // cleanup q
    for (int i = 0; i < M; ++i)
    {
        free(q[i]);
    }
    free(q);

    // cleanup g
    for (int i = 0; i < M - 1; ++i)
    {
        free(g[i]);
    }
    free(g);

    return 0;
}

// --------------------------------------------------------------------------
// Function Name : flandmark_detect
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
int flandmark_detect(cv::Mat src, int *bbox, FLANDMARK_Model *model, double *landmarks, int *bw_margin)
{
    int retval = 0;

    if (bw_margin)
    {
        model->data.options.bw_margin[0] = bw_margin[0];
        model->data.options.bw_margin[1] = bw_margin[1];
    }

    // Get normalized image frame
    retval = flandmark_get_normalized_image_frame(src, bbox, model->bb, model->normalizedImageFrame, model);
    if (retval)
    {
        // flandmark_get_normlalized_image_frame ERROR;
        return 1;
    }

    // Call flandmark_detect_base
    retval = flandmark_detect_base(model->normalizedImageFrame, model, landmarks);
    if (retval)
    {
        // flandmark_detect_base ERROR
        return 2;
    }

    // transform coordinates of detected landmarks from normalized image frame back to the original image
    model->sf[0] = (float)(model->bb[2]-model->bb[0])/model->data.options.bw[0];
    model->sf[1] = (float)(model->bb[3]-model->bb[1])/model->data.options.bw[1];
    for (int i = 0; i < 2*model->data.options.M; i += 2)
    {
        landmarks[i]   = landmarks[i]*model->sf[0] + model->bb[0];
        landmarks[i+1] = landmarks[i+1]*model->sf[1] + model->bb[1];
    }

    return 0;
}

// --------------------------------------------------------------------------
// Function Name : flandmark_maximize_gdotprod
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
void flandmark_maximize_gdotprod(double * maximum, double * idx, const double * first, const double * second, const int * third, const int cols, const int tsize)
{
    *maximum = -FLT_MAX;
    *idx = -1;
    for (int dp_i = 0; dp_i < cols; ++dp_i)
    {
        double dotprod = 0.0f;
        for (int dp_j = 0; dp_j < tsize; ++dp_j)
        {
            dotprod += second[dp_j]*(double)(third[dp_i*tsize+dp_j]);
        }
        if (*maximum < first[dp_i]+dotprod)
        {
            *idx = dp_i;
            *maximum = first[dp_i]+dotprod;
        }
    }
}

// --------------------------------------------------------------------------
// Function Name : flandmark_imcrop
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
int flandmark_imcrop(cv::Mat src, cv::Mat &dst, const CvRect region)
{
    if (src.cols <= 0 || src.rows <= 0 || region.width <= 0 || region.height <= 0)
    {
        return 1;
    }

    if (src.type() != CV_8U)
    {
        return 1;
    }

	cv::Mat cropImg = cv::Mat(src, cv::Rect(region.x, region.y, region.width, region.height));    
    dst = cropImg.clone();

    return 0;
}

// --------------------------------------------------------------------------
// Function Name : flandmark_get_normalized_image_frame
// Auther : chen chen
// Data : 2016-02-15
// --------------------------------------------------------------------------
int flandmark_get_normalized_image_frame(cv::Mat src, const int bbox[], double *bb, uint8_t *face_img, FLANDMARK_Model *model)
{
    bool flag;
    int d[2];
    double c[2], nd[2];

    // extend bbox by bw_margin
    d[0] = bbox[2]-bbox[0]+1;  d[1] = bbox[3]-bbox[1]+1;
    c[0] = (bbox[2]+bbox[0])/2.0f; c[1] = (bbox[3]+bbox[1])/2.0f;
    nd[0] = d[0]*model->data.options.bw_margin[0]/100.0f + d[0];
    nd[1] = d[1]*model->data.options.bw_margin[1]/100.0f + d[1];

    bb[0] = (c[0] - nd[0]/2.0f);
    bb[1] = (c[1] - nd[1]/2.0f);
    bb[2] = (c[0] + nd[0]/2.0f);
    bb[3] = (c[1] + nd[1]/2.0f);

    flag = bb[0] > 0 && bb[1] > 0 && bb[2] < src.cols && bb[3] < src.rows
        && bbox[0] > 0 && bbox[1] > 0 && bbox[2] < src.cols && bbox[3] < src.rows;

    if (!flag)
    {
        return 1;
    }

    cv::Mat cropImg;
    IplImage *resizedImage = cvCreateImage(cvSize(model->data.options.bw[0], model->data.options.bw[1]), IPL_DEPTH_8U, 1);

    // crop and resize image to normalized frame
    if(flandmark_imcrop(src, cropImg, cvRect((int)bb[0], (int)bb[1], (int)bb[2]-(int)bb[0]+1, (int)bb[3]-(int)bb[1]+1)))
    {
        // something was bad
        return 1;
    }

    // resize
	IplImage croppedImg = cropImg;
    cvResize(&croppedImg, resizedImage, CV_INTER_CUBIC);

    // tranform IplImage to simple 1D uint8 array representing 2D uint8 normalized image frame
    for (int x = 0; x < model->data.options.bw[0]; ++x)
    {
        for (int y = 0; y < model->data.options.bw[1]; ++y)
        {
            face_img[INDEX(y, x, model->data.options.bw[1])] = (uint8_t)((resizedImage->imageData + resizedImage->widthStep*y)[x]);
        }
    }

    cvReleaseImage(&resizedImage);

    return 0;
}

