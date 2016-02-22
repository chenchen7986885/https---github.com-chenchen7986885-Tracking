#include "isodata_threshold.h"

isodata_threshold::isodata_threshold()
{
}

int isodata_threshold::process(Mat src)
{
    int level;
    int maxValue = 256;
    double result, sum1, sum2, sum3, sum4;

    int min = 0;
    while ((src.data[min]==0) && (min<maxValue))
        min++;
    int max = maxValue;
    while ((src.data[max]==0) && (max>0))
        max--;
    if (min>=max) {
        level = maxValue/2;
        return level;
    }

    int movingIndex = min;
    do {
        sum1=sum2=sum3=sum4=0.0;
        for (int i=min; i<=movingIndex; i++) {
            sum1 += i*src.data[i];
            sum2 += src.data[i];
        }
        for (int i=(movingIndex+1); i<=max; i++) {
            sum3 += i*src.data[i];
            sum4 += src.data[i];
        }
        result = (sum1/sum2 + sum3/sum4) / 2.0;
        movingIndex++;
    } while ((movingIndex+1)<=result && movingIndex<max-1);

    level = floor(result/3 + 0.5);

    return level;
}
