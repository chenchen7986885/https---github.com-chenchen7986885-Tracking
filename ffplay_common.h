#ifndef __COMMON_H__
#define __COMMON_H__


//#include <pthread.h>
//#include <cpu-features.h>

#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"

//#include "cmdutils.h"	
//#include "audioconvert.h"


//////////////////////////////////////////////////////////////////////////
#define QUEUE_SIZE					5

#define CODEC_TYPE_VIDEO			AVMEDIA_TYPE_VIDEO
#define CODEC_TYPE_AUDIO			AVMEDIA_TYPE_AUDIO

#define VIDEO_AUDIO_ID				0
#define AUDIO_ID					1
#define VIDEO_ID					2

#define AUDIO_DATA_ID   			1
#define VIDEO_DATA_ID   			2

#define VIDEO_TYPE_MP4				1
#define VIDEO_TYPE_3GP				2
#define VIDEO_TYPE_UNKNOWN			-1

#define OPEN_LIMIT					100

#define	ERR_SUCCESS					1
#define ERR_FILENAME				-1000
#define ERR_NO_OPENINDEX			-1001
#define ERR_OPEN_INPUT_FILE			-1002
#define ERR_FIND_STREAM_INFO		-1003
#define ERR_ALLOCATE_FRAME			-1004
#define ERR_NO_MEDIA_TYPE			-1005
#define ERR_NO_DECODER_CODEC		-1006
#define ERR_OPEN_DECODER_CODEC		-1007

#define STREAM_PIX_FMT				PIX_FMT_YUV420P /* default pix_fmt */

#define SAFE_FREE(v)	if(v) { free(v);v=NULL;}
#define SAFE_AV_FREE(v)	if(v) { av_free(v);v=NULL;}


//////////////////////////////////////////////////////////////////////////
// define struct

typedef struct _FrameBuff
{
    int width;		// real video width
    int height;		// real video height
    int nBuffSize;	// video frame buffer size
    int fps;
    char* pBuff;	// video frame buffer
    int	use;
}FrameBuff;

typedef struct _MediaContext
{
	struct AVFormatContext *		formatCtx;
	struct AVCodecContext *			codecCtx;
	struct SwsContext *				swsCtx;

	int								ref_count;
	int								eof;

	int								sample_fmt;
	int								sample_rate;
	int								channels;
	struct AVAudioConvert *			reformat_ctx;
	struct ReSampleContext *		resample_ctx;

	int								streamIdx;
	double							totalTime;

	int								mediaType;

	int								skip_request;
	int								abort_request;
	
	int								seek_request;
	int								seek_flags;
	int64_t							seek_pos;
	int64_t							seek_rel;

	int64_t							pts_request;
	double							time_base;
	double							time_base_rev;
	int								frame_rate;
} MediaContext;

#endif //__COMMON_H__
