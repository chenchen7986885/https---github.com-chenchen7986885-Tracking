#include "ffplayer.h"
#include <QThread>


//--------------------------------------------//
// FFmpeg

int		g_ScreenWidth = 0;
int		g_ScreenHeight = 0;

MediaContext *	g_mediaCtx[OPEN_LIMIT];
FrameBuff*		g_pVideoFrameBuff[OPEN_LIMIT];

AVFrame*		m_frame_scale = NULL;

////------------------------------------------------------------------------------------------------//
//// FFMpeg Video and Audio
//

int init_ffmpeg_lib()
{
    int i;

    printf("Init FFMpeg library. %d", 123);

    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    av_register_all();
    avcodec_register_all();

    for (i = 0; i < OPEN_LIMIT; i ++){
        g_mediaCtx[i] = NULL;
    }

    m_frame_scale = avcodec_alloc_frame();
    m_frame_scale->pts = 0;

    printf("Init FFMpeg library Succeded.");

    return ERR_SUCCESS;
}

int uninit_ffmpeg_lib()
{
    for(int i = 0; i < OPEN_LIMIT; i++)
    {
        SAFE_FREE(g_pVideoFrameBuff[i]->pBuff);
        SAFE_FREE(g_pVideoFrameBuff[i]);
    }
    return 0;
}


int load_media( const char * file_name)
{
    int result;
    int open_index;
    int i;
    int media_stream_idx;
    AVCodec * codec;
    MediaContext * mediaCtx;

    if ( file_name == NULL ){
        printf("Failed to retrieve Media File Name");
        return ERR_FILENAME;
    }

    printf("Open Media File %s", file_name);

    //Find out empty open index;
    open_index = find_open_index();
    if ( open_index < 0 )
    {
        printf("There is no empty media context");
        return open_index;
    }

    printf("Open Media step1");

    //Initialize MediaContext structure.
    mediaCtx = (MediaContext *) malloc(sizeof(MediaContext));
    mediaCtx->formatCtx = NULL;
    mediaCtx->codecCtx = NULL;
    mediaCtx->swsCtx = NULL;
    mediaCtx->ref_count = 1;
    mediaCtx->streamIdx = 0;
    mediaCtx->totalTime = 0;
    mediaCtx->skip_request = 0;
    mediaCtx->abort_request = 0;
    mediaCtx->seek_request = 0;
    mediaCtx->pts_request = 0;
    mediaCtx->sample_fmt = AV_SAMPLE_FMT_S16;
    mediaCtx->sample_rate = 44100;
    mediaCtx->channels = 2;
    mediaCtx->resample_ctx = NULL;
    mediaCtx->reformat_ctx = NULL;
    mediaCtx->eof = 0;


    printf("Open Media step2");

    //Open Input File to open_index media context.
    //If failed to open input file, return ERR_OPEN_INPUT_FILE error code.
    //result = av_open_input_file(&mediaCtx->formatCtx, file_name, NULL, 0, NULL);
    result = avformat_open_input(&mediaCtx->formatCtx, file_name, NULL, NULL);
    if (result < 0)
    {
        printf("Failed to open input file by av_open_input_file function, result=%d", result);
        mediaCtx->ref_count = 0;
        unload_mediacontext(mediaCtx);
        printf("3");
        //return ERR_OPEN_INPUT_FILE;
        return -1;
    }

    printf("Open Media step3");

    if (mediaCtx->formatCtx == NULL){
        printf("Open Media step3-1");
    }

    //Find out stream info from AVFormatContext.
    //If failed to find out stream info, return ERR_FIND_STREAM_INFO error code.
    result = avformat_find_stream_info(mediaCtx->formatCtx, NULL);
    if (result < 0)
    {
        printf("Failed to find out stream info by av_find_stream_info function");
        mediaCtx->ref_count = 0;
        unload_mediacontext(mediaCtx);
        return ERR_FIND_STREAM_INFO;
    }

    printf("Open Media step4");

    mediaCtx->mediaType = AVMEDIA_TYPE_VIDEO;
    mediaCtx->pts_request = AV_NOPTS_VALUE;

    //Find out matched media type from format context's opened streams.
    media_stream_idx = -1;
    for ( i = 0; i < (int)mediaCtx->formatCtx->nb_streams; i ++ ){
        if (mediaCtx->formatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO){
            media_stream_idx = i;
            break;
        }
    }

    printf("Open Media step5");

    //If there is no matched media type, return ERR_NO_MEDIA_TYPE error code.
    if (media_stream_idx == -1)
    {
        mediaCtx->ref_count = 0;

        printf("There is no matched media type");
        unload_mediacontext(mediaCtx);
        return ERR_NO_MEDIA_TYPE;
    }

    //Set codec context to codex context of opened index's media context.
    mediaCtx->codecCtx = mediaCtx->formatCtx->streams[media_stream_idx]->codec;

    printf("Open Media step6");

    //Find out decoder codec
    codec = avcodec_find_decoder(mediaCtx->codecCtx->codec_id);
    if (!codec)
    {
        mediaCtx->ref_count = 0;

        printf("Failed to find out decoder codec");
        unload_mediacontext(mediaCtx);
        return ERR_NO_DECODER_CODEC;
    }

    // decoding option. for smooth play
    av_opt_set_int(mediaCtx->codecCtx, "refcounted_frames", 1, 0);

    //Open decoder codec
#if LIBAVCODEC_VERSION_INT >= ((53<<16)+(8<<8)+0)
    result = avcodec_open2( mediaCtx->codecCtx, codec, NULL );
#else
    result = avcodec_open(mediaCtx->codecCtx, codec);
#endif


    if (result != 0)
    {
        mediaCtx->ref_count = 0;

        printf("Failed to open decoder codec");
        unload_mediacontext(mediaCtx);
        return ERR_OPEN_DECODER_CODEC;
    }

    printf("Open Media step7");

    //all good, set index so that nativeProcess() can now recognize the media stream
    mediaCtx->streamIdx = media_stream_idx;
    mediaCtx->totalTime = av_q2d(mediaCtx->formatCtx->streams[media_stream_idx]->time_base) * mediaCtx->formatCtx->streams[media_stream_idx]->duration;
    mediaCtx->time_base = (double)av_q2d(mediaCtx->formatCtx->streams[mediaCtx->streamIdx]->time_base);
    mediaCtx->time_base_rev = (double)(1.0 / av_q2d(mediaCtx->formatCtx->streams[mediaCtx->streamIdx]->time_base));

    int den = mediaCtx->formatCtx->streams[media_stream_idx]->avg_frame_rate.den;
    int num = mediaCtx->formatCtx->streams[media_stream_idx]->avg_frame_rate.num;

    if (den > 0)
        mediaCtx->frame_rate = (int)((int)num / (int)den);
    else
        mediaCtx->frame_rate = -1;

    printf("Succeded opening media. frame_rate=%d, num=%d, den=%d", mediaCtx->frame_rate, num, den);
    printf("Media time base is %d", av_q2d(mediaCtx->formatCtx->streams[media_stream_idx]->time_base));
    printf("Media duration is %d", mediaCtx->formatCtx->streams[media_stream_idx]->duration);
    printf("Media M=%d, n=%d", mediaCtx->formatCtx->streams[mediaCtx->streamIdx]->time_base.den, mediaCtx->formatCtx->streams[mediaCtx->streamIdx]->time_base.num);

    g_mediaCtx[open_index] = mediaCtx;

    //------------------------------------------------------------------------//
    int width = mediaCtx->codecCtx->width;
    int height = mediaCtx->codecCtx->height;

    int rgbRate = 3;
    int buff_size = width * height * rgbRate;

    g_pVideoFrameBuff[open_index] = (FrameBuff*)malloc(sizeof(FrameBuff));
    memset(g_pVideoFrameBuff[open_index], 0 , sizeof(FrameBuff));

    g_pVideoFrameBuff[open_index]->pBuff = (char*)malloc(buff_size);
    g_pVideoFrameBuff[open_index]->nBuffSize = buff_size;
    g_pVideoFrameBuff[open_index]->width = width;
    g_pVideoFrameBuff[open_index]->height = height;
	g_pVideoFrameBuff[open_index]->fps = mediaCtx->frame_rate;

    //------------------------------------------------------------------------//

    return open_index;

}

void unload_mediacontext( MediaContext * mediaCtx )
{
    if ( mediaCtx == NULL )
        return;

    int cnt = 0;
    while (mediaCtx->ref_count > 0)
    {
        printf("unload_mediacontext while, mediaCtx->ref_count=%d", mediaCtx->ref_count);

        if (cnt++ > 200)
            break;

        QThread::usleep(1000);
    }

    if ( mediaCtx->codecCtx )
        avcodec_close(mediaCtx->codecCtx);

    if ( mediaCtx->formatCtx )
    {
        avformat_close_input(&mediaCtx->formatCtx);
    }

    if ( mediaCtx->resample_ctx )
        audio_resample_close(mediaCtx->resample_ctx);

    free(mediaCtx);
}

void unload_media( int open_index )
{
    printf("unload_media 1");

    MediaContext * mediaCtx;

    if ( open_index < 0 || open_index >= OPEN_LIMIT )
        return;

    mediaCtx = g_mediaCtx[open_index];
    g_mediaCtx[open_index] = NULL;

    if ( mediaCtx == NULL )
        return;

    mediaCtx->abort_request = 1;
    mediaCtx->ref_count --;

    unload_mediacontext(mediaCtx);
    printf("unload_media 2");
}

int find_open_index()
{
    int open_index, i;

    open_index = ERR_NO_OPENINDEX;

    for ( i = 0; i < OPEN_LIMIT; i ++ )
    {
        if ( g_mediaCtx[i] == NULL )
        {
            open_index = i;
            break;
        }
    }

    return open_index;
}

FrameBuff* get_video_buff(int open_index)
{
    int			ret;
    int			sleep_time;

    MediaContext * mediaCtx;
    AVPacket	pkt;
    AVFrame*	frame;

    int rest = 0;
    mediaCtx = g_mediaCtx[open_index];
    if ( mediaCtx == NULL )
    {
        printf("media_read_thread null detected");
        return NULL;
    }

    printf("media_read_thread started");
    mediaCtx->ref_count ++;

    while(1)
    {
        sleep_time = 1000;

        printf("media_read_thread 1 --->");

        //If thread abort request flag is set, break from loop
        if ( mediaCtx->abort_request )
            break;

        //If media seek request flag is set, seek.
        if ( mediaCtx->seek_request )
        {
            int64_t seek_target = mediaCtx->seek_pos;
            int64_t seek_min = mediaCtx->seek_rel > 0 ? seek_target - mediaCtx->seek_rel + 2 : INT64_MIN;
            int64_t seek_max = mediaCtx->seek_rel < 0 ? seek_target - mediaCtx->seek_rel - 2 : INT64_MAX;

            mediaCtx->seek_request = 0;
            printf("seek media to pos %d", seek_target);

            //Seek the media stream.
            //If succeded in seeking, clear packet queue.
            ret = avformat_seek_file(mediaCtx->formatCtx, mediaCtx->streamIdx, seek_min, seek_target, seek_max, mediaCtx->seek_flags);
            //ret = avformat_seek_file(mediaCtx->formatCtx, -1, seek_min, seek_target, seek_max, mediaCtx->seek_flags);
            if (ret < 0)
            {
                printf("Failed to seek file");
            }
            else
            {
            }

            //kimyh be care....
            //avcodec_flush_buffers(mediaCtx->codecCtx);
            mediaCtx->eof = 0;
        }

        //Read the media frame from file.
        ret = av_read_frame(mediaCtx->formatCtx, &pkt);

        if (ret != 0)
        {
            if ( ret == AVERROR_EOF )
            {
                //printf("media read thread. end of file has reached");
            }

            //printf("media_read_thread <------ av_read_frame fail ret=%d", ret);

            mediaCtx->eof = 1;

            //break;
            QThread::usleep(1000);
            return NULL;
        }

        //If media stream is not matched with media stream, continue the loop.
        if ( pkt.stream_index != mediaCtx->streamIdx )
        {
            printf("media_read_thread <------ stream_index no same, pkt.stream_index=%d, mediaCtx.stream_index=%d, pkt.dts=%d", pkt.stream_index, mediaCtx->streamIdx, pkt.dts);

            av_free_packet(&pkt);
            QThread::usleep(20);

            continue;
        }

        mediaCtx->eof = 0;

        //kimyh be care....
        if ( mediaCtx->skip_request )
        {
            printf("media_read_thread skip_request --> pkt.dts = %d, mediaCtx->pts_request = %d", (int) pkt.dts, (int) mediaCtx->pts_request);

            if ( pkt.dts < mediaCtx->pts_request )
            {
                av_free_packet(&pkt);
                QThread::usleep(20);
                continue;
            }

            //if ( !(pkt.flags & AV_PKT_FLAG_KEY) )
            //{
            //	av_free_packet(&pkt);
            //	usleep(100);
            //	continue;
            //}
        }

        if ( mediaCtx->mediaType == CODEC_TYPE_VIDEO )
        {
            printf("media_read_thread 4-1");

            if (decode_video_frame(mediaCtx, &pkt, &frame) > 0)
            {
                mediaCtx->skip_request = 0;

                printf("catch_video_frame pop framed video->pkt_dts=%d, mediaCtx->pts_request=%d", (int)frame->pkt_dts, (int)mediaCtx->pts_request);

                if ( avpicture_fill((AVPicture*)m_frame_scale, (uint8_t*)g_pVideoFrameBuff[open_index]->pBuff, AV_PIX_FMT_RGB24, mediaCtx->codecCtx->width, mediaCtx->codecCtx->height) < 0){
                    printf("catch_video_frame avpicture_fill 2 error");

                    continue;
                }

                printf("catch_video_frame sws_scale 1 linesize=%d", m_frame_scale->linesize);

                mediaCtx->swsCtx = sws_getCachedContext(mediaCtx->swsCtx, mediaCtx->codecCtx->width, mediaCtx->codecCtx->height, mediaCtx->codecCtx->pix_fmt,
                                                        mediaCtx->codecCtx->width, mediaCtx->codecCtx->height, AV_PIX_FMT_RGB24, SWS_X , NULL, NULL, NULL);

                printf("catch_video_frame sws_scale 2, video_width=%d, video_height=%d", mediaCtx->codecCtx->width, mediaCtx->codecCtx->height);

                if ( sws_scale(mediaCtx->swsCtx, frame->data, frame->linesize, 0, mediaCtx->codecCtx->height, m_frame_scale->data, m_frame_scale->linesize) < 0){
                    printf("catch_video_frame sws_scale  error");

                    SAFE_AV_FREE(frame);

                    return NULL;
                }


                SAFE_AV_FREE(frame);

                printf("catch_video_frame succeded ===================>>>>");

                return g_pVideoFrameBuff[open_index];
            }
            else
            {
                if (mediaCtx->skip_request)
                {
                    sleep_time = 100;
                }
            }

            av_free_packet(&pkt);

            printf("media_read_thread 4-2");
        }

        printf("media_read_thread packet_queue_pushed <------ ");
    }

    mediaCtx->ref_count --;

    printf("media_read_thread end");

    return NULL;
}

// decode video packet to frame
int decode_video_frame(MediaContext * mediaCtx, AVPacket* pkt_in, AVFrame** frame_out)
{
    AVFrame * frame;
    int got_picture;

    printf("media_read_thread 9-1");

    frame = avcodec_alloc_frame();
    int len1 = avcodec_decode_video2(mediaCtx->codecCtx, frame, &got_picture, pkt_in);

    //printf("Decode Frame. pkt_dts = %d, pts_request = %d", frame->pkt_dts, (int) mediaCtx->pts_request);

    av_free_packet(pkt_in);


    if (got_picture) {
        *frame_out = frame;

        printf("media_read_thread 9-2");
        return 1;
    }

    printf("media_read_thread fail 9-3");

    //avcodec_default_release_buffer(mediaCtx->codecCtx, frame);
    av_free(frame);
    return 0;
}

int decode_audio_frame(MediaContext * mediaCtx, AVPacket* pkt_in)
{
    printf("decode_audio_frame s ---->");

    //	AVFrame * frame;
    //	int len1;
    //	int got_picture;
    //
    //	int16_t dec_buffer1[AVCODEC_MAX_AUDIO_FRAME_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    //	int16_t dec_buffer2[AVCODEC_MAX_AUDIO_FRAME_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    //	int16_t * temp_buffer1, * temp_buffer2;
    //	int frame_size;
    //	int pts_sum;
    ////	AVPacket bef_pkt;
    //
    //	pts_sum = 0;
    //
    //	while (pkt_in->size > 0)
    //	{
    //		if ( mediaCtx->abort_request )
    //		{
    //			printf("queue audio frame abort1");
    //
    //			pkt_in->size = 0;
    //			break;
    //		}
    //
    // 		frame_size = AVCODEC_MAX_AUDIO_FRAME_SIZE;
    //		len1 = avcodec_decode_audio3(mediaCtx->codecCtx, dec_buffer1, &frame_size, pkt_in);
    //
    // 		if (len1 < 0)
    //		{
    //			//printf("queue audio frame abort2, size = %d, bef_size = %d, res = %08X", pkt.size, bef_pkt.size, len1);
    //			pkt_in->size = 0;
    //			continue;
    //		}
    //
    //		pkt_in->data += len1;
    //		pkt_in->size -= len1;
    //
    //		if (frame_size <= 0)
    //		{
    //			//printf("queue audio frame abort3");
    //			//pkt.size = 0;
    //			continue;
    //		}
    //
    //		printf("decode_audio_frame 1, mediaCtx->codecCtx->sample_fmt=%d, mediaCtx->codecCtx->sample_rate=%d", mediaCtx->codecCtx->sample_fmt, mediaCtx->codecCtx->sample_rate);
    //
    //		/*
    // 		if(mediaCtx->codecCtx->sample_fmt > 5){
    //			printf("decode_audio_frame 1.1, mediaCtx->codecCtx->sample_fmt=%d", mediaCtx->codecCtx->sample_fmt);
    //			mediaCtx->codecCtx->sample_fmt = mediaCtx->codecCtx->sample_fmt - 5;
    //		}
    //		*/
    //
    // 		if (mediaCtx->codecCtx->sample_fmt != mediaCtx->sample_fmt)
    //		{
    //			printf("av_audio_convert 1, codecCtx->sample_fmt=%d, mediaCtx->sample_fmt=%d", mediaCtx->codecCtx->sample_fmt, mediaCtx->sample_fmt);
    //
    //
    //			if (mediaCtx->reformat_ctx == NULL)
    //				mediaCtx->reformat_ctx = av_audio_convert_alloc((enum AVSampleFormat) mediaCtx->sample_fmt, 1, (enum AVSampleFormat) mediaCtx->codecCtx->sample_fmt, 1, (const float *) NULL, 0);
    //
    //			if (mediaCtx->reformat_ctx == NULL)
    //			{
    //				printf("Failed to create audio convert context");
    //				pkt_in->size = 0;
    //				continue;
    //			}
    //
    //			if (mediaCtx->reformat_ctx)
    //			{
    //				const void *ibuf[6]= {dec_buffer1};
    //				void *obuf[6]= {dec_buffer2};
    //				int istride[6]= {av_get_bytes_per_sample(mediaCtx->codecCtx->sample_fmt)};
    //				int ostride[6]= {2};
    //				int len = frame_size / istride[0];
    //
    //				if (av_audio_convert(mediaCtx->reformat_ctx, obuf, ostride, ibuf, istride, len) < 0)
    //				{
    //					printf("Failed to convert audio frame");
    //					pkt_in->size = 0;
    //					continue;
    //				}
    //
    //				temp_buffer1 = dec_buffer2;
    //				temp_buffer2 = dec_buffer1;
    //				frame_size = len * 2;
    //			}
    //		}
    //		else
    //		{
    //			temp_buffer1 = dec_buffer1;
    //			temp_buffer2 = dec_buffer2;
    //		}
    //
    // 		if (mediaCtx->codecCtx->sample_rate != mediaCtx->sample_rate || mediaCtx->codecCtx->channels != mediaCtx->channels)
    //		{
    //			printf("audio_resample 1, codecCtx->sample_rate=%d, mediaCtx->sample_rate=%d", mediaCtx->codecCtx->sample_rate, mediaCtx->sample_rate);
    //
    //			if (mediaCtx->resample_ctx == NULL)
    //			{
    //				printf("Create audio resample context. %d, %d, %d, %d, %d, %d", (int) mediaCtx->channels, (int) mediaCtx->codecCtx->channels,
    //																					(int) mediaCtx->sample_rate, (int) mediaCtx->codecCtx->sample_rate, (int) mediaCtx->sample_fmt, (int) mediaCtx->codecCtx->sample_fmt);
    //				mediaCtx->resample_ctx = av_audio_resample_init(mediaCtx->channels, mediaCtx->codecCtx->channels, mediaCtx->sample_rate, mediaCtx->codecCtx->sample_rate,
    //																mediaCtx->sample_fmt, mediaCtx->sample_fmt, 0, 10, 0, 1.0);
    //			}
    //
    //			if (mediaCtx->resample_ctx == NULL)
    //			{
    //				printf("Failed to create audio resample context");
    //				pkt_in->size = 0;
    //				continue;
    //			}
    //
    //			frame_size = audio_resample(mediaCtx->resample_ctx, (short *) temp_buffer2, (short *) temp_buffer1, frame_size / (mediaCtx->codecCtx->channels * 2));
    //			frame_size = frame_size * mediaCtx->channels * 2;
    //		}
    //		else
    //		{
    //			temp_buffer2 = temp_buffer1;
    //		}
    //
    //		printf("decode_audio_frame 3, pkt_in->size=%d", pkt_in->size);
    //
    //		frame =(AVFrame*) av_malloc(frame_size + sizeof(AVFrame) + FF_INPUT_BUFFER_PADDING_SIZE);
    //		frame->data[0] = (uint8_t *) frame + sizeof(AVFrame);
    //		frame->linesize[0] = frame_size;
    //		frame->linesize[1] = 0;
    //		frame->linesize[2] = frame_size;
    //		frame->pkt_dts = (int64_t)(av_q2d(mediaCtx->formatCtx->streams[mediaCtx->streamIdx]->time_base) * (double) pkt_in->dts * 1000.0) + pts_sum;
    //		pts_sum += get_time_from_byte(mediaCtx, frame_size);
    //		memcpy(frame->data[0], temp_buffer2, frame_size);
    //
    // 		media_queue_push(&mediaCtx->mediaQueue, frame);
    //
    // 	}
    //
    //	printf("decode_audio_frame e <----");

    return 1;
}

int64_t get_time_from_byte(MediaContext * mediaCtx, int64_t bytes)
{
    return (int64_t) ((double) bytes * 1000.0 / (double) (2 * mediaCtx->channels * mediaCtx->sample_rate));
}

int64_t get_byte_from_time( MediaContext * mediaCtx, int64_t pts )
{
    return 2 * mediaCtx->channels * (int64_t) ((double) pts * (double) mediaCtx->sample_rate / 1000.0);
}

