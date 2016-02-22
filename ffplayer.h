#ifndef __FFPLAYER_H__
#define __FFPLAYER_H__

	#include "ffplay_common.h"


	//----------------------------------------------------------------------------------------//
	// load & unload media
	int		init_ffmpeg_lib();
	int		uninit_ffmpeg_lib();

	//////////////////////////////////////////////////////////////////////////
    int		load_media(const char * file_name);
	void	unload_media(int open_index);
	void	unload_mediacontext(MediaContext * mediaCtx);

	int		find_open_index();

    FrameBuff* 	get_video_buff(int open_index);


	//----------------------------------------------------------------------------------------//
	// FFMPEG
	// decode video packet to frame
	int		decode_video_frame(MediaContext * mediaCtx, AVPacket* pkt_in, AVFrame** frame_out);
	int		decode_audio_frame(MediaContext * mediaCtx, AVPacket* pkt_in);
	int		_get_video_frame(MediaContext * mediaCtx, double sec, AVFrame** frame_out, int* video_must_free);

	// catch video frame
	int		catch_video_frame(int open_index, double sec);
	int		get_audio_frame(int open_index, void * buffer, int buf_size, double sec);

	// seek media
	int		seek_media(int open_index, double sec);
	int		clear_frame(int frame_index);
	int		clear_video_screen(int index);
	int		get_frame_rate(int open_index);


	// get media total time
	double	get_total_time(int open_index);
	int64_t	get_time_from_byte(MediaContext * mediaCtx, int64_t bytes);
	int64_t	get_byte_from_time(MediaContext * mediaCtx, int64_t pts);

	//----------------------------------------------------------------------------------------//
	//	Render & Shader
	void	SetGLScreenSize(int width, int height);

	void	InitRenderFrame();
	void 	OnRenderFrame(char* pGLBuff);

#endif
