#pragma once
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include<libavfilter/avfilter.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include "libavutil/time.h"
#include <libavutil/mathematics.h>
    
}
#include<iostream>


class MyEncoder
{
public:
    MyEncoder();
public:
    AVFrame *m_pRGBFrame;   //帧对象  
    AVFrame *m_pYUVFrame;   //帧对象  
    AVCodec *pCodecH264;    //编码器  
    AVCodecContext *c;      //编码器数据结构对象
    uint8_t *yuv_buff;      //yuv图像数据区  
    uint8_t *rgb_buff;      //rgb图像数据区  
    SwsContext *scxt;       //图像格式转换对象  
    uint8_t *outbuf;        //编码出来视频数据缓存  
    int outbuf_size;        //编码输出数据去大小  
    int nDataLen;           //rgb图像数据区长度  
    int width;              //输出视频宽度  
    int height;             //输出视频高度  
    AVPacket pkt;            //数据包结构体
    AVFormatContext *fmtctx = NULL;//数据帧
    AVOutputFormat *ofmt = NULL;
    long int frame_index=0;
    int64_t calc_duration;
    int64_t start_time=0;
    AVStream *pOutStream;

    char* cPacketData;
    uint8_t* cExtradata;

    int sps_id_add;

public:
    void Ffmpeg_Encoder_Init();//初始化  
    void Ffmpeg_Encoder_Setpara(AVCodecID mycodeid, int vwidth, int vheight, int bit_rate);//设置参数,第一个参数为编码器,第二个参数为压缩出来的视频的宽度，第三个为其高度，第四个为码率
    // void Ffmpeg_Encoder_Encode(FILE *file,uint8_t *data);//编码并写入数据到文件
    void Ffmpeg_Encoder_Encode(uint8_t *data);
    void Ffmpeg_Encoder_Close();//关闭 
};
