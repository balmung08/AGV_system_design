#ifndef  _Codec_Server
#define  _Codec_Server

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

#include "opencv2/opencv.hpp"
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>


#include <stdio.h>                                                                                                                              
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ip.h>

using namespace std;

class Codec_Server
{
public:
    //输入图像以确定视频分辨率
    bool set_resolution(cv::Mat &input);
    //直接设置视频分辨率
    bool set_resolution(int width, int height);
    //设置传输状态，0:开始；1:结束
    void set_state(int in);
    //设置传输时长，单位：秒
    void set_timeSec(long in);
    //设置输出的模式、IP地址、端口号等
    void set_address(string &data);
    //设置帧率、数据流格式（h264）、比特率等，并初始化编码器
    void initialize_encoder(int frame_rate, AVCodecID mycodeid, int bit_rate);
    //这是传输图像的主体，每当要发送新的一张图片时，都应当调用此函数
    void Encoder_and_Send(cv::Mat image);

    Codec_Server();
    ~Codec_Server();

private:
    int state;//视频推流状态，0:开始；1:结束，默认为1
    long timeSec;//视频推流时长
    int64_t duration_timer;
    char *out_ip_address;//输出的模式、IP地址、端口号等

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
    int m_width;              //输出视频宽度  
    int m_height;             //输出视频高度  
    AVPacket pkt;            //数据包结构体
    AVFormatContext *fmtctx;//数据帧
    AVOutputFormat *ofmt;
    long long frame_index;
    int64_t calc_duration;
    int64_t pts_time;
    int64_t start_time;
    AVStream *pOutStream;

    char* cPacketData;
    uint8_t* cExtradata;

    int sps_id_add;

};

#endif