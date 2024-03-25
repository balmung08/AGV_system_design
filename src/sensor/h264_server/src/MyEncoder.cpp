#include "MyEncoder.h"

#define AV_PKG_FLAG_KEY 0x65


using namespace std;

MyEncoder::MyEncoder()
{
    sps_id_add = 0;
}

void MyEncoder::Ffmpeg_Encoder_Init()
{
    av_register_all();
    avcodec_register_all();

    m_pRGBFrame = new AVFrame[1];//RGB帧数据赋值    
    m_pYUVFrame = new AVFrame[1];//YUV帧数据赋值    

    c = NULL;//解码器指针对象赋初值  
}

void MyEncoder::Ffmpeg_Encoder_Setpara(AVCodecID mycodeid, int vwidth, int vheight, int bit_rate)
{
    pCodecH264 = avcodec_find_encoder(mycodeid);//打开h264编码器  
    if (!pCodecH264)
    {
        fprintf(stderr, "h264 codec not found\n");
        exit(1);
    }
    width = vwidth;
    height = vheight;

    c = avcodec_alloc_context3(pCodecH264);//函数用于分配一个AVCodecContext并设置默认值，如果失败返回NULL，并可用av_free()进行释放 
    c->bit_rate = bit_rate; //设置采样参数，即比特率  
    c->width = vwidth;//设置编码视频宽度
    c->height = vheight; //设置编码视频高度  
    c->time_base.den = 30;//单帧时间（单位：秒），num为分子和den为分母，与帧率相统一
    c->time_base.num = 1;
    c->framerate = { 30,1 };//设置帧率，{ 30,1 }则表示30帧/s  
    c->gop_size = 5; //设置GOP大小,该值表示每10帧会插入一个I帧  
    c->max_b_frames = 1;//设置B帧最大数,该值表示在两个非B帧之间，所允许插入的B帧的最大帧数  
    c->pix_fmt = AV_PIX_FMT_YUV420P;//设置像素格式

    av_opt_set(c->priv_data, "tune", "zerolatency", 0);//设置编码器的延时，解决前面的几十帧不出数据的情况  

    pkt.flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    

    // if (avcodec_open2(c, pCodecH264, NULL) < 0)
    // {
    //     avcodec_free_context(&c);
    //     return;//打开编码器  
    // }

    m_pRGBFrame = av_frame_alloc();
    m_pRGBFrame->format = c->pix_fmt;
    m_pRGBFrame->width = c->width;
    m_pRGBFrame->height = c->height;

    m_pYUVFrame = av_frame_alloc();
    m_pYUVFrame->format = c->pix_fmt;
    m_pYUVFrame->width = c->width;
    m_pYUVFrame->height = c->height;

    nDataLen = vwidth*vheight * 3;//计算图像rgb数据区长度

    yuv_buff = new uint8_t[nDataLen / 2];//初始化数据区，为yuv图像帧准备填充缓存
    rgb_buff = new uint8_t[nDataLen];//初始化数据区，为rgb图像帧准备填充缓存

    scxt = sws_getContext(c->width, c->height, AV_PIX_FMT_BGR24, c->width, c->height, AV_PIX_FMT_YUV420P, SWS_POINT, NULL, NULL, NULL);//初始化格式转换函数  

    calc_duration=(double)AV_TIME_BASE/c->time_base.den/1000;//单帧的时长，转换为ffmpeg的时间格式
    av_init_packet(&pkt);//初始化数据包结构体
    // cout << "calc_duration: " << calc_duration << endl;
}

void MyEncoder::Ffmpeg_Encoder_Encode(uint8_t *data)
{
    int ret;

    // pkt.data = NULL;
    av_init_packet(&pkt);

    // cout << "frame_index: " << frame_index << endl;
    // cout << "width: " << c->width << "    height: " << c->height << endl;

    memcpy(rgb_buff, data, nDataLen);//拷贝图像数据到rgb图像帧缓存中准备处理  
    avpicture_fill((AVPicture*)m_pRGBFrame, (uint8_t*)rgb_buff, AV_PIX_FMT_RGB24, width, height);//将rgb_buff填充到m_pRGBFrame  
    avpicture_fill((AVPicture*)m_pYUVFrame, (uint8_t*)yuv_buff, AV_PIX_FMT_YUV420P, width, height);//将yuv_buff填充到m_pYUVFrame  

    sws_scale(scxt, m_pRGBFrame->data, m_pRGBFrame->linesize, 0, c->height, m_pYUVFrame->data, m_pYUVFrame->linesize);// 将RGB转化为YUV  

    int myoutputlen = 0;
    int returnvalue = avcodec_encode_video2(c, &pkt, m_pYUVFrame, &myoutputlen);//将YUV图像以编码参数c进行编码，然后数据写入pkt

    pkt.stream_index = pOutStream->index;

    pkt.pts=(double)(frame_index*calc_duration);//将pkt中此帧所属的视频时间刻写入
    pkt.dts=pkt.pts;
    pkt.duration=(double)calc_duration;//将pkt中此帧的时长写入
   
    pkt.pos = -1;

    int64_t pts_time = calc_duration*1000;
    int64_t now_time = av_gettime() - start_time;
    
    //如果编码压缩速度过快，则休眠，以保持帧率
    if (pts_time > now_time)
        av_usleep(pts_time - now_time);
    
    frame_index ++;
    start_time = av_gettime();


// 设置SPS和PPS
AVCodecParameters *codec_par = avcodec_parameters_alloc();

// 设置编码器参数
avcodec_parameters_from_context(codec_par, c);

if (codec_par->extradata_size > 0) 
{
    // pkt.datasize = codec_par->extradata_size;// + AV_INPUT_BUFFER_PADDING_SIZE;
    // pkt.size = codec_par->extradata_size;// + AV_INPUT_BUFFER_PADDING_SIZE;
    memcpy(pkt.data, codec_par->extradata, codec_par->extradata_size);
    av_packet_split_side_data(&pkt);
}

AVDictionary* options = nullptr;
    av_dict_set(&options, "sps_id", to_string(sps_id_add).c_str(), 0);
    avcodec_open2(c, pCodecH264, &options);
sps_id_add = (sps_id_add + 1) % 31;


    if (returnvalue == 0)
    {
        // fwrite(pkt.data, 1, pkt.size, file);
        ret = av_interleaved_write_frame(fmtctx,&pkt);//将数据包pkt写入数据帧，然后推流
    }
    /*if (ret < 0) 
    {
        printf( "Error muxing packet\n");
    }*/
    av_free_packet(&pkt);//清空此帧的数据
}

//释放变量的内存
void MyEncoder::Ffmpeg_Encoder_Close()
{
    delete[]m_pRGBFrame;
    delete[]m_pYUVFrame;
    delete[]rgb_buff;
    delete[]yuv_buff;
    // delete[]outbuf;   
    av_write_trailer(fmtctx);
    sws_freeContext(scxt);
    avcodec_close(c);//关闭编码器  
    av_free(c);
    avformat_free_context(fmtctx);
}