#include "../include/push_h264/MyEncoder.h"

#define AV_PKG_FLAG_KEY 0x65


using namespace std;

MyEncoder::MyEncoder()
{
}

void MyEncoder::Ffmpeg_Encoder_Init()
{
    av_register_all();
    avcodec_register_all();

    m_pRGBFrame = new AVFrame[1];//RGB帧数据赋值    
    m_pYUVFrame = new AVFrame[1];//YUV帧数据赋值    

    c = NULL;//解码器指针对象赋初值  
}

void MyEncoder::Ffmpeg_Encoder_Setpara(AVCodecID mycodeid, int vwidth, int vheight)
{
    pCodecH264 = avcodec_find_encoder(mycodeid);//查找h264编码器  
    if (!pCodecH264)
    {
        fprintf(stderr, "h264 codec not found\n");
        exit(1);
    }
    width = vwidth;
    height = vheight;

    c = avcodec_alloc_context3(pCodecH264);//函数用于分配一个AVCodecContext并设置默认值，如果失败返回NULL，并可用av_free()进行释放 
    c->bit_rate = 4096000; //设置采样参数，即比特率  
    c->width = vwidth;//设置编码视频宽度   
    c->height = vheight; //设置编码视频高度  
    c->time_base.den = 30;//设置帧率,num为分子和den为分母，如果是1/25则表示25帧/s  
    c->time_base.num = 1;
    c->framerate = { 30,1 };//帧率
    c->gop_size = 5; //设置GOP大小,该值表示每10帧会插入一个I帧  
    c->max_b_frames = 1;//设置B帧最大数,该值表示在两个非B帧之间，所允许插入的B帧的最大帧数
    // c->qmin = 10;
    // c->qmax = 51;  
    c->pix_fmt = AV_PIX_FMT_YUV420P;//设置像素格式

    av_opt_set(c->priv_data, "tune", "zerolatency", 0);





    // c->codec_type = AVMEDIA_TYPE_VIDEO;



    // av_opt_set(c->priv_data, "preset", "slow", 0);
    // av_opt_set(c->priv_data, "tune","zerolatency",0);
    // av_opt_set(c->priv_data, "x264opts","crf=26:vbv-maxrate=728:vbv-bufsize=3640:keyint=25",0);

    av_opt_set(c->priv_data, "tune", "zerolatency", 0);//设置编码器的延时，解决前面的几十帧不出数据的情况  



     pkt.flags |= AV_CODEC_FLAG_GLOBAL_HEADER;


    if (avcodec_open2(c, pCodecH264, NULL) < 0)
    {
        avcodec_free_context(&c);
        return;//打开编码器  
    }



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
    // outbuf_size = 100000;//初始化编码输出数据区
    // outbuf = new uint8_t[outbuf_size];

    scxt = sws_getContext(c->width, c->height, AV_PIX_FMT_BGR24, c->width, c->height, AV_PIX_FMT_YUV420P, SWS_POINT, NULL, NULL, NULL);//初始化格式转换函数  

    calc_duration=(double)AV_TIME_BASE/c->time_base.den/1000;
    av_init_packet(&pkt);
    cout << "calc_duration: " << calc_duration << endl;
}

// void MyEncoder::Ffmpeg_Encoder_Encode(FILE *file,uint8_t *data)
void MyEncoder::Ffmpeg_Encoder_Encode(uint8_t *data)
{
    int ret;
    // av_init_packet(&pkt);

    pkt.data = NULL;

    // cout << "frame_index: " << frame_index << endl;
    // cout << "width: " << c->width << "    height: " << c->height << endl;

    memcpy(rgb_buff, data, nDataLen);//拷贝图像数据到rgb图像帧缓存中准备处理  
    avpicture_fill((AVPicture*)m_pRGBFrame, (uint8_t*)rgb_buff, AV_PIX_FMT_RGB24, width, height);//将rgb_buff填充到m_pRGBFrame  
    //av_image_fill_arrays((AVPicture*)m_pRGBFrame, (uint8_t*)rgb_buff, AV_PIX_FMT_RGB24, width, height);
    avpicture_fill((AVPicture*)m_pYUVFrame, (uint8_t*)yuv_buff, AV_PIX_FMT_YUV420P, width, height);//将yuv_buff填充到m_pYUVFrame  

    sws_scale(scxt, m_pRGBFrame->data, m_pRGBFrame->linesize, 0, c->height, m_pYUVFrame->data, m_pYUVFrame->linesize);// 将RGB转化为YUV  

    int myoutputlen = 0;
    int returnvalue = avcodec_encode_video2(c, &pkt, m_pYUVFrame, &myoutputlen);

    /*if (pkt.flags &AV_PKG_FLAG_KEY)//找到带I帧的AVPacket
    // if((pkt.data[4] & 0x1f) == 5)
    {
        cPacketData = (char*)pkt.data; 
        //找到I帧，插入SPS和PPS
        memcpy(cExtradata, c->extradata, c->extradata_size);
        memcpy(cExtradata + c->extradata_size, pkt.data, pkt.size);//&pkt.data[0]
        pkt.size += c->extradata_size;
        pkt.data = cExtradata;
    }*/

    pkt.stream_index = pOutStream->index;

    pkt.pts=(double)(frame_index*calc_duration);
    pkt.dts=pkt.pts;
    pkt.duration=(double)calc_duration;
   
    pkt.pos = -1;

    int64_t pts_time = calc_duration*1000;
    int64_t now_time = av_gettime() - start_time;
    // cout << "pts_time:="<<pts_time<<" "<<now_time << endl;
    
    // if (pts_time > now_time)
    //     av_usleep(pts_time - now_time);
    
    frame_index ++;
    // cout << "frame_index:="<<frame_index<< endl;
    start_time = av_gettime();

    if (returnvalue == 0)
    {
        // fwrite(pkt.data, 1, pkt.size, file);
        ret = av_interleaved_write_frame(fmtctx,&pkt);
    }
    /*if (ret < 0) 
    {
        printf( "Error muxing packet\n");
    }*/
    av_free_packet(&pkt);
}

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