#include "codec_server.h"



Codec_Server::Codec_Server()
{
    state = 1;
    timeSec = 0;
    duration_timer = 0;
    out_ip_address = NULL;

    m_pRGBFrame = NULL;
    m_pYUVFrame = NULL;
    pCodecH264 = NULL;
    c = NULL;
    yuv_buff = NULL;
    rgb_buff = NULL;
    scxt = NULL;
    outbuf = NULL;
    outbuf_size = 0;
    nDataLen = 0;
    m_width = 0;
    m_height = 0;
    av_free_packet(&pkt);
    fmtctx = NULL;
    ofmt = NULL;
    frame_index = 0;
    calc_duration = 0;
    pts_time = 0;
    start_time = 0;
    pOutStream = NULL;

    cPacketData = NULL;
    cExtradata = NULL;
    sps_id_add = 0;
}

Codec_Server::~Codec_Server()
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
    free(cExtradata);
}

bool Codec_Server::set_resolution(cv::Mat &input)
{
    if(input.empty())
        return false;
    m_width = input.cols - input.cols % 2;
    m_height = input.rows - input.rows % 2;
    return true;
}

bool Codec_Server::set_resolution(int width, int height)
{
    m_width = width - width % 2;
    m_height = height - height % 2;
}

void Codec_Server::set_state(int in)
{
    if(state = 1 && in == 0)
        duration_timer = av_gettime();
    state = in;
}

void Codec_Server::set_timeSec(long in)
{
    timeSec = in;
    duration_timer = av_gettime();
}

void Codec_Server::set_address(string &data)
{
    out_ip_address = const_cast<char*>(data.c_str());
}

void Codec_Server::initialize_encoder(int frame_rate, AVCodecID mycodeid, int bit_rate)
{
    if(m_width == 0 || m_height == 0 || out_ip_address == NULL)
    {
        cout << "Error Message: " << "Invalid image size!" << endl << endl; 
    }
    
    av_register_all();//注册一个ffmpeg空间
    avformat_network_init();//打开网络传输通道
    avcodec_register_all();

    m_pRGBFrame = new AVFrame[1];//RGB帧数据赋值    
    m_pYUVFrame = new AVFrame[1];//YUV帧数据赋值    


    //打开编码器
    pCodecH264 = avcodec_find_encoder(mycodeid);

    // if (!pCodecH264)
    // {
    //     fprintf(stderr, "mycodeid codec not found\n");
    //     exit(1);
    // }

    c = avcodec_alloc_context3(pCodecH264);//函数用于分配一个AVCodecContext并设置默认值，如果失败返回NULL，并可用av_free()进行释放 
    c->bit_rate = bit_rate; //设置采样参数，即比特率  
    c->width = m_width;//设置编码视频宽度
    c->height = m_height; //设置编码视频高度  
    c->time_base.den = frame_rate;//单帧时间（单位：秒），num为分子和den为分母，与帧率相统一
    c->time_base.num = 1;
    c->framerate = { frame_rate,1 };//设置帧率，{ 30,1 }则表示30帧/s  
    c->gop_size = 10; //设置GOP大小,该值表示每gop_size帧会插入一个I帧  
    c->max_b_frames = 2;//设置B帧最大数,该值表示在两个非B帧之间，所允许插入的B帧的最大帧数  
    c->pix_fmt = AV_PIX_FMT_YUV420P;//设置像素格式

    av_opt_set(c->priv_data, "tune", "zerolatency", 0);//设置编码器的延时，解决前面的几十帧不出数据的情况
    pkt.flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    m_pRGBFrame = av_frame_alloc();
    m_pRGBFrame->format = c->pix_fmt;
    m_pRGBFrame->width = c->width;
    m_pRGBFrame->height = c->height;

    m_pYUVFrame = av_frame_alloc();
    m_pYUVFrame->format = c->pix_fmt;
    m_pYUVFrame->width = c->width;
    m_pYUVFrame->height = c->height;

    nDataLen = m_width * m_height * 3;//计算图像rgb数据区长度

    yuv_buff = new uint8_t[nDataLen / 2];//初始化数据区，为yuv图像帧准备填充缓存
    rgb_buff = new uint8_t[nDataLen];//初始化数据区，为rgb图像帧准备填充缓存

    scxt = sws_getContext(c->width, c->height, AV_PIX_FMT_BGR24, c->width, c->height, AV_PIX_FMT_YUV420P, SWS_POINT, NULL, NULL, NULL);//初始化格式转换函数  

    calc_duration = (double)AV_TIME_BASE/c->time_base.den/1000;//单帧的时长，转换为ffmpeg的时间格式
    pts_time = calc_duration * 1000;
    av_init_packet(&pkt);//初始化数据包结构体

    avformat_alloc_output_context2(&fmtctx, NULL, "flv", out_ip_address);
    AVCodec *video_codec = avcodec_find_encoder(AV_CODEC_ID_H264);//读取h264编码
    if(video_codec == NULL )
    {
        cout << "video_codec == NULL"<< endl;
    }

    avcodec_open2(c, video_codec, NULL);//打开编码的参数设置

    //初始化AVStream（数据帧）
    //然后以此数据帧格式（video_st），打开一个数据流通道，此后往数据帧参数（fmtctx）执行av_interleaved_write_frame函数写入数据后，即可将图像传输出去
    AVStream *video_st = avformat_new_stream(fmtctx, video_codec);
    video_st->id = 0;
    video_st->codecpar->codec_tag = 0;
    video_st->time_base = AVRational{1, 30};
    avcodec_parameters_from_context(video_st->codecpar, c);
    pOutStream = video_st;
    if (!video_st)
    {
        printf( "Failed allocating output stream\n");
        printf("Please exit!!!\n");
        // return;
    }
    video_st->id = fmtctx->nb_streams-1; //add

    int ret = avcodec_copy_context(video_st->codec, c);//将编码参数复制给video_st
    if (ret < 0)
    {
        printf( "Failed to copy context from input to output stream codec context\n");
        printf("Please exit!!!\n");
        // return;
    }

    av_dump_format(fmtctx, 0, out_ip_address, 1);//将发送目标的IP地址写入数据帧参数（fmtctx）
    ret = avio_open(&fmtctx->pb,out_ip_address, AVIO_FLAG_WRITE);//打开数据流
    if (ret < 0) 
    {
        printf( "Could not open output URL '%s'", out_ip_address);
        printf("Please exit!!!\n");
        // return;
    }

    AVDictionary * opts = nullptr;
    av_dict_set(&opts, "flvflags", "no_duration_filesize", 0);//设置相关参数
    ret = avformat_write_header(fmtctx, opts ? &opts : NULL);
    // ret = avformat_write_header(myencoder.fmtctx, NULL);
    if (ret < 0) 
    {
        printf( "Error occurred when opening output URL\n");
        printf("Please exit!!!\n");
        // return;
    }

    //申请内存，设置数据包内存大小，以将数据写入内存进行处理
    cPacketData = new char(c->extradata_size  * sizeof(char*));
    cExtradata = (uint8_t *)malloc((2000000) * sizeof(uint8_t));
}

void Codec_Server::Encoder_and_Send(cv::Mat image)
{
    /*if(state == 1)
    {
        int64_t now_time = av_gettime() - start_time;//微秒
        //如果编码压缩速度过快，则休眠，以保持帧率
        if (pts_time > now_time)
        av_usleep(pts_time - now_time);

        return;
    }

    if((double)(av_gettime() - duration_timer) / 1000000 > timeSec)
    {
        state = 1;
        return;
    }*/
    
    cv::Rect m_area(0, 0, m_width, m_height);//定义图像裁切区域
    cv::Mat send_image = image(m_area).clone();//裁切原图像并复制

    uint8_t *data = (uchar*)send_image.data;

    av_init_packet(&pkt);

    memcpy(rgb_buff, data, nDataLen);//拷贝图像数据到rgb图像帧缓存中准备处理  
    avpicture_fill((AVPicture*)m_pRGBFrame, (uint8_t*)rgb_buff, AV_PIX_FMT_RGB24, m_width, m_height);//将rgb_buff填充到m_pRGBFrame  
    avpicture_fill((AVPicture*)m_pYUVFrame, (uint8_t*)yuv_buff, AV_PIX_FMT_YUV420P, m_width, m_height);//将yuv_buff填充到m_pYUVFrame  

    sws_scale(scxt, m_pRGBFrame->data, m_pRGBFrame->linesize, 0, c->height, m_pYUVFrame->data, m_pYUVFrame->linesize);// 将RGB转化为YUV  

    int myoutputlen = 0;
    int returnvalue = avcodec_encode_video2(c, &pkt, m_pYUVFrame, &myoutputlen);//将YUV图像以编码参数c进行编码，然后数据写入pkt

    pkt.stream_index = pOutStream->index;

    pkt.pts=(double)(frame_index * calc_duration);//将pkt中此帧所属的视频时间刻写入
    pkt.dts=pkt.pts;
    pkt.duration=(double)calc_duration;//将pkt中此帧的时长写入
   
    pkt.pos = -1;

    int64_t now_time = av_gettime() - start_time;//微秒
    
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
        av_interleaved_write_frame(fmtctx,&pkt);//将数据包pkt写入数据帧，然后推流
    }
    /*if (ret < 0) 
    {
        printf( "Error muxing packet\n");
    }*/
    av_free_packet(&pkt);//清空此帧的数据
}