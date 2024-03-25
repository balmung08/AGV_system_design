
#include "../include/push_h264/push_h264.h"


using namespace cv;
using namespace std;



/*void ImageCallback(const sensor_msgs::Image& msg)
{
    // cv::Mat src =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    // flip(src,imageTemp,0);
    imageTemp =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    picture_updated = true;
    // cout << "original size:   " << imageTemp.cols << "x" << imageTemp.rows << endl;
}*/

/*void PushH264::init_image_size()
{
    // while(imageTemp.cols == 0)
    // {
    //     cout << "Can not get the image!" << endl;
    //     ros::spinOnce();
    //     cout << imageTemp.cols << "   " << imageTemp.rows << endl;
    //     usleep(200000);
    // }
    int x=0; // 裁剪区域起始点 x坐标
    int y=0; // 裁剪区域起始点 y坐标

    Rect m_area(x, y, m_width, m_height);
    send_image = imageTemp(m_area).clone();//不要clone
}*/

void PushH264::init_Encoder()
{
    av_register_all();
    avformat_network_init();

    
    Ffmpeg_Encoder_Init();//初始化编码器  
    Ffmpeg_Encoder_Setpara(AV_CODEC_ID_H264, m_width, m_height);//设置编码器参数

    // cout << "send_image: " << m_width << " x " << m_height << endl;

    avformat_alloc_output_context2(&fmtctx, NULL, "flv", out_ip_address); //RTMP
    //打开网络流 fmtctx->pb:创建的AVIOContext结构体 fmtctx->filename url协议地址 方式为只写
    if (!fmtctx)
    {
        printf( "Could not create output context\n");
        // ret = AVERROR_UNKNOWN;
        printf("Please exit!!!\n");
        // return;
    }

    AVCodec *video_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if(video_codec == NULL )
    {
        cout << "video_codec == NULL"<< endl;
    }

    avcodec_open2(c, video_codec, NULL);

    // AVCodecContext *ctx_video = avcodec_alloc_context3(video_codec);

    //初始化AVStream
    AVStream *video_st = avformat_new_stream(fmtctx, video_codec);
    video_st->id = 0;
    video_st->codecpar->codec_tag = 0;
    video_st->time_base = AVRational{1, 30};
    avcodec_parameters_from_context(video_st->codecpar, c);
    pOutStream = video_st;
    if (!video_st)
    {
        printf( "Failed allocating output stream\n");
        // ret = AVERROR_UNKNOWN;
        printf("Please exit!!!\n");
        // return;
    }
    video_st->id = fmtctx->nb_streams-1; //add

    int ret = avcodec_copy_context(video_st->codec, c);
    if (ret < 0)
    {
        printf( "Failed to copy context from input to output stream codec context\n");
        printf("Please exit!!!\n");
        // return;
    }
    //video_st->codec->codec_tag = 0;

    av_dump_format(fmtctx, 0, out_ip_address, 1);
    ret = avio_open(&fmtctx->pb,out_ip_address, AVIO_FLAG_WRITE);
    if (ret < 0) 
    {
        printf( "Could not open output URL '%s'", out_ip_address);
        printf("Please exit!!!\n");
        // return;
    }
    //打开输出流


    AVDictionary * opts = nullptr;
    av_dict_set(&opts, "flvflags", "no_duration_filesize", 0);
    ret = avformat_write_header(fmtctx, opts ? &opts : NULL);
    // ret = avformat_write_header(fmtctx, NULL);
    if (ret < 0) 
    {   
        // cout << ret << endl;
        printf( "Error occurred when opening output URL\n");
        printf("Please exit!!!\n");
        // return;
    }

    cPacketData = new char(c->extradata_size  * sizeof(char*));
    cExtradata = (uint8_t *)malloc((200000) * sizeof(uint8_t));
}


    //创建AVFormatContext结构体
//分配一个AVFormatContext，FFMPEG所有的操作都要通过这个AVFormatContext来进行
void PushH264::push_image()
{
    start_time = av_gettime();//av_gettime()获取系统时间

    // namedWindow("send_image",WINDOW_NORMAL);//创建窗口
    // resizeWindow("send_image", 500, 500);
    // if(kill_Child_thread)
    //     break;
    Ffmpeg_Encoder_Encode((uchar*)send_image.data);//编码
    // imshow("send_image",send_image);
    // cv::waitKey(1);
}

void PushH264::pushout_image(uchar* send_data)
{
    start_time = av_gettime();//av_gettime()获取系统时间
    Ffmpeg_Encoder_Encode(send_data);//编码
}

/*int main(int argc, char* argv[])
{
    ros::init(argc, argv, "UdpClient");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe(topic_name,10,&ImageCallback);

    init_image_size();
    Rect m_area(x, y, m_width, m_height);
    init_Encoder();

    ros::Rate loop_rate(40);//寻找话题的频率

    TermSignal::init();
    pthread_t tids;
    //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
    pthread_create(&tids, NULL, push_image, NULL);

    while(TermSignal::ok())
    {
        ros::spinOnce();
        if(picture_updated && !imageTemp.empty() && imageTemp.cols >= m_width)
        {
            send_image = imageTemp(m_area).clone();//不要clone
            picture_updated = false;
        }
        loop_rate.sleep();
    }

    kill_Child_thread = true;
    cout << "Wait for the child thread to exit!!" << endl;

    
    Ffmpeg_Encoder_Close();
    free(cExtradata);
    destroyWindow("send_image");

    cout << "Child thread exited!!" << endl;
    return 0;
}*/