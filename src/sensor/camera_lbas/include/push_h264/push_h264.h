#include "../include/push_h264/MyEncoder.h"


#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>                                                                                                                              
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ip.h> /* superset of previous */

// #include <pthread.h>
// #include "TermSig.h"
// #include <chrono>
// #include <thread>
// #include <unistd.h>

class PushH264:public MyEncoder
{
private:
    // cv::Mat imageTemp;
    cv::Mat send_image;

    
    int m_width; // 裁剪区域宽度
    int m_height; // 裁剪区域高度

    // MyEncoder myencoder;

    // const char *out_ip_address = "udp://192.168.8.1:8359";
    // char *out_ip_address;
    // const char *topic_name = "/lbas_image";

    // bool kill_Child_thread = false;
    // bool picture_updated = false;


public:

    char *out_ip_address;
    PushH264(int height,int width)
    {
        m_width = width;
        m_height = height;
        // out_ip_address = const_cast<char*>(ip_address.c_str());
    }
    void GetImage(cv::Mat input)
    {
        send_image = input;
    }
    
    // void init_image_size();
    void init_Encoder();
    void push_image();
    void pushout_image(uchar* send_data);
};