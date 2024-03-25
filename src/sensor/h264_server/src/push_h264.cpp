#include "codec_server.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pthread.h>
#include <chrono>
#include <thread>
#include <unistd.h>

using namespace cv;
using namespace std;

ros::NodeHandle *pn;

Codec_Server server;

cv::Mat imageTemp;
int push_flag, show_flag;

//topic回调函数
void ImageCallback(const sensor_msgs::Image& msg)
{
    imageTemp =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;//ros图片格式转为opencv图片格式
}

void *push_image(void *arg)
{
    namedWindow("send_image",cv::WINDOW_NORMAL);//创建窗口
    cv::resizeWindow("send_image", 500, 500);

    while(1)
    {
        if(show_flag == 1)
        {
            imshow("send_image",imageTemp);
            cv::waitKey(1);
        }
        // else
        // {
        //     destroyWindow("send_image");
        // }

        if(push_flag == 1)
            server.Encoder_and_Send(imageTemp);//编码，推流
    }
}

void image_receive()
{
    pthread_t tid = 1;
    pthread_create(&tid, NULL, push_image, NULL);
    
    ros::Rate loop_rate(40);
    while(ros::ok())
    {
        pn->getParam("push_flag",push_flag);
        pn->getParam("show_flag",show_flag);

        ros::spinOnce();
        loop_rate.sleep();
    }
    pthread_cancel(tid);
    cout << "Encoding child thread exited!" << endl;
}

void func()
{
    sleep(2);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Server");

    pn=new ros::NodeHandle("~");
    string ip_address;
    int bit_rate, frame_rate, timeSec;
    string topic_name;
    //读取launch中的参数
    pn->param<string>("ip_address", ip_address, "");
    pn->getParam("bit_rate", bit_rate);
    pn->getParam("frame_rate", frame_rate);
    pn->param<string>("topic_name", topic_name, "");

    server.set_address(ip_address);////////////////////////
    server.set_timeSec(timeSec);////////////////////
    server.set_state(0);////////////////////////

    ros::Subscriber image_sub = pn->subscribe(topic_name,10,&ImageCallback);//订阅话题

    while(ros::ok())
    {
        ros::spinOnce();//订阅一次话题，获取原图像的尺寸
        if(server.set_resolution(imageTemp))
            break;
        usleep(200000);
    }
    
    server.initialize_encoder(frame_rate, AV_CODEC_ID_H264, bit_rate);///////////////////

    thread t2(image_receive);
    t2.join();
    sleep(0.5);

    destroyWindow("send_image");//关闭本地显示的视频窗口
    cout<<"exit main"<<endl;
    return 0;
}