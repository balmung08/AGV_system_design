#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "ros/ros.h"

#include <QMainWindow>
#include <QListWidgetItem>
#include <QApplication>

#include "IRNetDemo/irnetsdkmanager.h"
#include "IRNetDemo/playerwidget.h"


using namespace cv;
using namespace std;
using namespace IRNet;
Mat src;

#define DEVIDE_ID1   1

PlayerWidget* player_widget_1;

long m_device_id = -1;
void OnVideoShow(uchar* rgbData, int w, int h, int macIndex,int type);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "infrared_camera_node");
	QApplication a(argc, argv);
	ros::NodeHandle nh;
	ros::NodeHandle prv_nh("~");

	std::string pub_topic;
	std::string cam_ip;

	int cv_show;

	prv_nh.param<int>("display",cv_show,0);	

	image_transport::ImageTransport it(nh);
    image_transport::Publisher Imprex_image_publish = it.advertise("/infrared_image", 1);
	
	src.create(288,384,CV_8UC1);

	IRNetSDKManager::GetInstance()->Init();

	player_widget_1=new PlayerWidget();
	player_widget_1->SetDeviceId(DEVIDE_ID1);

	printf("init!\n");

	m_device_id=DEVIDE_ID1;
	QString ip ="192.168.4.202";
    int model=0;

    player_widget_1->Connect(ip,2100,model,5000);

	usleep(2000);
    player_widget_1->StartTemperatureMeasurement();
	Mat dst;
	printf("come here to show\n");
	ros::Rate loop_rate(300);	
	while(ros::ok())
	{   
		// printf("!!!！！！！max info temp:%.2f,x=%d,y=%d\n",player_widget_1->maxPoint->temp_,player_widget_1->maxPoint->x,player_widget_1->maxPoint->y);
        // printf("!!!！！！！min info temp:%.2f,x=%d,y=%d\n",player_widget_1->minPoint->temp_ ,player_widget_1->minPoint->x,player_widget_1->minPoint->y);
		// printf("come here to show!!!\n");
		cv::cvtColor(player_widget_1->srcdata,dst,cv::COLOR_RGB2BGR);
		// cv::flip(dst,dst,0);//player_widget_1->
		// // printf("src.size=%d,%d\n",player_widget_1->srcdata.rows,player_widget_1->srcdata.cols);
		// cv::rectangle(dst,cv::Point(380-50,280-50),cv::Point(380+50,280+50),cv::Scalar(255,255,255),2);
		// char buffer1[20];
		// sprintf(buffer1,"%.2f",player_widget_1->minPoint->temp_);
		// std::string str1= buffer1;
		// str1="ave: "+str1;
		// std::cout<<"buffer0: "<<str1<<std::endl;
		// cv::putText(dst, str1, cv::Point(350,220), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        // cv::circle(dst,cv::Point(player_widget_1->maxPoint->x,dst.rows-player_widget_1->maxPoint->y),2,cv::Scalar(255,255,255),2);
        // char buffer[20];
        // sprintf(buffer,"%.2f",player_widget_1->maxPoint->temp_);
        // std::string str=buffer;
		// str="max: "+str;
		// std::cout<<"buffer1: "<<str<<std::endl;
        // cv::putText(dst, str, cv::Point(player_widget_1->maxPoint->x-40,dst.rows-player_widget_1->maxPoint->y-20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
		// sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    	// image_msg->header.stamp = ros::Time::now();
    	// Imprex_image_publish.publish(image_msg);

		if (cv_show)
		{
			// printf("src.size=%d,%d\n",player_widget_1->srcdata.rows,player_widget_1->srcdata.cols);
			imshow("src",dst);			
			
		}
		waitKey(1);
		loop_rate.sleep();
	}

    return 0;
}


