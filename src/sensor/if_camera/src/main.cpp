#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fstream>
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
#include "std_msgs/Float64MultiArray.h"          
#include "std_msgs/String.h" 

using namespace cv;
using namespace std;
using namespace IRNet;
// Mat src;

#define DEVIDE_ID1   1

// PlayerWidget* player_widget_1;

long m_device_id = -1;
// void OnVideoShow(uchar* rgbData, int w, int h, int macIndex,int type);

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
	prv_nh.param<int>("rect_Xcenter",rect_Xcenter,350);
	prv_nh.param<int>("rect_Ycenter",rect_Ycenter,180);
	prv_nh.param<int>("rect_bias",rect_bias,80);
	prv_nh.param<int>("rect_Xcenter2",rect_Xcenter2,100);  
	prv_nh.param<int>("rect_Ycenter2",rect_Ycenter2,300);
	prv_nh.param<int>("rect_bias2",rect_bias2,60);

	image_transport::ImageTransport it(nh);
    image_transport::Publisher Imprex_image_publish = it.advertise("/infrared_image", 1);
	ros::Publisher temp_pub = nh.advertise<std_msgs::Float64MultiArray>("/temp_data", 1000); 	
	
	// src.create(288,384,CV_8UC1);

	IRNetSDKManager::GetInstance()->Init();

	PlayerWidget* player_widget_1=new PlayerWidget();
	player_widget_1->SetDeviceId(DEVIDE_ID1);
	printf("init!\n");
	m_device_id=DEVIDE_ID1;
	QString ip ="192.168.4.202";
    int model=0;


    player_widget_1->Connect(ip,2100,model,5000);

	usleep(2000);
    player_widget_1->StartTemperatureMeasurement();
	Mat infrared_image;

	char aveTempBuffer[20], aveTempBuffer2[20];
	std::string aveTempStr, aveTempStr2;
	char maxTempBuffer[20], maxTempBuffer2[20];
	std::string maxTempStr, maxTempStr2;

	string work_state = "";

	ros::Rate loop_rate(300);	
	while(ros::ok())
	{   
		string new_work_state="";
		prv_nh.getParam("/work_state", new_work_state);
		if(work_state=="start_work" && new_work_state=="work_done")
		{
			string path;
            prv_nh.getParam("/save_file_path", path);
            
            std::ofstream file;
	        file.open(path+"/if_temperature.txt");
	        if(file)
	 	       file<<ros::Time::now()<<","<<aveTempStr<<","<<maxTempStr<<","<<aveTempStr2<<","<<maxTempStr2<<std::endl;
	        else
		       std::cout<<"Can't open the file!"<<std::endl;

			imwrite(path+"/if.jpg", infrared_image);   
			// ROS_INFO("path=%s if saved", (path+"/if_temperature.txt").c_str());
		}
        work_state=new_work_state;

		// printf("max info temp:%.2f,x=%d,y=%d\n",player_widget_1->maxPoint->temp_,player_widget_1->maxPoint->x,player_widget_1->maxPoint->y);
        // printf("ave info ,player_widget_1->minPoint->x,player_widget_1->minPoint->ytemp:%.2f,x=%d,y=%d\n",player_widget_1->minPoint->temp_ );
		cv::cvtColor(player_widget_1->srcdata,infrared_image,cv::COLOR_RGB2BGR);

		cv::flip(infrared_image,infrared_image,0);  //翻转图像

		cv::circle(infrared_image,cv::Point(player_widget_1->maxPoint->x,infrared_image.rows-player_widget_1->maxPoint->y),2,cv::Scalar(255,255,255),2);
		cv::circle(infrared_image,cv::Point(player_widget_1->maxPoint2->x,infrared_image.rows-player_widget_1->maxPoint2->y),2,cv::Scalar(255,255,255),2);
		cv::flip(infrared_image,infrared_image,1);  //翻转图像
		int mat_rec_x = 768-rect_Xcenter;
		int mat_rec_y = 576-rect_Ycenter;
		int mat_rec_x2 = 768-rect_Xcenter2;
		int mat_rec_y2 = 576-rect_Ycenter2;
		cv::rectangle(infrared_image,cv::Point(mat_rec_x-rect_bias,mat_rec_y-rect_bias),cv::Point(768-rect_Xcenter+rect_bias+5, 576-rect_Ycenter+rect_bias+5),cv::Scalar(255,255,255),2);
		cv::rectangle(infrared_image,cv::Point(mat_rec_x2-rect_bias2,mat_rec_y2-rect_bias2),cv::Point(768-rect_Xcenter2+rect_bias2+5, 576-rect_Ycenter2+rect_bias2+5),cv::Scalar(255,255,255),2);
	
		if(work_state == "temp_get"){
			std::cout<<"temp 1 2:"<<player_widget_1->avgPoint<<" "<<player_widget_1->avgPoint2
								<<" "<<player_widget_1->maxPoint->temp_<<" "<<player_widget_1->maxPoint2->temp_<<std::endl;
			
			std::vector<double> temp_vector;
			temp_vector.push_back(player_widget_1->avgPoint);
			temp_vector.push_back(player_widget_1->avgPoint2);
			temp_vector.push_back(player_widget_1->maxPoint->temp_);
			temp_vector.push_back(player_widget_1->maxPoint2->temp_);		
			std_msgs::Float64MultiArray temp_msg;
			temp_msg.data = temp_vector;
			temp_pub.publish(temp_msg);
			prv_nh.setParam("/work_state", "work_processing");
		}
		
		sprintf(aveTempBuffer,"%.2f",player_widget_1->avgPoint);
		sprintf(aveTempBuffer2,"%.2f",player_widget_1->avgPoint2);
		aveTempStr= aveTempBuffer;
		aveTempStr2= aveTempBuffer2;
		aveTempStr="ave: "+aveTempStr;
		aveTempStr2="ave2: "+aveTempStr2;
		cv::putText(infrared_image, aveTempStr, cv::Point(mat_rec_x-30,mat_rec_y-rect_bias-10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        cv::putText(infrared_image, aveTempStr2, cv::Point(mat_rec_x2-30,mat_rec_y2-rect_bias2-10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
        sprintf(maxTempBuffer,"%.2f",player_widget_1->maxPoint->temp_);
		sprintf(maxTempBuffer2,"%.2f",player_widget_1->maxPoint2->temp_);
        maxTempStr=maxTempBuffer;
		maxTempStr2=maxTempBuffer2;
		maxTempStr  ="max: " + maxTempStr;
		maxTempStr2 ="max2: " + maxTempStr2;
        cv::putText(infrared_image, maxTempStr, cv::Point(mat_rec_x-30,mat_rec_y-rect_bias+20), 
												cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);
		cv::putText(infrared_image, maxTempStr2, cv::Point(mat_rec_x2-30,mat_rec_y2-rect_bias2+20), 
												cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3);


		 sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", infrared_image).toImageMsg();
    	 image_msg->header.stamp = ros::Time::now();
    	 Imprex_image_publish.publish(image_msg);
		if (cv_show)
		{
			// printf("src.size=%d,%d\n",player_widget_1->srcdata.rows,player_widget_1->srcdata.cols);
			imshow("src",infrared_image);			
		}
		waitKey(1);
		loop_rate.sleep();
	}

    return 0;
}


