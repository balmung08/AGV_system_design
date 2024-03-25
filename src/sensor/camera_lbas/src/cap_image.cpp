#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "../include/camera_lbas/MvCameraControl.h"
#include <iostream>
#include <sstream>

#include <assert.h>
#include "math.h"
#include "pthread.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "ros/ros.h"

#include "../include/push_h264/push_h264.h"

using namespace cv;
using namespace std;
  //视频流推送的初始化
std::string ip_address="udp://192.168.2.5:8090";
PushH264 MyPushH264(1080,1440);//图像的尺寸，高*宽

Mat src_img;

void __stdcall ImageCallBack(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        src_img.data=pData;

        

    }
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n" , "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
    // print current ip and user defined name
        // printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);
        printf("%s %s\n\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_lbas_node");
  ros::NodeHandle prv_nh("~");
  ros::NodeHandle nh;
  std::string pub_topic;
  std::string cam_ip;
  bool if_auto,push_flag,if_resize;
  
  Size2d size = {640,512};
  int cv_show;
  double Exp_time;
  int img_height, img_width;
  prv_nh.param<std::string>("pubtopic_name", pub_topic, "lbas_image");
  prv_nh.param<int>("display",cv_show,1);
  prv_nh.param<double>("exp_time",Exp_time,50000.);;
  prv_nh.param<bool>("if_resize",if_resize,false);
  prv_nh.param<bool>("if_auto",if_auto,true);
  prv_nh.param<bool>("push_flag",push_flag,true);
  prv_nh.param<std::string>("ip",cam_ip,"192.168.1.201");
  prv_nh.param<int>("img_height",img_height,1080);
  prv_nh.param<int>("img_width",img_width,1440);
  image_transport::ImageTransport it(prv_nh);
  image_transport::Publisher Luster_image_publish = it.advertise(pub_topic, 1);

  src_img.create(img_height, img_width, CV_8UC3);

  int nRet = -1;
  void* m_handle = NULL;

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nIp1, nIp2, nIp3, nIp4;
  // enum device
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet)
  {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    return -1;
  }
  unsigned int nIndex = 0;
  bool flag = false;
  std::stringstream ss_ip;
  if (stDeviceList.nDeviceNum > 0)
  {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
      {
        printf("[device %d]:\n", i);
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
            break;
        }
        nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        ss_ip.str("");
        ss_ip << nIp1 << "." << nIp2 << "." << nIp3 << "." << nIp4;
        if (ss_ip.str()==cam_ip)
        {
            flag = true;
            nIndex = i;
            ROS_INFO("[CAM 1]: compatiable IP found: %s", ss_ip.str().c_str());
            break;
        }
        else
        {
          ROS_WARN("[CAM 1]: incompatiable IP found: %s", ss_ip.str().c_str());
        }
        PrintDeviceInfo(pDeviceInfo);
      }
  }
  else
  {
      ROS_WARN("Find No Devices!\n");
      return -1;
  }

  if (!flag)
  {
    ROS_INFO("None of Valid Camera!");
    return -1;
  }    

    //枚举子网内指定的传输协议对应的所有设备
    // unsigned int nTLayerType = MV_GIGE_DEVICE | MV_USB_DEVICE;
    // MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    // nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    // if (MV_OK != nRet)
    // {
    //     printf("error: EnumDevices fail [%x]\n", nRet);
    //     return -1;
    // }

    // int i = 0;
    // if (m_stDevList.nDeviceNum == 0)
    // {
    //     printf("no camera found!\n");
    //     return -1;
    // }

    //选择查找到的第一台在线设备，创建设备句柄
    int nDeviceIndex = 0;

    // MV_CC_DEVICE_INFO m_stDevInfo = {0};
    // memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));

    nRet = MV_CC_CreateHandle(&m_handle, stDeviceList.pDeviceInfo[nIndex]);

    if (MV_OK != nRet)
    {
        printf("error: CreateHandle fail [%x]\n", nRet);
        return -1;
    }
    MyPushH264.out_ip_address = const_cast<char*>(ip_address.c_str());
    MyPushH264.init_Encoder();
    //注册数据回调函数
     nRet = MV_CC_RegisterImageCallBackForBGR(m_handle, ImageCallBack, NULL);
    if (MV_OK != nRet)
    {
        printf("error: RegisterImageCallBack fail [%x]\n", nRet);
        return -1;
    }

    //连接设备
     nRet = MV_CC_OpenDevice(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: OpenDevice fail [%x]\n", nRet);
        return -1;
    }
    //...其他处理 

      //设置自动曝光模式
    unsigned int nValue = if_auto? MV_EXPOSURE_AUTO_MODE_CONTINUOUS:MV_EXPOSURE_AUTO_MODE_OFF; //一次曝光模式
    nRet = MV_CC_SetExposureAutoMode(m_handle, nValue);
    if (MV_OK != nRet)
    {
        ROS_WARN("error: SetExposureAutoMode fail [%x]\n", nRet);
        return -1;
    }else ROS_INFO("setting ExposureAUtoMode OK!");
    
    if(!if_auto){
        nRet = MV_CC_SetFloatValue(m_handle, "ExposureTime", Exp_time);
        if (MV_OK == nRet)
        {
            ROS_INFO("set exposure time OK!\n\n");
        }
        else
        {
            ROS_WARN("set exposure time failed! nRet [%x]\n\n", nRet);
        }
    }

    //开始采集图像
     nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: StartGrabbing fail [%x]\n", nRet);
        return -1;
    }
    
    string work_state="";
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        // if(push_flag)
        // {
        //     MyPushH264.pushout_image(src_img.data);
        // }

        // printf("src_img:%d,%d\n", src_img.cols, src_img.rows);

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_img).toImageMsg();
        image_msg->header.stamp = ros::Time::now();
        Luster_image_publish.publish(image_msg);

		string new_work_state="";
		prv_nh.getParam("/work_state", new_work_state);
		if(work_state=="start_work" && new_work_state=="work_done")
		{
			string path;
            prv_nh.getParam("/save_file_path", path);
            imwrite(path+"/ccd.jpg", src_img);  
            // ROS_INFO("ccd saved");
		}
        work_state=new_work_state;

        if(cv_show)
        {
            imshow("src", src_img);
            waitKey(1);
        }

        loop_rate.sleep();
    }
    


    //停止采集图像 
     nRet = MV_CC_StopGrabbing(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: StopGrabbing fail [%x]\n", nRet);
        return -1;
    }

    //关闭设备，释放资源
     nRet = MV_CC_CloseDevice(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: CloseDevice fail [%x]\n", nRet);
        return -1;
    }

    //销毁句柄，释放资源
     nRet = MV_CC_DestroyHandle(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: DestroyHandle fail [%x]\n", nRet);
        return -1;
    }    
}
