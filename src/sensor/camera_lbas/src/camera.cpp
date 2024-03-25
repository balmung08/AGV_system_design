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
#define MAX_IMAGE_DATA_SIZE   (20*1024*1024)
#define NIL (0)


//IP:192.168.1.201
//IP:192.168.1.202

using namespace cv;

// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
  int c;
  while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
  sleep(1);
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
  // ros::NodeHandle nh;
  std::string pub_topic;
  std::string cam_ip;
  bool if_auto;
  bool if_resize;
  Size2d size = {640,512};
  double max_freq;
  int cv_show;
  double Exp_time;
  prv_nh.param<std::string>("pubtopic_name",pub_topic,"lbas_image");
  prv_nh.param<int>("display",cv_show,1);
  prv_nh.param<double>("exp_time",Exp_time,50000.);
  prv_nh.param<bool>("if_resize",if_resize,false);
  prv_nh.param<bool>("if_auto",if_auto,true);
  prv_nh.param<std::string>("ip",cam_ip,"192.168.1.201");
  prv_nh.param<double>("max_hz",max_freq,30.0);
  image_transport::ImageTransport it(prv_nh);
  image_transport::Publisher Imprex_image_publish = it.advertise(pub_topic, 1);

  int nRet = MV_OK;

  void* handle = NULL;

  unsigned char *pDataForRGB = NULL;
  unsigned char* pData = NULL;   // define the pointer to store the data
  unsigned char *pDataForSaveImage = NULL;

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

  // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        ROS_WARN("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return -1;
    }


  // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        ROS_WARN("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return -1;
    }

  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
  if (nRet != MV_OK)
  {
    ROS_WARN("set TrigerMode failed");
    // break;
  }
  

  //设置自动曝光模式
  unsigned int nValue = if_auto? MV_EXPOSURE_AUTO_MODE_CONTINUOUS:MV_EXPOSURE_AUTO_MODE_OFF; //一次曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, nValue);
  if (MV_OK != nRet)
  {
      ROS_WARN("error: SetExposureAutoMode fail [%x]\n", nRet);
      return -1;
  }else ROS_INFO("setting ExposureAUtoMode OK!");
  
  if(!if_auto){
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", Exp_time);
    if (MV_OK == nRet)
    {
        ROS_INFO("set exposure time OK!\n\n");
    }
    else
    {
        ROS_WARN("set exposure time failed! nRet [%x]\n\n", nRet);
    }
  }

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  if (nRet != MV_OK)
  {
    ROS_WARN("set PayLoadSize failed");
    // break;
  }


  // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }
  double last_t = ros::Time::now().toSec();

  //视频流推送的初始化
  // std::string ip_address="udp://192.168.2.5:8090";
  // // PushH264 MyPushH264(1024,1280);//图像的尺寸，高*宽
  // PushH264 MyPushH264(1024,1280);//图像的尺寸，高*宽
  // MyPushH264.out_ip_address = "udp://192.168.4.5:8090";//const_cast<char*>(ip_address.c_str());
  // MyPushH264.init_Encoder();
  
  while (ros::ok())
  {
    
    if(max_freq>0){
      if(ros::Time::now().toSec()-last_t<1.0/max_freq)continue;
      else last_t = ros::Time::now().toSec();
    }
    last_t = ros::Time::now().toSec();
    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);

    unsigned int nDataSize = stParam.nCurValue;
    // ros::Time time_start = ros::Time::now();
    nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);

    // image processing
    // src.create(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC1);

    if (nRet == MV_OK)
    {
      pDataForSaveImage = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);



      // if (NULL == pDataForSaveImage)
      // {
      //     break;
      // }
      // 填充存图参数
      // fill in the parameters of save image
      MV_SAVE_IMAGE_PARAM_EX stSaveParam;
      memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
      // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
      // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
      // Top to bottom are：
      stSaveParam.enImageType = MV_Image_Bmp;
      stSaveParam.enPixelType = stImageInfo.enPixelType;
      stSaveParam.nBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
      stSaveParam.nWidth = stImageInfo.nWidth;
      stSaveParam.nHeight = stImageInfo.nHeight;
      stSaveParam.pData = pData;
      stSaveParam.nDataLen = stImageInfo.nFrameLen;
      stSaveParam.pImageBuffer = pDataForSaveImage;
      stSaveParam.nJpgQuality = 80;
      nRet = MV_CC_SaveImageEx(&stSaveParam);


     ROS_INFO_ONCE("nWidth=%d  nHeight=%d\n",stImageInfo.nWidth,stImageInfo.nHeight);
      ///opt/LBAS/Samples_LinuxSDK/lib/64

      // printf("stImageInfo.nHeight=%d  stImageInfo.nWidth=%d\n",stImageInfo.nHeight,stImageInfo.nWidth);
      //Mat src = Mat::zeros(1280, 1024, CV_8UC3);
      
      Mat src = Mat::zeros(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3);
      Mat src_join = Mat::zeros(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3);
      src.data = (uchar *)pDataForSaveImage;
      Mat src_join_1 = src(cv::Rect(0, 0, 18, stImageInfo.nHeight));
      Mat src_join_2 = src(cv::Rect(18, 0, stImageInfo.nWidth-18, stImageInfo.nHeight));
      hconcat(src_join_2, src_join_1, src_join);
      flip(src_join,src_join,0);
      if(if_resize){
        resize(src_join,src_join,size);
      }

      // printf("size=%d %d\n",src_join.rows,src_join.cols);
      //视频流推送
      // MyPushH264.GetImage(src_join);
      // MyPushH264.push_image();
      // printf("time=%f\n",ros::Time::now().toSec()-last_t);




      //src_join(cv::Rect(18, 0, 1262, 1024)) = src(cv::Rect(0, 0, 18, 1024));
      sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_join).toImageMsg();
      image_msg->header.stamp = ros::Time::now();
      Imprex_image_publish.publish(image_msg);

     //printf("src.size=%d  %d\n",src_join.rows,src_join.cols);

      //printf("src.rows=%d  src.cols=%d\n",src.rows,src.cols);
      //flip(src, src, 0);
      if(cv_show){
        imshow("src1", src_join);
        waitKey(1);
      }

      free(pDataForSaveImage);
      pDataForSaveImage = NULL;

    }
    free(pData);
    pData = NULL;
    waitKey(10);
  }
  // after ros breaks down, close the device manually as follows:
  // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        ROS_WARN("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }else ROS_INFO("MV_CC_StopGrabbing succeeded!");


  // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        ROS_WARN("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        return -1;
    }else ROS_INFO("MV_CC_CloseDevice succeeded!");

  // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        ROS_WARN("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return -1;
    }else ROS_INFO("MV_CC_DestroyHandle succeeded!");
    ROS_INFO("Exit camera normally!");
    return 0;
}
