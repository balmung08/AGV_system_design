
#pragma once

#include <math.h>
#include <time.h>
#include <unistd.h>
#include "ros/ros.h"
#include <algorithm>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "radar/ARS_Ladar.h"
#include "radar/const_vars.h"
#include "config/radar_config_200.h"
#include "config/radar_config_202.h"
#include "radar/configuration_vars.h"
#include "config/motion_input_speed_300.h"
#include "config/object_list_status_60a.h"
#include "config/object_general_info_60b.h"
#include "config/object_quality_info_60c.h"
#include "config/object_extended_info_60d.h"
/*#include "config/motion_input_speed_300.h"*/
/*#include "config/motion_input_yawrate_301.h"*/
#include <sstream>
#include <common/public.h>
#include <common/can.h>

#define PI 3.1415926
#define MAX_SIZE 108


using namespace std;

typedef struct 
{
    bool isExit;
    int id;//目标的ID号
    float m_Angle;//目标与纵向的夹角
    float m_Range;//目标距离（直线距离）
    float m_Rate;
   
    int obj_DistLong_rms,obj_DistLat_rms;    

    float old_pre_Distx,old_pre_Disty;
    float pre_Distx,pre_Disty;
    float m_Distx, m_Disty;//物体的横向位置和纵向位置

    float m_Velx, m_Vely;//横向速度和纵向速度
    int m_DynProp;//目标的动态属性，只有给载体的正确速度时这个数值才会正确

    float RCS;//雷达散射截面（用于判断物体大小的一种衡量方法）
    char object_Class;//物体属性
    float object_orientationangle;//目标方位角度
    float length;//目标长度
    float width;//目标宽度
    int exitpro;

    bool operator ==(const int &index)
    {
        return (this->id==index);
    }

} TObsInfo;



class ARS_Ladar: public Thread
{
private:
    ros::NodeHandle *pn;
    TCan *can;
    TNodeCheck *nodecheck;

    float speed;//车辆速度
    string frame_id;
    
/*
*毫米波雷达数据处理公共部分
*/
    MotionInputSpeed300 motioninputspeed300;//输入Radar载体速度
    ObjectListStatus60A objectListStatus60A;//60A/61A数据处理
    ObjectGeneralInfo60B objectGeneralInfo60B;//60B/61B数据处理
    ObjectQualityInfo60C objectQualityInfo60C;//60C/61C数据处理
    ObjectExtendedInfo60D objectExtendedInfo60D;//60D/61D数据处理
    
    bool Check(TObsInfo p);
    void send_configdata_200();//ARS408配置信息
    void send_configdata_202();//目标配置信息
    void ProcessRadarData(const can_frame can_data);   //根据ESR的CAN协议进行解析
    
/*
*前向毫米波雷达数据
*/
    int ROI_longth;//待检测区域的前方距离
    int ROI_width;//待检测区域的左右距离
    float buffer_time=1;//缓冲时间
    int NUMBER;//当前Object数量
    float NoObsTime;//ROI内无Object的持续时长
    bool isRadarObsExit;//
    TObsInfo objects[MAX_SIZE];
    
    struct timeval now_time,obstacle_time;
    ros::Publisher Radarpoints_pub;
    void PubRadarObject();
    

public:
    void Init();
    void speedsend();//发送速度信息
    ros::Publisher radar_pub, radarstate_pub;
    
    float distance;  
    TTimer radartimer,radartimer1;
    bool radarstate,backradarstate;
    ARS_Ladar();   //  初始化
    void run();          //  线程函数
};

