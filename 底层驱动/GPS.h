
#ifndef TEST_WSK_GPS_H
#define TEST_WSK_GPS_H

#include <termio.h>
#include "Serial.h"
#include "public.h"

#define PI 3.1415926


class TGPSData
{
public:
    double Lat_raw, Lon_raw;  //  原始值
    double Lat, Lon, Angle;    // 转换值   纬度  经度  角度
    string utm_zone_data;
    double UTM_Y, UTM_X;   //  当前坐标点
    double Pre_Y, Pre_X;   //  上一次坐标点
    TTimer tmr_speed;
    float speed;   //  移动速度  m/s
    float Hight;  //高程

    TDataFilter *df_X, *df_Y, *df_Ang;   //对数据进行均值滤波处理
    int Lat_state, Lon_state, Ang_state;
    int ReadyState;
    int Quality_factor;

    TGPSData()
    {
        Lat=Lon=Angle=0;
        Lat_raw=Lon_raw=0;
        Lat_state=Lon_state=Ang_state=0;
        Quality_factor=0;
        ReadyState=0;

        df_X=new TDataFilter(10);
        df_Y=new TDataFilter(10);
        df_Ang=new TDataFilter(10);
    };

    void Resolve(unsigned char *buf);
};


class TGPS: public Thread
{
private:
    TCOM *com;

public:
    TGPSData gdata;
    int HeartBeat;

    TGPS(int com_id);
    void run();
};


#endif //TEST_WSK_GPS_H
