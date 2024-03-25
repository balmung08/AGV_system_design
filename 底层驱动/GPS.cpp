

#include <termios.h>
#include <math.h>
#include "GPS.h"
#include "NavConversion.h"

using namespace std;


void LatLonConvert(double lat_raw, double lon_raw, double *lat, double *lon)
{
    //经纬度的格式为ddmm.mmmm格式，如1122.3333表示11度22.3333分，转换为度为单位，整数部分除100取整为度的整数部分，减去分乘以100为分

    int intlongitude = ((int) lon_raw) / 100;  //将GPS经纬度ddmm形式统一转换为度的形式
    *lon = (lon_raw - intlongitude * 100.0) / 60 + double(intlongitude);

    int intlatitude = ((int) lat_raw) / 100;
    *lat = (lat_raw - intlatitude * 100.0) / 60 + double(intlatitude);
}

int LatLon2XY(double lat, double lon, double *y, double *x)
{
    double oval_Lonradius= 6378137;//椭球体长半轴,米,oval_Lonradius,WGS84
    double oval_Shrradius= 6356752.3142;//椭球体短半轴,米,oval_Shrradius,WGS84
    double StdLat=PI*30.0 / 180.0, StdLng=0;

    double latitude=lat*PI/180.0;
    double longitude=lon*PI/180.0;

    //Mercator投影正解过程
    double f, e, e_, NB0, K, dtemp;
    double E = exp(1);
    //确保经纬度在合理范围之内
    if (longitude<-PI || longitude>PI || latitude<-PI / 2 || latitude>PI / 2)
    {
        return 1;
    }
    //确保椭球体参数在合理范围之内
    if (oval_Lonradius <= 0 || oval_Shrradius <= 0)
    {
        return 1;
    }
    //计算扁率
    f = (oval_Lonradius - oval_Shrradius) / oval_Lonradius;
    //第一偏心率e
    dtemp = 1 - (oval_Shrradius / oval_Lonradius)*(oval_Shrradius / oval_Lonradius);
    if (dtemp<0)
    {
        return 1;
    }
    e = sqrt(dtemp);
    //第二偏心率
    dtemp = (oval_Lonradius / oval_Shrradius)*(oval_Lonradius / oval_Shrradius) - 1;
    if (dtemp<0)
    {
        return 1;
    }
    e_ = sqrt(dtemp);
    NB0 = ((oval_Lonradius*oval_Lonradius) / oval_Shrradius) / sqrt(1 + e_*e_*cos(StdLat)*cos(StdLat));
    K = NB0*cos(StdLat);
    *x = K*(longitude - StdLng);
    *y = K*log(tan(PI / 4 + latitude / 2)*pow((1 - e*sin(latitude)) / (1 + e*sin(latitude)), e / 2));

    return 0;
}

double GetDistance(double x1,double y1,double x2,double y2)
{
    double r=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
    if(r>0.01) r=sqrt(r);
    else r=0;

    return r;
}

double P2P(double x1,double y1,double x2,double y2)
{
    double r=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
    if(r>0.01) r=sqrt(r);
    else r=0;

    return r;
}

void TGPSData::Resolve(unsigned char *buf)
{
    //printf("%s\n",buf);
    vector<string> ss = split((char *)buf, "$");
    for (int i = 0; i < ss.size(); i++)
    {
        vector<string> ssa = split(ss[i], ",");
        for (int j = 0; j < ssa.size(); j++)
            if ((ssa[j] == "GNGGA"||ssa[j] == "GPGGA") && ssa.size() >= 10)   //
            {
                if (ssa[j + 2] != "") Lat_raw = atof(ssa[j + 2].data()), Lat_state = 1;
                else Lat_state = 0;
                if (ssa[j + 4] != "") Lon_raw = atof(ssa[j + 4].data()), Lon_state = 1;
                else Lon_state = 0;
                if (ssa[j + 3] == "S") Lat_raw = -Lat_raw;
                if (ssa[j + 5] == "W") Lon_raw = -Lon_raw;

                if (ssa[j + 6] != "") Quality_factor = atoi(ssa[j + 6].c_str());
                else Quality_factor = 0;

                if (ssa[j + 9] != "")  Hight = atof(ssa[j + 9].c_str());
                else Hight = 0;

                //printf("%.2f\n", Hight);

                LatLonConvert(Lat_raw,Lon_raw, &Lat, &Lon);
                LLtoUTM(Lat,Lon, UTM_Y, UTM_X, utm_zone_data);

                //printf("%.5f %.5f\n",Latitude,Longitude);
                df_X->GetValue(UTM_X);
                df_Y->GetValue(UTM_Y);
                //printf("%.5f %.5f\n",df_X->value,df_Y->value);

                if(tmr_speed.GetValue()>0.2)  //
                {
                    float d=P2P(Pre_X, Pre_Y, df_X->value, df_Y->value);
                    Pre_X=df_X->value, Pre_Y=df_Y->value;
                    speed=d/tmr_speed.GetValue();
                    //printf("ds=%.2f dt=%.3f speed=%.2f\n",d,tmr_speed.GetValue(), speed);
                    tmr_speed.Clear();
                }

                ReadyState = 1;

                break;
            }
            else if ((ssa[j] == "GNHDT" || ssa[j] == "GPHDT") && ssa.size() >= 3)
            {
                //printf("%s %d\n",ss[i].c_str(),ssa[j+1].length());
                if (ssa[j+1]!="")
                {
                    Angle = atof(ssa[j + 1].c_str());
                    Angle=180-Angle;   //  计算北向角度
                    Ang_state = 1;
                }
                else Ang_state = 0;

                df_Ang->GetValue(Angle);

                if(ReadyState==1) ReadyState=2;   //  差分解
                //printf("r=%d\n",ReadyState);
                break;
            }
    }
}

TGPS::TGPS(int com_id)
{
    char buf[100];
    sprintf(buf,"/dev/ttyS%d",com_id);
    com=new TCOM(buf);
    com->SetSpeed(115200);

    start();  // 启动线程
}


void TGPS::run()
{
    int tmpcount=0;
    TTimer tmr_hb;


    while(1)
    {
        usleep(1000);

        if(tmr_hb.GetValue()>=1)
        {
            tmr_hb.Clear();

            HeartBeat=tmpcount/tmr_hb.GetValue();
            tmpcount=0;
        }

        if (com->RecCount > 50)
        {
            gdata.Resolve(com->RecBuf);
            if (gdata.ReadyState==2)
            {
                gdata.ReadyState=0;
                HeartBeat++;

                printf("$GPS %d %.4f %.4f %.2f %.2f %.2f %.3f\n", gdata.Quality_factor,gdata.Lat, gdata.Lon, gdata.df_X->value, gdata.df_Y->value, gdata.Angle, gdata.speed);
//                if(HeartBeat%2==0 && udp!=NULL)
//                {
//                    char buf[500];
//                    sprintf(buf,"$GPS B %d %.4f %.4f %.2f %.2f %.2f %.3f\n", gdata.Quality_factor,gdata.Lat, gdata.Lon,
//                                                                      gdata.df_X->value, gdata.df_Y->value, gdata.Angle, gdata.speed);
//                    udp->AddSendStr(buf);
//                    //printf("%s", buf);
//                }

            }

            com->RecCount = 0;
        }
    }
}