#include <serial/serial.h>
#include "ros/ros.h"
#include <NavConversion.h>
#include <common/public.h>
#include <gps/MyGPS_msg.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
#include <common/pudp.h>
#include <queue>
#include <cstring>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <common/mydisplay.h>

using namespace std;

class TGPSData
{
private:
    int sock;
    string utm_zone_data;
    TTimer tmr_speed;
    int Lat_state, Lon_state, Ang_state;

public:
    double Lat_raw, Lon_raw; //  原始值
    double Lat, Lon, Alt, Angle;  // 转换值   纬度  经度  角度
    double UTM_Y, UTM_X;     //  当前坐标点
    double Pre_Y, Pre_X;     //  上一次坐标点
    double PreY, PreX;
    float speed; //  移动速度  m/s
    float Hight; //高程

    int count = 0;
    TTimer tmr;

    // TDataFilter *df_X, *df_Y, *df_Ang;   //对数据进行均值滤波处理
    int Quality_factor;
    int ReadyState;
    int Resolve(const char *buf);

    TGPSData()
    {
        Lat = Lon = Alt = Angle = 0;
        Lat_raw = Lon_raw = 0;
        Quality_factor = 0;
        PreX = PreY = 0;

        ClearState();
    };

    void ClearState()
    {
        ReadyState = 0;
        Lat_state = Lon_state = Ang_state = 0;
    }
};

class TGPS_Base
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher gps_pub, gps_raw_pub, gps_fix_pub;
    ros::Subscriber keyboard_sub, joy_sub, simpose_sub;
    serial::Serial *ser;
    PUDP *udp;
    // THeartbeat ht;

    double utmx_zero, utmy_zero; //零点位置，车启动瞬间，标记为零点位置
    gps::MyGPS_msg gps_sim_msg;
    float angle_bias=0, x_bias=0, y_bias=0;

public:
    // int HeartBeat;
    TGPSData gdata;

    bool simmode;

    TGPS_Base();

    void KeyboardCallback(const std_msgs::UInt16::ConstPtr &msg); // 处理键盘数据,专用仿真
    void JoyCallback(const sensor_msgs::Joy::ConstPtr &Joy);      //处理手柄数据
    void SimPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);   // 接收Task,专用仿真

    int DataProc(const char *buf);
    void simdata();
    void run();
};

int TGPSData::Resolve(const char *buf)
{

    if (tmr.GetValue() >= 1.0)
    {
        // printf("count=%d\n",count);
        count = 0;

        tmr.Clear();
    }
    // vector<string> ss = split(buf, "$");
    Ang_state = 0;
    vector<string> str = split(buf, ",");
    if (str[0] == "$GNGGA" || str[0] == "$GPGGA")
    {
        count++;
        if (str[2] != "")
            Lat_raw = atof(str[2].data()), Lat_state = 1;
        else
            Lat_state = 0;
        if (str[4] != "")
            Lon_raw = atof(str[4].data()), Lon_state = 1;
        else
            Lon_state = 0;
        if (str[3] == "S")
            Lat_raw = -Lat_raw;
        if (str[5] == "W")
            Lon_raw = -Lon_raw;

        if (str[6] != "")
            Quality_factor = atoi(str[6].c_str());
        else
            Quality_factor = 0;
        //*NIU 字段9为海拔高度
        if(str[9] != "") Alt = atof(str[9].data()); 
        else Alt = 0;

        LatLonConvert(Lat_raw, Lon_raw, &Lat, &Lon);
        LLtoUTM(Lat, Lon, UTM_Y, UTM_X, utm_zone_data); //  Y 北向  X 正东

        if (tmr_speed.GetValue() > 0.3)
        {
            float d = P2P(Pre_X, Pre_Y, UTM_X, UTM_Y);
            Pre_X = UTM_X, Pre_Y = UTM_Y;
            speed = d / tmr_speed.GetValue();
            tmr_speed.Clear();
        }
    }
    else if (str[0] == "$GNHDT" || str[0] == "$GPHDT")
    {
        if (str[1] != "")
        {
            Angle = atof(str[1].c_str());
            Ang_state = 1;
        }
        else
            Ang_state = 0;
    }

    // int n=4;
    // static TDataFilter f_angle(n), f_x(n), f_y(n);
    // if(fabs(Angle-f_angle.value)>60)   for(int i=0;i<n;i++)  f_angle.GetValue(Angle);
    // f_angle.GetValue(Angle);   Angle=f_angle.value;
    // f_x.GetValue(UTM_X);   UTM_X=f_x.value;
    // f_y.GetValue(UTM_Y);   UTM_Y=f_y.value;

    // printf("ABC=%s\n",buf);

    return Ang_state;
}

TGPS_Base::TGPS_Base()
{
    nh = new ros::NodeHandle;
    nh_local = new ros::NodeHandle("~"); //  局部空间
    gps_pub = nh_local->advertise<gps::MyGPS_msg>("GPS_Base", 10);
    gps_fix_pub = nh_local->advertise<sensor_msgs::NavSatFix>("GPS_fix", 1);
    gps_raw_pub = nh_local->advertise<std_msgs::String>("GPSRaw_Base", 10);
    string gpsBuf;

    keyboard_sub = nh->subscribe<std_msgs::UInt16>("/keyboard", 10, &TGPS_Base::KeyboardCallback, this);
    joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy", 10, &TGPS_Base::JoyCallback, this);
    simpose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/sim_pose", 10, &TGPS_Base::SimPoseCallback, this);

    int tmp_count = 0;
    TTimer gps_tmr;

    nh_local->getParam("angle_bias", angle_bias);
    nh_local->getParam("x_bias", x_bias);
    nh_local->getParam("y_bias", y_bias);
    
    utmx_zero = utmy_zero = 0;
    nh_local->getParam("utmx_zero", utmx_zero);
    nh_local->getParam("utmy_zero", utmy_zero);
    string m = "";
    nh_local->getParam("mode", m);
    ser = NULL;
    udp = NULL;

    nh_local->getParam("sim_flag", simmode);
    // printf("%d\n",simmode);
    if (simmode)
    {
        gps_sim_msg.header.frame_id = "map";
        gps_sim_msg.header.stamp = ros::Time::now();

        gps_sim_msg.Lat = gps_sim_msg.Lon = 0, gps_sim_msg.Angle = 0;
        gps_sim_msg.UTM_X = gps_sim_msg.UTM_Y = 0;
        
        gps_sim_msg.map_x = 0, gps_sim_msg.map_y = 0;
        gps_sim_msg.Angle= 90;
        
        gps_sim_msg.Vel = 0;
        gps_sim_msg.Quality = 4;
    }
    if (simmode)  return;

    if (m == "UDP")
    {
        string ip = "192.168.3.5";
        int port = 1234;
        nh_local->getParam("IP", ip);
        nh_local->getParam("port", port);
        udp = new PUDP(1234);
        udp->AddRemoteIP(ip, port);

        udp->Send("OK"); // must have
    }
    else if (m == "SER")
    {
        string devname = "";
        int baud = 0;
        nh_local->getParam("devname", devname);
        nh_local->getParam("baud", baud);
        ROS_INFO("%s %d", devname.c_str(), baud);
        if (devname != "" && baud > 0)
        {
            system("sudo chmod 777 /dev/ttyS*");

            ser = new serial::Serial;
            ser->setPort(devname);
            ser->setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser->setTimeout(to);
            ser->open();
        }
    }
}

int TGPS_Base::DataProc(const char *buf)
{
    int res = gdata.Resolve(buf);
    // printf("res=%d buf=%s\n",res,buf);

    std_msgs::String strmsg;
    strmsg.data = buf;
    gps_raw_pub.publish(strmsg);

    if (res) //  必须有角度
    {
        gdata.ClearState();

        bool setMapZeroFlag = false;
        nh_local->getParam("setMapZeroFlag", setMapZeroFlag);
        if (setMapZeroFlag) //  将当前位置设置为Map零点
        {
            nh_local->setParam("setMapZeroFlag", false);
            utmx_zero = gdata.UTM_X;
            utmy_zero = gdata.UTM_Y;
            nh_local->setParam("utmx_zero", gdata.UTM_X);
            nh_local->setParam("utmy_zero", gdata.UTM_Y);
        }

        // *NIU 发布gps_fix 话题(经纬高)
        sensor_msgs::NavSatFix Nav;
        Nav.header.stamp = ros::Time::now();
        Nav.header.frame_id = "map";
        Nav.latitude = gdata.Lat;
        Nav.longitude = gdata.Lon;
        Nav.altitude = gdata.Alt;
        Nav.status.status = gdata.Quality_factor; // 定位质量(为0则未定位)
        Nav.position_covariance[0] = gdata.Angle; // 得到的角度
        gps_fix_pub.publish(Nav);

        gps::MyGPS_msg msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        // msg.Lat = gdata.Lat, msg.Lon = gdata.Lon, msg.Angle = angle_bias + 180 - gdata.Angle + 90.0; 
        // msg.Angle = msg.Angle > 180.0 ? msg.Angle -= 360.0 : msg.Angle <= -180.0 ? msg.Angle += 360.0
        //                                                                          : msg.Angle;
        // msg.raw_Angle=gdata.Angle;
        // if(gdata.Angle>180)  msg.raw_Angle-=360;
        // msg.raw_Angle=-msg.raw_Angle;

        // msg.UTM_Y = gdata.UTM_Y, msg.UTM_X = gdata.UTM_X;
        // msg.UTM_Y+= x_bias*sin(msg.Angle*M_PI/180)+y_bias*cos(msg.Angle*M_PI/180);
        // msg.UTM_X+= x_bias*cos(msg.Angle*M_PI/180)-y_bias*sin(msg.Angle*M_PI/180);
        // msg.map_x = msg.UTM_X-utmx_zero;
        // msg.map_y = msg.UTM_Y-utmy_zero;

        msg.Lat = gdata.Lat, msg.Lon = gdata.Lon;
        msg.raw_UTM_Y = gdata.UTM_Y;
        msg.raw_UTM_X = gdata.UTM_X;
        msg.raw_Angle = 270-gdata.Angle - 90;
        if (msg.raw_Angle >= 180)  
            msg.raw_Angle -= 360;
        else if (msg.raw_Angle < -180)  
            msg.raw_Angle += 360;
        msg.raw_map_x = msg.raw_UTM_X - utmx_zero;
        msg.raw_map_y = msg.raw_UTM_Y - utmy_zero;

        msg.mqtt_angle=msg.raw_Angle + angle_bias-90;
        // if(msg.mqtt_angle>180)  msg.mqtt_angle-=360;
        // msg.mqtt_angle=-msg.mqtt_angle;
        // printf("%.2f\n", msg.mqtt_angle);

        msg.Angle=msg.raw_Angle + angle_bias;
        msg.map_x=msg.raw_map_x + x_bias*cos(msg.Angle*M_PI/180) - y_bias*sin(msg.Angle*M_PI/180);
        msg.map_y=msg.raw_map_y + x_bias*sin(msg.Angle*M_PI/180) + y_bias*cos(msg.Angle*M_PI/180);
        msg.UTM_X=msg.map_x + utmx_zero;
        msg.UTM_Y=msg.map_y + utmy_zero;
      
        msg.Vel = gdata.speed;
        msg.Quality = gdata.Quality_factor;

        msg.HeartBeat = 20; // ht.value;
        msg.header.frame_id = "utm";
        msg.header.stamp = ros::Time::now();

        gps_pub.publish(msg);
        //  printf("%.2f\n",msg.Vel);
    }

    return res;
}

//  GPS信号模拟
void TGPS_Base::simdata()
{
    if (!simmode)
        return;

    static float pre_mapx = 0, pre_mapy = 0;
    static TTimer tmr;

    if (tmr.GetValue() > 0.3)
    {
        float d = P2P(pre_mapx, pre_mapy, gps_sim_msg.map_x, gps_sim_msg.map_y);
        pre_mapx = gps_sim_msg.map_x, pre_mapy = gps_sim_msg.map_y;
        gps_sim_msg.Vel = d / tmr.GetValue();
        tmr.Clear();
    }

    double utmx_zero = 0, utmy_zero = 0;
    nh_local->getParam("utmx_zero", utmx_zero);
    nh_local->getParam("utmy_zero", utmy_zero);
    gps_sim_msg.UTM_X = gps_sim_msg.map_x + utmx_zero;
    gps_sim_msg.UTM_Y = gps_sim_msg.map_y + utmy_zero;
    gps_sim_msg.mqtt_angle=gps_sim_msg.Angle-90;

    string zone = "51N";
    UTMtoLL(gps_sim_msg.UTM_Y, gps_sim_msg.UTM_X, zone, gps_sim_msg.Lat, gps_sim_msg.Lon); //  Y 北向  X 正东

    gps_pub.publish(gps_sim_msg);
}

void TGPS_Base::SimPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!simmode)  return;

    gps_sim_msg.map_x=msg->pose.position.x;
    gps_sim_msg.map_y=msg->pose.position.y;
    gps_sim_msg.Angle=GetYawFromPose(*msg)*180/M_PI;
}

void TGPS_Base::KeyboardCallback(const std_msgs::UInt16::ConstPtr &msg)
{
    // printf("\nkey=%d\n", msg->data);
    static tf::TransformListener listener;

    if (!simmode)
        return;

    if (msg->data == 65 || msg->data == 66)
    {
        geometry_msgs::PointStamped src_p, dst_p;
        src_p.header.frame_id = "base_link";
        src_p.header.stamp = ros::Time::now() - ros::Duration(0.05);
        src_p.point.x = src_p.point.y = src_p.point.z = 0;
        try
        {
            if (msg->data == 65)
                src_p.point.x += 1;
            else
                src_p.point.x -= 1;
            listener.transformPoint("map", src_p, dst_p);

            gps_sim_msg.map_x = dst_p.point.x;
            gps_sim_msg.map_y = dst_p.point.y;
            // printf("AAA=%.2f %.2f\n", dst_p.point.x, dst_p.point.y);
        }
        catch (exception e)
        {
            ROS_INFO("%s", e.what());
        }
    }

    else if (msg->data == 68)
        gps_sim_msg.Angle += 2;
    else if (msg->data == 67)
        gps_sim_msg.Angle -= 2;
}

void TGPS_Base::JoyCallback(const sensor_msgs::Joy::ConstPtr &Joy)
{
    // cout << "接收手柄数据成功" << endl;
    static tf::TransformListener listener;

    if (!simmode)
        return;

    if (Joy->axes[5] != 0)
    {
        geometry_msgs::PointStamped src_p, dst_p;
        src_p.header.frame_id = "base_link_gps";
        src_p.header.stamp = ros::Time::now() - ros::Duration(0.05);
        src_p.point.x = src_p.point.y = src_p.point.z = 0;
        try
        {
            if (Joy->axes[5] == 1)
                src_p.point.x += 0.4;
            else
                src_p.point.x -= 0.4;
            listener.transformPoint("map", src_p, dst_p);

            gps_sim_msg.map_x = dst_p.point.x;
            gps_sim_msg.map_y = dst_p.point.y;
            // printf("AAA=%.2f %.2f\n", dst_p.point.x, dst_p.point.y);
        }
        catch (exception e)
        {
            ROS_INFO("%s", e.what());
        }
    }

    if (Joy->axes[4] > 0)
        gps_sim_msg.Angle += 2;
    if (Joy->axes[4] < 0)
        gps_sim_msg.Angle -= 2;
}

void TGPS_Base::run()
{
    static string gpsBuf;
    static TTimer gps_tmr;

    char msg[1024];
    int n = 0;

    simdata();

    if (udp != NULL)
    {
        if (gps_tmr.GetValue() > 1)
        {
            udp->Send("OK"); // must have
            gps_tmr.Clear();
        }

        n = udp->Recv();
        if (n > 0)
            strcpy(msg, udp->rec_buf);
    }
    else if (ser != NULL)
    {
        n = ser->available();
        n = ser->read((unsigned char *)msg, n);
        msg[n] = 0;
        // if(n) printf("%s\n", msg);
    }

    for (int i = 0; i < n; ++i)
    {
        if (msg[i] != '\n')
            gpsBuf.push_back(msg[i]);
        else
        {
            gpsBuf.push_back('\n');

            DataProc(gpsBuf.c_str());
            gpsBuf.clear();
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_base");
    ROS_INFO_STREAM("gps base start by wsk");
    TGPS_Base gps;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // printf("A\n");
        gps.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
