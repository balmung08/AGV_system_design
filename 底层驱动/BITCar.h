#include "public.h"
#include "Axis.h"
#include "Serial.h"
#include "Gpio.h"
#include <math.h>
#include <cmath>

#include "QuadProg++.hh"

extern TDigit digit;

class TBMS : public Thread
{
public:
    double SOC = 0.0;             // 电池电量剩余%
    int BatChgSta = 0;            // 电池组充电状态
    int CellMaxTempera = 0;       // 单体最高温度
    int CellMinTempera = 0;       // 单体最低温度
    double BatSumVolt = 0.0;      // 电池累加电压
    double BatTotalCurrent = 0.0; // 电池组电流
    TCan *can;
    int heartbeat = 0;

    TBMS(TCan *c);
    void run();
    void send_cmd();
};

class TBIT_Car : public Thread
{
private:
    int StoppingFlag = 0;
    float StoppingTime = 0;
    int CtrMode = 0, PreCtrMode = 0;
    int turnmode = 0;
    int wheelCtrMode = 0; // 新增轮速控制模式选择，0：速度模式；1：力矩模式
    TTimer light_tmr, print_tmr;
    int ad_fd = 0;

    const double Length = 0.63;           // 前后轴距
    const double Width = 0.515;             // 宽度
    const double dwm = 0.09;               // 转向轴到车轮平面距离
    const double dw = 0.25;               // 车轮半径
    const double turn_acceleration = 1000; // 转向加速度限制，单位为度/二次秒,暂时没有用到
    const double turn_limspeed = 45;       // 转向角度限制，单位为度/秒

    double angle_mode4; // 模式4原地旋转时的车轮角度

//    float k = 0.11 * dwm;                                                                     // 与转向电机的减速比有关，待精确整定
    float leftfront_off = 0.0, rightfront_off = 0.0, leftrear_off = 0.0, rightrear_off = 0.0; // 转向电机编码器的零偏

public:
    TAxis *wheel_motor[4], *turn_motor[4];
    float wheel_ratio[4] = {0}, turn_ratio[4] = {0};
    TKalmanFilter filer, kalman[4];
    TUDP *udp;
    TBMS *bms;
    TRemoteCtr *remoteCtr;
    int Active = 0; //
    float max_angle = 0, max_speed = 0;
    double angle = 0, ang_L = 0, ang_R = 0, giv_angle[4] = {0, 0, 0, 0}, giv_speed[4] = {0, 0, 0, 0};
    double Radius = 999.0;
    float giv_spd = 0, giv_spd_L = 0, giv_spd_R = 0;

    int dir = 1;                                                                         // 转弯方向，左转为正右转为负
    double ang_F = 0, ang_B = 0, dtc = 0, dtcth = 0;
    double Rc = 1e10, R1 = 1e10, R2 = 1e10, R3 = 1e10, R4 = 1e10; // 中心和各个车轮的转弯半径
    int stick_offset = 1;                                         // 左摇杆零偏补偿量


    TBIT_Car();
    ~TBIT_Car();
   
    void Enable(int v);
    void WheelEnable(int v);
    void TurnEnable(int v);
    void Init();
    void SpdCtr();
    void AngleCtr();
    void RemoteOp();
    void LightCtr();
    void UDP_Proc();
    float GetRemoteBatteryVoltage();
    void GetRadiusAng();
    void run();
    void SpdAngleCalculate();
    TTimer carCtrlTmr; // 控制周期定时器
    TTimer tmr_demo;
    TTimer tmr_tempt;
    TTimer tmr_protect;
    limspeed_vessel *sideAngleLimspd_RC = nullptr, *angleLimspd_RC = nullptr;
    limspeed_vessel *angleLimspd = nullptr, *sideAngleLimspd = nullptr, *givSpdLim = nullptr;
};
