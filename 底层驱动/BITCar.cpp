#include "iostream"
#include "BITCar.h"

using namespace std;

float TBIT_Car::GetRemoteBatteryVoltage()
{
    int value, ch = 0;
    float res;

    lseek(ad_fd, ch, SEEK_SET);
    read(ad_fd, &value, 2);
    res = value / 4096.0 * 5;
    res /= 0.193;

    return res;
}

// pid_t pit;

TBIT_Car::TBIT_Car()
{

    udp = NULL;
    bms = NULL;
    remoteCtr = NULL;

    ad_fd = open("/dev/tlc2543", O_RDWR);

    giv_spd = 0;
    giv_angle[0] = giv_angle[1] = 0;
    Active = 0;
    StoppingFlag = 0;

    wheel_ratio[0] = wheel_ratio[2] = 1.0 / 8;
    wheel_ratio[1] = wheel_ratio[3] = -1.0 / 8;
    for (int i = 0; i < 4; i++)
        turn_ratio[i] = 1;
    max_angle = 35;
    max_speed = 400;

//    leftfront_off = 4.1; //???这是??
//    rightfront_off = 10.58;
//    leftrear_off = 7.49;
//    rightrear_off = 95.32;


    angle_mode4 = atan2(Length, Width) * 180 / M_PI;

    angleLimspd_RC = new limspeed_vessel(turn_limspeed);
    sideAngleLimspd_RC = new limspeed_vessel(turn_limspeed);
    angleLimspd = new limspeed_vessel(turn_limspeed);
    sideAngleLimspd = new limspeed_vessel(turn_limspeed);
}

TBIT_Car::~TBIT_Car()
{
    Enable(0);
    delete remoteCtr;
    delete angleLimspd_RC;
    delete sideAngleLimspd_RC;
    delete angleLimspd;
    delete sideAngleLimspd;
}

void TBIT_Car::WheelEnable(int v)
{
    for (int i = 0; i < 4; i++)
        if (wheel_motor[i]->active != v)
        {
            wheel_motor[i]->ClrFlt();
            wheel_motor[i]->SetCtrMode(SPEED_CONTROL);
            wheel_motor[i]->Enable(v);
        }
}

void TBIT_Car::TurnEnable(int v)
{
    for (int i = 0; i < 4; i++)
        if (turn_motor[i]->active != v)
        {
            turn_motor[i]->ClrFlt();
            turn_motor[i]->SetCtrMode(POS_CONTROL);
            turn_motor[i]->Enable(v);
        }
}

void TBIT_Car::Enable(int v)
{
    WheelEnable(v);
    TurnEnable(v);
}

void TBIT_Car::Init()
{
    Enable(0);
    start();
}



void TBIT_Car::UDP_Proc()
{
    if (udp == NULL)
        return;
    // char buf[10][1000], sbuf[2000];
    // sprintf(buf[0], "Car %d %d", Active, CtrMode);
    // sprintf(buf[1], "RC %d %d %dS", remoteCtr->heartbeat, remoteCtr->CH[0], remoteCtr->CH[1]);
    // // printf("%s\n",buf[1]);
    // // sprintf(buf[2],"BMS %.0f %.0f %.1f %d %d %.1f",bms->SOC, bms->BatSumVolt, bms->BatTotalCurrent, bms->CellMaxTempera, bms->CellMinTempera, GetRemoteBatteryVoltage());

    // for (int i = 0; i < 4; i++)
    //     sprintf(buf[2 + i], "Wheel%d %d %.1f %.0f",
    //             i, wheel_motor[i]->active, wheel_motor[i]->speed * wheel_ratio[i], wheel_motor[i]->drv_info.bv);

    // for (int i = 0; i < 2; i++)
    //     sprintf(buf[6 + i], "Turn%d %d %.1f %.0f",
    //             i, turn_motor[i]->active, turn_motor[i]->pos, turn_motor[i]->drv_info.bv);

    // sprintf(buf[8], "angle %.2f %.2f %.2f", imu->attiAng[0], imu->attiAng[1], imu->attiAng[2]);

    // sprintf(buf[9], "WheelIa %.4f %.4f %.4f %.4f",
    //         wheel_motor[0]->ia, -wheel_motor[1]->ia, wheel_motor[2]->ia, -wheel_motor[3]->ia);

    // sprintf(sbuf, "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s",
    //         buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
    // udp->Send(sbuf);
    // printf("%s\n",sbuf);

    // if (udp->rec_count == 0 || CtrMode != 1)
    // {
    //     // printf("UDP not received\n");
    //     return;
    // }

    // if (udp->rec_count == 0)
    //     return;

    // vector<string> substr = udp->recv_strs;
    // udp->rec_count = 0;
    if (udp->rec_flag == 0)
        return;

    vector<string> substr = split(udp->rec_output, " ");
    udp->rec_flag = 0;

    if (substr.size() < 2 || substr[0] != "Car")
    {
        // cout << "substr error."<<endl;
        return;
    }
    // cout << "UDP received: ";
    // print_vecstr(substr);

    if (CtrMode != 1)
        return;

    // Car Enable/Disable
    if (substr[1] == "Enable")
        Enable(1);
    else if (substr[1] == "Disable")
        Enable(0);

    else if (substr[1] == "Run" && substr.size() >= 4)
    { // 对称式四轮转向 Car Run speed angle
        float v = atof(substr.at(2).c_str());
        //cout<<"v"<<v<<endl;
        float a = atof(substr.at(3).c_str());
        giv_spd = v;
        // angle = -a;
        angle = angleLimspd->getValue(-a);
        if(giv_spd >=2.0) giv_spd=2.0;
        else if(giv_spd <=-2.0)  giv_spd=-2.0;
        // printf("giv_spd=%.2f\tangle=%.2f\t\n", giv_spd, angle);
    }
    else if (substr[1] == "Stop" && substr.size() >= 3)
    { // Car Stop 0/1    0表示急停
        if (substr.at(2) == "0")
            giv_spd = 0;
        else
            StoppingFlag = 1;
    }
}

//  CH0 speed  CH1 turn-angle  CH2 speed rate  CH3 MotorEnable  CH4 RemoteEnable CH5 TurnMode
void TBIT_Car::RemoteOp()
{
    if (remoteCtr->heartbeat < 5)
        return;

    //    remoteCtr->SCH[3]=bms->BatSumVolt;      // 电池电压
    //    remoteCtr->SCH[6]=bms->SOC;             // 电池SOC
    //    remoteCtr->SCH[5]=turn_motor[0]->active<<+turn_motor[1]->active<<4;     // EnableState
    //    remoteCtr->SCH[5]=wheel_motor[0]->active*8+wheel_motor[1]->active*4+wheel_motor[2]->active*2+wheel_motor[0]->active;
    //    remoteCtr->SCH[5]+=turn_motor[0]->active*16+turn_motor[1]->active*32;

    // 速度控制/力矩控制
//    if (remoteCtr->CH[8] <= 0)
//        wheelCtrMode = 0;
//    else if (remoteCtr->CH[8] > 90)
//        wheelCtrMode = 1;

    // printf("%d\n",remoteCtr->CH[4]);
    //   遥控/程控
    //   断使能/转向使能/车轮使能
    if (remoteCtr->CH[3] < -90)
        Enable(0);
    else if (remoteCtr->CH[3] == 0)
    {
        TurnEnable(1);
        WheelEnable(0);
    }
    else if (remoteCtr->CH[3] > 90)
    {
        TurnEnable(1);
        WheelEnable(1);
    }

//    for (int mm=0;mm<15;mm++)
//        std::cout<<mm<<":"<<remoteCtr->CH[mm]<<std::endl;

    if (remoteCtr->CH[4] > 90)
    {
        CtrMode = 1; //  上位机程序控制
    }
    else
    {
        CtrMode = 0;
    } // 遥控器控制

    if(CtrMode != PreCtrMode) {
        giv_spd = 0.0;
        PreCtrMode = CtrMode;
    }



    if (remoteCtr->CH[5] < -90 && remoteCtr->CH[9] <= 0)
        turnmode = 0;
    else if (remoteCtr->CH[5] == 0)
        turnmode = 1;
//    else if (remoteCtr->CH[5] > 90)
//        turnmode = 2;
//    else if (remoteCtr->CH[5] < -90 && remoteCtr->CH[9] > 90)
//        turnmode = 3; // 2022.10.26新增转向模式3，CH[9]对应开关为SG（遥控器右上）
    //std::cout<<"remoteCtr->CH[13]"<<remoteCtr->CH[13]<<std::endl;

    if (remoteCtr->CH[11] > 90)
        turnmode = 4; // 原地旋转模式
                      //    printf("turnmode = %d\n",turnmode);
    // printf("CtrMode = %d\n",CtrMode);
    if (CtrMode)
        return;

    //  速度档位   速度控制
    float speed_factor = 1;
    if (remoteCtr->CH[2] < -90)
        speed_factor = 0.1; // 低速
    else if (remoteCtr->CH[2] == 0)
        speed_factor = 0.2;
    else
        speed_factor = 0.3; // 高速
    // giv_spd = remoteCtr->CH[0] * 0.01 * max_speed * speed_factor;
    int vd = 0;
    if (remoteCtr->CH[0] > 0)
        vd = 1;
    else if (remoteCtr->CH[0] < 0)
        vd = -1;
    giv_spd = (vd * remoteCtr->CH[0] * 0.01 * remoteCtr->CH[0] * 0.01 * max_speed * speed_factor) * 1.5; // 改变遥控器操作手感
    giv_spd = giv_spd / 60 * dw * M_PI;                                                                //    printf("%.1f\n", speed_factor);
//     printf("remote_givspd = %.3f\n", giv_spd);

    angle = angleLimspd_RC->getValue(-remoteCtr->CH[1] * 0.01 * max_angle);
//         printf("remote_angle = %.3f\n", angle);


//    side_angle = sideAngleLimspd_RC->getValue(-(remoteCtr->CH[6] + stick_offset) * 0.01 * max_side_angle);
}

void TBIT_Car::GetRadiusAng()
{
    if (fabs(angle) > 0.00001 && turnmode == 0)
    {
        float temp_tan = (2 * tan(angle * M_PI / 180));
        if(temp_tan < 0.0001 && temp_tan > 0.0) temp_tan = 0.0001;
        if(temp_tan > -0.0001 && temp_tan < 0.0) temp_tan = -0.0001;

        Radius = Length / temp_tan;

        R1 = R3 = sqrt((2 * Radius - Width) * (2 * Radius - Width) + Length * Length) / 2;
        R2 = R4 = sqrt((2 * Radius + Width) * (2 * Radius + Width) + Length * Length) / 2;

        R1 += R1 > R2 ? -dwm : dwm;
        R3 += R3 > R4 ? -dwm : dwm;

        R2 += R1 > R2 ? dwm : -dwm;
        R4 += R3 > R4 ? dwm : -dwm;

        ang_L = 180 * atan2(Length , (2 * Radius - Width)) / M_PI;
        ang_R = 180 * atan2(Length , (2 * Radius + Width)) / M_PI;
        if(ang_L > 90.0) ang_L-=180.0;
        else if(ang_L < -90.0) ang_L+=180.0;

        if(ang_R > 90.0) ang_R-=180.0;
        else if(ang_R < -90.0) ang_R+=180.0;
//        printf("angle =%.2f  ang_L=%.2f, ang_R=%.2f\n",angle,ang_L,ang_R);
    }
    else
    {
        Radius = 999.0;
        R1 = R2 = R3 = R4 = 999.0;
        ang_L = 0.0;
        ang_R = 0.0;
    }
}

void TBIT_Car::SpdAngleCalculate()
{
    if (wheelCtrMode == 0)
        giv_spd = giv_spd * 60 / dw / M_PI;
    if (StoppingFlag)
    {
        if (fabs(giv_spd) < 0.1)
            giv_spd = 0, StoppingFlag = 0;
        // else
        // {
        //     if (giv_spd > 0)
        //         giv_spd -= 3;
        //     else
        //         giv_spd += 3;
        // }
    }

    if (remoteCtr->heartbeat < 5 && CtrMode == 0)
        giv_spd = 0; // 4月6日改

    if (giv_spd > max_speed)
        giv_spd = max_speed;
    else if (giv_spd < -max_speed)
        giv_spd = -max_speed;

    if (remoteCtr->heartbeat < 5)
        for (int i = 0; i < 4; i++)
            giv_angle[i] = 0;

    //angle calculate

    if (turnmode == 0)
    {
        GetRadiusAng();
        giv_angle[0] = ang_L;
        giv_angle[1] = ang_R;
        giv_angle[2] = -ang_L;
        giv_angle[3] = -ang_R;
    }
    else if (turnmode == 1)
    {
        giv_angle[0] = angle;
        giv_angle[1] = angle;
        giv_angle[2] = angle;
        giv_angle[3] = angle;
    }else if (turnmode == 4) {
        //std::cout<<"angle_mode4="<<angle_mode4<<std::endl;

        giv_angle[0] = -angle_mode4;
        giv_angle[1] = angle_mode4;
        giv_angle[2] = -angle_mode4;
        giv_angle[3] = angle_mode4;
    }

    //spd calculate

    if (turnmode == 0)
    {

        giv_spd_L = giv_spd * R1 / fabs(Radius);
        giv_spd_R = giv_spd * R2 / fabs(Radius);

        giv_speed[0] = (giv_spd_L - 0 * turn_motor[0]->speed) / wheel_ratio[0];
        giv_speed[1] = (giv_spd_R + 0 * turn_motor[1]->speed) / wheel_ratio[1];
        giv_speed[2] = (giv_spd_L - 0 * turn_motor[2]->speed) / wheel_ratio[2];
        giv_speed[3] = (giv_spd_R + 0 * turn_motor[3]->speed) / wheel_ratio[3];
//        printf("%.2f, %.2f, %.2f, %.2f\n",giv_spd_L / wheel_ratio[0], giv_spd_L / wheel_ratio[1], giv_spd_L / wheel_ratio[2], giv_spd_L / wheel_ratio[3]);
    }
    else if (turnmode == 1)
    {
        for (int i = 0; i < 4; i++)
            giv_speed[i] = giv_spd / wheel_ratio[i];
    }else if (turnmode == 4) {
        giv_speed[0] = (giv_spd / wheel_ratio[0]);
        giv_speed[1] = (-giv_spd / wheel_ratio[1]);
        giv_speed[2] = (giv_spd / wheel_ratio[2]);
        giv_speed[3] = (-giv_spd / wheel_ratio[3]);
    }

}

void TBIT_Car::AngleCtr()
{

//        printf("R1=%.2f, R2=%.2f, R3=%.2f, R4=%.2f\n",R1,R2,R3,R4);
//        printf("Rc=%.2f, dtc=%.2f, ang_F=%.2f, ang_B=%.2f, giv_angle: %.2f, %.2f, %.2f, %.2f\n",dir*Rc,dtc,ang_F,ang_B,giv_angle[0],giv_angle[1],giv_angle[2],giv_angle[3]);

    for (int i = 0; i < 4; i++)
    {
        if(giv_angle[i]>= 30) giv_angle[i] = 30.0;
        else if(giv_angle[i]<= -30) giv_angle[i] = -30.0;
        giv_angle[i]/=3.0;//这个3.0
    }
    turn_motor[0]->Moving(giv_angle[0] + leftfront_off);
    turn_motor[1]->Moving(giv_angle[1] + rightfront_off);
    turn_motor[2]->Moving(giv_angle[2] + leftrear_off);
    turn_motor[3]->Moving(giv_angle[3] + rightrear_off);
}


void TBIT_Car::SpdCtr()
{
//    printf("send_speed: %.2f, %.2f, %.2f, %.2f, %.3f\n",giv_speed[0],giv_speed[1],giv_speed[2],giv_speed[3],giv_spd);

    for (int i = 0; i < 4; i++){
        //cout<<giv_speed[i]<<endl;
                wheel_motor[i]->Moving(giv_speed[i]);
    }
}


void TBIT_Car::LightCtr()
{
    if (light_tmr.GetValue() < 0.5)
        return;
    light_tmr.Clear();

    if (Active < 6 || remoteCtr->heartbeat < 10) //  报警闪动
    {
        digit.v_out[1] = !digit.v_out[1];
        digit.v_out[3] = !digit.v_out[3];
        // if(remoteCtr->heartbeat<10) printf("%d %d\n",Active, remoteCtr->heartbeat);
    }
    else
    {
        if (giv_angle[0] > 1)
            digit.v_out[1] = !digit.v_out[1], digit.v_out[3] = 1;
        else if (giv_angle[0] < -1)
            digit.v_out[3] = !digit.v_out[3], digit.v_out[1] = 1;
        else
            digit.v_out[1] = digit.v_out[3] = 1;
    }
}



double sat(double delta, double x) // 饱和函数
{
    if (x > delta)
        return 1.0;
    else if (x < -delta)
        return -1.0;
    else
        return x / delta;
}



void TBIT_Car::run()
{
    while (1)
    {
        // usleep(20000); //  20ms控制周期
        if (carCtrlTmr.GetValue() < 0.02)
            continue;
        carCtrlTmr.Clear();
        UDP_Proc();
        Active = 0;
        for (int i = 0; i < 4; i++)
        {
            if (wheel_motor[i]->active)
                Active++;
        }
        for (int i = 0; i < 4; i++)
        {
            if (turn_motor[i]->active)
                Active++;
        }
        LightCtr();
        RemoteOp();

        SpdAngleCalculate();
        AngleCtr();
        SpdCtr();

    }
}

TBMS::TBMS(TCan *c)
{
    can = c;
    send_cmd();
    start();
}

/*
 * 函数功能：给电池发送两组指令，才会收到电池的反馈信息
 * 指令信息如下：
 *  01 00 00 00 00 00 00 00
 * 21 02 00 31 C2 01 01 00*/
void TBMS::send_cmd()
{
    can->send_frame.can_id = 0x98a0f3f3;
    can->send_frame.can_dlc = 0x08;

    can->send_frame.data[0] = 0x01;
    for (int i = 1; i < 8; ++i)
        can->send_frame.data[i] = 0x00;
    can->Send();

    usleep(1000);

    can->send_frame.data[0] = 0x21;
    can->send_frame.data[1] = 0x02;
    can->send_frame.data[2] = 0x00;
    can->send_frame.data[3] = 0x31;
    can->send_frame.data[4] = 0xc2;
    can->send_frame.data[5] = 0x01;
    can->send_frame.data[6] = 0x01;
    can->send_frame.data[7] = 0x00;
    can->Send();
}

void TBMS::run()
{
    int tmp_count = 0;
    TTimer tmr;

    while (1)
    {
        usleep(50000);
        if (tmr.GetValue() > 1)
        {
            heartbeat = tmp_count / tmr.GetValue();
            tmp_count = 0, tmr.Clear();
            // printf("%d\n",heartbeat);
        }

        if (!can->rec_flag)
            continue;
        can->rec_flag = false;
        //        printf("%x\n",can->rec_frame.can_id);

        if (can->rec_frame.can_id == 0x98A1F3F3)
        {
            can->rec_frame.can_id = 0;
            tmp_count++;
            // printf("%x\n",can->rec_frame.can_id);
            unsigned char first_byte = can->rec_frame.data[0];
            switch (first_byte)
            {
            case 0x00:
                BatChgSta = can->rec_frame.data[2] / 64; // 电池组充电状态
                break;
            case 0x05:
                SOC = can->rec_frame.data[7] * 0.4; // 电池电量剩余%
                break;
            case 0x07:
                BatSumVolt = (256 * can->rec_frame.data[3] + can->rec_frame.data[4]) * 0.1;              // 电池累加电压
                BatTotalCurrent = (256 * can->rec_frame.data[5] + can->rec_frame.data[6] - 32000) * 0.1; // 电池组电流
                break;
            case 0x0B:
                CellMaxTempera = can->rec_frame.data[5] - 40; // 单体最高温度
                CellMinTempera = can->rec_frame.data[6] - 40; // 单体最低温度
            default:
                break;
            }
        }
        //        cout<<BatChgSta<<" "<<SOC<<" "<<BatSumVolt<<" "<<BatTotalCurrent;
        //        printf("\n");
        //        cout<<" "<<CellMaxTempera<<" "<<CellMinTempera<<endl;
    }
}
