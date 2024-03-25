#include <cstring>
#include <cstdlib>
#include "Axis.h"
#include "BITCar.h"
#include "Gpio.h"
#include "GPS.h"
#include "signal.h"

using namespace std;

TUDP *udp;
TCan *can[4];
TAxis *axis[8];
TBIT_Car *car;
TDigit digit;
// TBMS *bms;
TRemoteCtr *remoteCtr;
//TIMU imu;
// TGPS gps(2);
int exit_flag = 0;

//void signal_handler(int signo)
//{
//    if (signo == SIGINT)
//    {
//        printf("recv SIGINT\n");
//        exit_flag = 1;
//        delete car;
//        car = nullptr;
//        for (int i = 0; i < 8; i++)
//        {
//            axis[i]->Enable(0);
//            usleep(10000);
//        }
//        printf("Motors disabled.\n");
//    }
//}

int main()
{

    // udp = new TUDP(8080,6000,"192.168.3.192"); // 本程序的端口号默认为8080
    udp = new TUDP;
//    udp->AddRemoteIP("192.168.3.115", 8090);
    udp->AddRemoteIP("192.168.1.115", 8090);
    // udp->AddRemoteIP("192.168.3.190",8091);
    // bms=new TBMS(can[0]);
    remoteCtr = new TRemoteCtr(0); // 2

    for (int i = 0; i < 4; i++)
        can[i] = new TCan(i, 500000);
    for (int i = 0; i < 8; i++)
    {
        axis[i] = new TAxis(i, i % 2, can[i / 2]);
        usleep(20000);
        axis[i]->ClrFlt(); //clear fault
        usleep(20000);
        axis[i]->Enable(0);
        axis[i]->udp = udp;
    }

    car = new TBIT_Car;

    car->bms = NULL; // bms;
    car->remoteCtr = remoteCtr;

    sleep(1);
    car->udp = udp;


    car->turn_motor[0] = axis[2];
    car->turn_motor[1] = axis[0];
    car->turn_motor[2] = axis[6];
    car->turn_motor[3] = axis[4];

    // 车轮  2 3 6 7
    car->wheel_motor[0] = axis[3];
    car->wheel_motor[1] = axis[1];
    car->wheel_motor[2] = axis[7];
    car->wheel_motor[3] = axis[5];

    car->Init();

//    signal(SIGINT, signal_handler);
//    axis[1]->Enable(0);  //右前轮子,turn 左正右负
//    axis[3]->Enable(0);  //左前轮子,turn 左正右负
//    axis[5]->Enable(0);  //右后轮子,turn 左正右负
//    axis[7]->Enable(0);  //左后轮子,turn 左正右负


    TTimer tmr;
    while (1)
    {
//        if (exit_flag == 1)
//            break;

        //角度测试
//        usleep(20000);
//       //if can_id = 1  turn;
//        axis[0]->Enable(1);  //右前轮子,turn 左正右负
//        axis[0]->Moving(0.0);
//
//
//        usleep(20000);
//        //if can_id = 1  turn;
//        axis[2]->Enable(1);  //左前轮子,turn 左正右负
//        axis[2]->Moving(0.0);
//
//
//        usleep(20000);
//        //if can_id = 1  turn;
//        axis[4]->Enable(1);  //右后轮子,turn 左正右负
//        axis[4]->Moving(0.0);
//
//        usleep(20000);
//        //if can_id = 1  turn;
//        axis[6]->Enable(1);  //左后轮子,turn 左正右负
//        axis[6]->Moving(0.0);
////
        //速度测试//
//        usleep(20000);
               //if can_id = 1  turn;
//        usleep(20000);
//
//        axis[1]->SetCtrMode(2);
//        axis[1]->Enable(1);  //右前轮子,turn 左正右负
//        axis[1]->Moving(2.5 * 10);
////
//
//        usleep(20000);
//
//        axis[3]->SetCtrMode(2);
//        axis[3]->Enable(1);  //右前轮子,turn 左正右负
//        axis[3]->Moving(2.5 * 10);
//
//        usleep(20000);
//
//        axis[5]->SetCtrMode(2);
//        axis[5]->Enable(1);  //右前轮子,turn 左正右负
//        axis[5]->Moving(2.5 * 10);
//
//        usleep(20000);
//
//        axis[7]->SetCtrMode(2);
//        axis[7]->Enable(1);  //右前轮子,turn 左正右负
//        axis[7]->Moving(2.5 * 10);


//        remoteCtr->

        //        printf("%.2f %.2f %.2f  %.2f %.2f %.2f %.2f %.2f %.2f \n",imu.angle[0],imu.angle[1],imu.angle[2],
        //                                                    imu.anglevel[0],imu.anglevel[1],imu.anglevel[2],
        //                                                    imu.linearAcc[0],imu.linearAcc[1],imu.linearAcc[2]);
//        while (tmr.GetValue() >= 10)
    }

    return 0;
}
