#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include "motordrv.h"


TMotorDrv::TMotorDrv(uint clt_id,TCan *c,int mode)
{
    can=c;          // clt_id 是驱动器的ID识别号
    can->send_frame.can_dlc=8;
    client_id=clt_id;
    ClrFlt();
    RunMode(mode);  // 设置模式
    Monitor(true);  //  true;

    //printf("AB\n");

    //start();
}

void TMotorDrv::Enable(bool v)
{
    can->send_frame.data[0]='3';
    can->send_frame.data[1]='2';
    can->send_frame.data[4]=v;
    can->send_frame.can_id=0x401+client_id;
    can->Send();
    usleep(1000);
}

int TMotorDrv::RunMode(uint run_mode)
{
    if (drv_info.enable) return 0;

    if (run_mode<4)
    {
        can->send_frame.data[0] = '3';
        can->send_frame.data[1] = '0';
        can->send_frame.data[4] = run_mode;
        can->send_frame.can_id=0x401+client_id;
        can->Send();
    }
    return 1;
}

void TMotorDrv::Monitor(bool v)
{
    can->send_frame.data[0]='8';
    can->send_frame.data[1]='0';
    can->send_frame.data[4]=v;
    can->send_frame.can_id=0x401+client_id;
    can->Send();
    usleep(1000);
}

void TMotorDrv::Unlock(bool v)
{
    can->send_frame.data[0]='7';
    can->send_frame.data[1]='0';
    can->send_frame.data[4]=v;
    can->send_frame.can_id=0x401+client_id;
    can->Send();
    usleep(1000);
}

int TMotorDrv::SetZero()
{
    if(drv_info.enable) return 0;

    can->send_frame.data[0]='1';
    can->send_frame.data[1]='A';
    can->send_frame.can_id=0x401+client_id;
    can->Send();
    return 1;
}

void TMotorDrv::ClrFlt()
{
    if (drv_info.enable) return;
    can->send_frame.data[0]='5';
    can->send_frame.data[1]='4';
    can->send_frame.can_id=0x401+client_id;
    can->Send();
}

void TMotorDrv::Moving(float val)
{
    if(!drv_info.enable) return;

    //if(fabs(drv_info.spd)>1000)  printf("%.1f\n",val);

    char tmpbuf[4];
    memcpy(&tmpbuf,&val,4);

    can->send_frame.data[0]='5';
    can->send_frame.data[1]='2';
    for (int i = 0; i < 4; ++i) can->send_frame.data[7-i] = tmpbuf[i];
    can->send_frame.can_id=0x401+client_id;
    can->Send();
    usleep(1000);
}

int TMotorDrv::canMsgProc()
{
    int r=0;
    TTimer print_timer;
    for(int n=0;n<can->rec_frames.size();n++)
    {
        can_frame *frame=&(can->rec_frames[n]);
        if((frame->can_id&0x0F)!=client_id+1) continue; //区分驱动器


        int frame_id=frame->can_id&0xFF0;
        if(frame_id==0x640)          //  1000ms
        {
            r++;
            drv_info.wrn = frame->data[1] * 256 + frame->data[0];
            drv_info.digit_in = frame->data[3];
            drv_info.digit_out = frame->data[4];
            drv_info.mode = frame->data[5];
            drv_info.src = frame->data[6];
            drv_info.bv = frame->data[7]*2;

//            if(pre_drv_enable==frame->data[2]) drv_info.enable = frame->data[2];
//            pre_drv_enable=frame->data[2];
            drv_info.enable = frame->data[2];

//            printf("%d\n",drv_info.enable);
        }

        else if(frame_id==0x440)    //  2ms
        {
            r++;
//            drv_info.ia=sqrt((frame->data[4]*256+frame->data[5])*0.01);
            drv_info.ia=(frame->data[4]*256+frame->data[5])*0.01-50.0;
            drv_info.spd=frame->data[6]*256+frame->data[7]-32000;
            char tmpbuf[4];
            for (int i = 0; i < 4; ++i) tmpbuf[i]=frame->data[3-i];
            memcpy(&drv_info.pos,&tmpbuf,4);    //从tmpbuf复制4个数据到pos
//    if(print_timer.GetValue()>0.1) {
//        printf("%.3f\n", drv_info.ia);   //change
//        print_timer.Clear();
//    }
//            printf("ia=%.3f  spd=%.3f\n",drv_info.ia ,drv_info.spd);
        }


        frame->can_id=0;
    }


    return r;
}

void TMotorDrv::run()
{
    TTimer tim;
    int tmp_beat=0;

//    while(1)
//    {
//        usleep(2000);

//        if(tim.GetValue()>1)
//        {
//            heartbeat=tmp_beat/tim.GetValue();
//            tmp_beat=0;
//            tim.Clear();
//        }
//        tmp_beat+=canMsgProc();
//    }
}
