//
// Created by ros on 8/18/21.
//

#include "Axis.h"

TAxis::TAxis(int ch, int clt_id, TCan *c): TMotorDrv(clt_id, c)
{
    channel=ch;
    active=-1;
    //SetCtrMode(POS_CONTROL);
    //giv_value = 0;
    //SetZero();
    start();
}

void TAxis::SetCtrMode(int m)
{
    ctrmode=m;
    if(ctrmode==POS_CONTROL) RunMode(3);
    else if(ctrmode==SPEED_CONTROL) RunMode(2);
    else if(ctrmode==FORCE_CONTROL) RunMode(0);
}


void TAxis::Stop(void)
{
    Moving(0);
}

void TAxis::UDP_Proc()
{
}

void TAxis::run()
{
    TTimer tim;
    int tmp_beat=0;

    while(1)
    {
        usleep(10000);   //  2ms 控制周期
        UDP_Proc();   //???似乎可以删除
        tmp_beat+=canMsgProc();

        if(tim.GetValue()>2)
        {
            heartbeat=tmp_beat/tim.GetValue();
            //printf("ch=%d %d\n",channel, heartbeat);

            tmp_beat=0;
            tim.Clear();
            //if(channel==0)
        }

        pos = drv_info.pos;
        speed = drv_info.spd;
        ia = drv_info.ia;
        active = drv_info.enable;

        //printf("%d\n", drv_info.enable);

//        int cmd_enable=0;
//        if(cmd!=NULL&&cmd->runstate)
//        {
//            cmd_enable=1;
//            ctrmode=cmd->ctrmode;
//        }


//        if (ctrmode == POS_CONTROL)
//        {
//            //if(givpos>Stroke) givpos=Stroke;
//
//            if (AbsEncode) Moving(initlen - giv_value);
//            else Moving(giv_value);
//        }
//        if (ctrmode == FORCE_CONTROL)
//        {
//
//        }
//        if (ctrmode == SPEED_CONTROL)
//        {
//            //            givpos=actpos;
//            //            cv=cmd->value;
//        }
    }
}
