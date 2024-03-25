/*
 * linux/can.h
 *
 * Definitions for CAN network layer (socket addr / CAN frame / CAN filter)
 *
 * Authors: Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 *          Urs Thuermann   <urs.thuermann@volkswagen.de>
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Send feedback to <socketcan-users@lists.berlios.de>
 *
 */

#ifndef CAN_MOTOR_DRV_H
#define CAN_MOTOR_DRV_H

#include <linux/types.h>
#include "public.h"
#include "can.h"


struct DrvInfo
{
    float pos;
    float spd;
    float ia;
    float bv;
    uint enable;
    uint digit_in;
    uint digit_out;
    uint wrn;
    uint mode;
    uint src;
    uint id;
};


class TMotorDrv: public Thread
{
private:
    TCan *can;

public:
    DrvInfo drv_info;
    int pre_drv_enable;

    uint client_id;
    int heartbeat=0;

    TMotorDrv(uint clt_id,TCan *c,int mode=3);   //  初始化,也就是构造函数的声明 clt_id 驱动器的编号
    void Enable(bool v);
    void Monitor(bool v);
    int RunMode(uint run_mode);  //  0-torque?  2-speed 3-position
    void Unlock(bool v); //代替了制动
    int SetZero();  //相对编码器设置0位
    void ClrFlt();  //清除报警
    void Moving(float val);
    void run();       //  线程函数
    int canMsgProc();
};



#endif
