//
// Created by ros on 7/8/21.
//

#ifndef NEW_6DOF_SERIAL_H
#define NEW_6DOF_SERIAL_H

#include <iostream>
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termio.h>
#include "public.h"

void serial_setspeed(int fd, int speed);
int serial_setparity(int fd, int databits, int stopbits, int parity);


class TCOM: public Thread
{
private:
    int fd;
    pthread_mutex_t mutex;    // 互斥锁

public:
    TCOM(char *devname="/dev/ttyS2");
    int RecCount;
    unsigned char RecBuf[1000];
    int Hearbeat;

    void SetSpeed(int speed);
    int SetParity(int databits, int stopbits, int parity);
    void Send(unsigned char *buf,int count);
    void run();
};

class TRemoteCtr: public Thread
{
private:
    TCOM *com;
    TTimer tmr_hb;

public:
    int max_ch_count=14;
    int CH[20];
    unsigned int SCH[20];
    int elrs_rate;
    int heartbeat;

    TRemoteCtr(int com_id);
    void run();
    void SendData();
};

class TIMU: public Thread
{
private:
    TCOM *com;

public:
    float attiAng[3]={0,0,0},angVel[3]={0,0,0},linAcc[3]={0,0,0};//姿态角,角速度,线加速度
    int HeartBeat;

    TIMU();
    void SetZero(void);
    void run();
};


#endif //NEW_6DOF_SERIAL_H
