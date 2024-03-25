#ifndef NEW_6DOF_AXIS_H
#define NEW_6DOF_AXIS_H


#include <stdio.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <string>
#include "myudp.h"
#include "inic.h"
#include "motordrv.h"
#include "command.h"

using namespace std;

class TAxis: public TMotorDrv
{
private:
    int ctrmode;

public:
    int channel;
    float giv_value;
    float pos,speed,ia;

    TUDP *udp=NULL;
    int active;

    TAxis(int m, int clt_id, TCan *c);
    void SetCtrMode(int m);
    void Stop(void);
    void UDP_Proc();
    void run();
};


#endif //NEW_6DOF_AXIS_H
