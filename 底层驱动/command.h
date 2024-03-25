//
// Created by wsk on 19-10-27.
//

#ifndef NEW_6DOF_COMMAND_H
#define NEW_6DOF_COMMAND_H


#include "public.h"
#include <math.h>
#include <fstream>

#define POS_CONTROL 0
#define FORCE_CONTROL 1
#define SPEED_CONTROL 2
#define FORCE_POS_CONTROL 3

#define MAXSIGITEM 5
#define SIG_FIX 0
#define SIG_SIN 1
#define SIG_SPEEDMOVE 2
#define SIG_TRIMOVE 3
#define SIG_FILEDATA 4
#define SIG_USER 5
#define NONE -1

const float PI=3.1415926;
const float Ktha=PI/180;

class TRunDataFile                   // 将数据文件进行分解   a.txt 变为 a.txt.0 到 a.txt.6
{
private:
    ifstream *fin;
    ofstream *fout[10];
    string fname;
    int maxsubfilenum=6;

public:
    TRunDataFile(string fstr);
    void Clear();
};

class TCommandItem
{
public:
    int ctrmode, sigtype;
    float count;
    float para[10], value;
    vector<float> databuf;
    int enabled;
    TTimer tmr1,tmr2;
    ifstream *filehandle;

    TCommandItem();
    void Clear(void);
    int GetValue();

};


class TCommand: public Thread
{
private:

public:
    TCommandItem sigitem[MAXSIGITEM];
    int sigindex,sigcount;
    float value;
    int ctrmode, sigtype;
    int runstate;

    TCommand();
    void Clear(void);
    void AddSig(int ctrmode,int sigtype,float *data,int dcount);
    void AddSig(int ctrmode,int sigtype,float *data,int dcount,float dt);
    void AddFixSig(int ctrmode,float v,float t);
    void AddSinSig(int ctrmode,float amp,float freq,float offset,float phase,float count);
    void AddSinSig(int ctrmode,float amp,float freq,float offset,float count);
    void AddSinMoveSig(int ctrmode,float p1,float p2,float tw);
    void AddSpeedMoveSig(int ctrmode,float p1,float p2,float s,float st1,float st2);
    void AddTriMoveSig(int ctrmode,float p1,float p2,float s,float st1,float st2,int count);
    void AddFileData(int ctrmode,string fname,int count);
    void BeginRun();
    void run();
};

#endif //NEW_6DOF_COMMAND_H

