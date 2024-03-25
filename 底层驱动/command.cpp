//
// Created by ros on 8/18/21.
//

#include "command.h"

TRunDataFile::TRunDataFile(string fstr)                       //分解数据文件
{
    fname=fstr;
    fin=new ifstream(fstr);
    for(int i=0;i<maxsubfilenum;i++)
    {
        string s=fstr+"."+std::to_string(i);
        fout[i] = new ofstream(s);
    }
    string str;
    while(getline(*fin,str))
    {
        vector<string> strs=split(str," ");
        for(int i=0;i<strs.size();i++)
        {
            *fout[i]<<strs[i]<<"\n";
        }
    }
    for(int i=0;i<maxsubfilenum;i++) fout[i]->close();
}


void TRunDataFile::Clear()
{
    for(int i=0;i<maxsubfilenum;i++)
    {
        string s=fname+"."+std::to_string(i);
        remove(s.c_str());
    }
}

TCommandItem::TCommandItem()
{
    Clear();
}

void TCommandItem::Clear(void)
{
    tmr1.Clear();  tmr2.Clear();
    enabled=0;  count=0;
    databuf.clear();
    sigtype=NONE;
}

int TCommandItem::GetValue()
{
    if(enabled==0) return 0;

    float v;
    tmr1.GetValue();   tmr2.GetValue();

    if(sigtype==SIG_FIX)
    {
        v=para[0];
        if(para[1]>0 && tmr1.value>para[1]) enabled=0;
    }
    else if(sigtype==SIG_SIN)
    {
        v=para[0]*sin(2*PI*para[1]*tmr1.GetValue()+para[3])+para[2];
        count=tmr1.GetValue()*para[1];
        if(para[4]>0&&count>=para[4]) enabled=0;
    }
    else if(sigtype==SIG_SPEEDMOVE||sigtype==SIG_TRIMOVE)
    {
        float pos1=para[0], pos2=para[1], speed=para[2];
        float stopt1=para[3],stopt2=para[4];
        if(speed<0.0001) speed=200;
        float cycletime=fabs(pos1-pos2)/speed;


        if(tmr2.value<=stopt1) v=pos1;
        else if(tmr2.value>=stopt1&&tmr2.value<cycletime+stopt1)
        {
            if(pos1<pos2) v=pos1+speed*(tmr2.value-stopt1);
            else v=pos1-speed*(tmr2.value-stopt1);
        }
        else if(tmr2.value>=stopt1+cycletime&&tmr2.value<=cycletime+stopt1+stopt2)
            v=pos2;
        else if(tmr2.value>=stopt1+stopt2+cycletime&&sigtype==SIG_SPEEDMOVE)
        {
            v=pos2;  enabled=0;
        }
        else if(tmr2.value>=stopt1+stopt2+cycletime&&tmr2.value<=stopt1+stopt2+2*cycletime)
        {
            if(pos1<pos2) v=pos2-speed*(tmr2.value-cycletime-stopt1-stopt2);
            else v=pos2+speed*(tmr2.value-cycletime-stopt1-stopt2);
        }
        else if(tmr2.value>=stopt1+stopt2+2*cycletime&&tmr2.value<=2*stopt1+stopt2+2*cycletime)
            v=pos1;
        else if(tmr2.value>=2*stopt1+stopt2+2*cycletime&&sigtype==SIG_TRIMOVE)
        {
            count++;
            if(para[5]>=0&&count>=para[5]) enabled=0;
            else tmr2.Clear();
        }

    }
    else if(sigtype==SIG_USER)
    {
        int id=tmr1.value/para[1];
        if(id>=databuf.size()-1) enabled=0;
        else
        {
            double k=(databuf[id+1]-databuf[id])/para[1];
            k=k*(tmr1.value-id*para[1])+databuf[id];
            v=k;
        }
    }
    else if(sigtype==SIG_FILEDATA)
    {
        if(databuf.size()>0)
        {
            v=databuf[0];
            databuf.erase(databuf.begin());
        }
        else
        {
            string str;
            databuf.clear();
            for(int i=0;i<100;i++)
            {
                if(getline(*filehandle,str)) databuf.push_back(atof(str.c_str()));
                else break;
            }
            if(databuf.size()==0)
            {
                count++;
                filehandle->clear();
                filehandle->seekg(0);
            }
            if(para[0]>0 && count>=para[0]) enabled=0;
        }
    }
    value=v;

    return enabled;
}

TCommand::TCommand()
{
    Clear();
    start();
}

void TCommand::Clear(void)
{
    for(int i=0;i<sigcount;i++) sigitem[i].Clear();
    sigindex=0;  sigcount=0;
}

void TCommand::AddSig(int ctrmode,int sigtype,float *data,int dcount)
{
    if(sigcount>=MAXSIGITEM) return;

    TCommandItem *item=&sigitem[sigcount];
    item->Clear();

    item->ctrmode=ctrmode;
    item->sigtype=sigtype;
    for(int i=0;i<dcount;i++) item->para[i]=data[i];
    item->count=0;
    item->enabled=1;

    sigcount++;
}

void TCommand::AddSig(int ctrmode,int sigtype,float *data,int dcount,float dt)
{
    if(sigcount>=MAXSIGITEM) return;
    if(dcount>100) return;

    TCommandItem *item=&sigitem[sigcount];
    item->Clear();

    item->ctrmode=ctrmode;
    item->sigtype=sigtype;
    item->databuf.clear();
    for(int i=0;i<dcount;i++) item->databuf.push_back(data[i]);
    item->para[0]=dcount;
    item->para[1]=dt;
    item->count=0;
    item->enabled=1;

    sigcount++;
}

void TCommand::AddFixSig(int ctrmode,float v,float t)
{
    float data[10];
    data[0]=v,  data[1]=t;
    AddSig(ctrmode,SIG_FIX,data,2);
}

void TCommand::AddSinSig(int ctrmode,float amp,float freq,float offset,float phase,float count)
{
    float data[10];
    data[0]=amp,  data[1]=freq,  data[2]=offset, data[3]=phase, data[4]=count;
    AddSig(ctrmode,SIG_SIN,data,5);
}

void TCommand::AddSinSig(int ctrmode,float amp,float freq,float offset,float count)
{
    float data[10];
    data[0]=amp,  data[1]=freq,  data[2]=offset, data[3]=0,  data[4]=count;
    AddSig(ctrmode,SIG_SIN,data,5);
}

void TCommand::AddSinMoveSig(int ctrmode,float p1,float p2,float tw)
{
    float data[10];
    data[0]=(p2-p1)*0.5,  data[1]=0.5/tw,  data[2]=(p1+p2)*0.5, data[3]=-PI*0.5, data[4]=0.5;
    AddSig(ctrmode,SIG_SIN,data,5);
}

void TCommand::AddSpeedMoveSig(int ctrmode,float p1,float p2,float s,float st1,float st2)
{
    float data[10];
    data[0]=p1,  data[1]=p2,  data[2]=s, data[3]=st1, data[4]=st2;
    AddSig(ctrmode,SIG_SPEEDMOVE,data,5);
}

void TCommand::AddTriMoveSig(int ctrmode,float p1,float p2,float s,float st1,float st2,int count)
{
    float data[10];
    data[0]=p1,  data[1]=p2,  data[2]=s, data[3]=st1, data[4]=st2, data[5]=count;
    AddSig(ctrmode,SIG_TRIMOVE,data,6);
}

void TCommand::AddFileData(int ctrmode,string fname,int count)
{
    if(sigcount>=MAXSIGITEM) return;

    TCommandItem *item=&sigitem[sigcount];
    item->Clear();

    item->ctrmode=ctrmode;
    item->sigtype=SIG_FILEDATA;
    item->para[0]=count;
    item->count=0;
    item->filehandle=new ifstream(fname);

    if(item->filehandle)
    {
        item->enabled = 0, sigcount++;
    }
}

void TCommand::BeginRun()
{
    sigindex=0;
    for(int i=0;i<sigcount;i++)  sigitem[i].enabled=1;
}

void TCommand::run()
{
    while(1)
    {
        usleep(5000);
        int r = 0;
        if (sigcount > 0 && sigindex < sigcount)
        {
            if (sigitem[sigindex].GetValue()) value = sigitem[sigindex].value, r = 1;
            else
            {
                sigindex++;
                if (sigindex < sigcount)
                {
                    sigitem[sigindex].tmr1.Clear();
                    sigitem[sigindex].tmr2.Clear();
                }
            }
        }

        if (r) ctrmode = sigitem[sigindex].ctrmode, sigtype = sigitem[sigindex].sigtype;
        runstate=r;
    }
}
