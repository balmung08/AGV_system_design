#include <common/can.h>
#include <common/public.h>

class TMotorDrv
{
private:
    TCan *can;
    int can_id;
    int CntPerR=524288;
    can_frame send_frame, rec_frame;
    int pos1_cnt, posoff_cnt, spd_cnt;
    float send_interval=0.001;    // unit s  
    int err_code=0;
    int halt_ctr=0;

  public:
    float pos_deg, spd_deg;
    int pos_cnt;
    bool servo_on = false, reach_flag=true;
    THeartbeat heartbeat;

    TMotorDrv(int ch_id, int f_id, int off_cnt)
    {
        can=InitCan(ch_id,1000000);
        can_id=f_id;

        send_frame.can_dlc=8;
        send_frame.can_id=0x600+can_id;

        SetPosZero(off_cnt);
        CheckError();
        ClearError();
        SetPPMode();
        CheckPPMode();

        CheckStatus();
	    SetTargetPosDeg(pos_deg);
        Enable(0);
        usleep(500000);
    }

    int GetMsg(can_frame &frame)
    {
        can->rec_flag=0;
        for(int i=0;i<100;i++)  
        {
            usleep(10);
            if(can->rec_flag && (can->rec_frame.can_id & 0x0F)==can_id)   
            {
                frame=can->rec_frame;  //printf("%d\n", i);
                return 1;
            }
        }
        
        return 0;
    }

    void CheckError()
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x40,  send_frame.data[1]=0x3F,  send_frame.data[2]=0x60;
        can->send_frame=send_frame;
        can->Send();
        GetMsg(rec_frame);
    }

    void ClearError()
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x2b,  send_frame.data[1]=0x40,  send_frame.data[2]=0x60,  send_frame.data[4]=0x80;
        can->send_frame=send_frame;
        can->Send();
        GetMsg(rec_frame);
    }

    void SetPPMode()
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x2f,  send_frame.data[1]=0x60,  send_frame.data[2]=0x60,  send_frame.data[4]=0x01;
        can->send_frame=send_frame;
        can->Send();
        GetMsg(rec_frame);   
    }

    void CheckPPMode()
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x40,  send_frame.data[1]=0x61,  send_frame.data[2]=0x60;
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame))  
        {
            // for(int i=0;i<8;i++)  printf("%02x ", rec_frame.data[i]);
            // printf("  id=%d\n", can_id);
        }   
    }

    void SetPosZero(int v)
    {
        posoff_cnt=v;
    }

    void CheckPos()
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x40,  send_frame.data[1]=0x64,  send_frame.data[2]=0x60;
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame))  
        {
            memcpy(&pos_cnt, &rec_frame.data[4],4);
            pos1_cnt=pos_cnt-posoff_cnt;
            pos_deg=pos1_cnt*1.0/CntPerR*360;
            //printf("%d %.2f\n", pos_cnt, pos_deg);
        }  
        // else ROS_INFO("err\n"); 
    }

    void SetTargetPosDeg(float v)
    {
        int p=v/360.0*CntPerR;
        p+=posoff_cnt;
        SetTargetPosCnt(p);
    }

    void SetTargetPosCnt(int v)
    {
        send_frame.data[0]=0x23,  send_frame.data[1]=0x7a,  send_frame.data[2]=0x60;
        memcpy(&send_frame.data[4],&v,4);
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame));   

        if(servo_on)
        {
            for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

            send_frame.data[0]=0x2b,  send_frame.data[1]=0x40,  send_frame.data[2]=0x60;
            send_frame.data[4]=0x0F;  send_frame.data[5]=halt_ctr;
            can->send_frame=send_frame;
            can->Send();
            if(GetMsg(rec_frame))  {}  

            send_frame.data[4]=0x1F;
            can->send_frame=send_frame;
            can->Send();
            if(GetMsg(rec_frame))  {}
        }
    }

    void Stop(bool v)
    {
        halt_ctr=v;  
    }

    void SetAccSpd(int acc, int spd)
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x23,  send_frame.data[1]=0x83,  send_frame.data[2]=0x60;
        memcpy(&send_frame.data[4],&acc,4);

        // printf("%02x %02x %02x %02x\n", send_frame.data[4],send_frame.data[5],send_frame.data[6],send_frame.data[7]);
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame));

        send_frame.data[0]=0x23,  send_frame.data[1]=0x84,  send_frame.data[2]=0x60;
        memcpy(&send_frame.data[4],&acc,4);
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame));  

        send_frame.data[0]=0x23,  send_frame.data[1]=0x81,  send_frame.data[2]=0x60;
        memcpy(&send_frame.data[4],&acc,4);
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame));
    }

    void SetSpd_Deg(float spd)
    {
        int spd_cnt=spd/360.0*CntPerR;
        SetAccSpd(spd_cnt, spd_cnt);
    }

    void CheckServo()
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        send_frame.data[0]=0x40,  send_frame.data[1]=0x41,  send_frame.data[2]=0x60;
        can->send_frame=send_frame;
        can->Send();
        if(GetMsg(rec_frame))  
        {
            servo_on=((rec_frame.data[4] & 0x07)==0x07);
            reach_flag=((rec_frame.data[5] & 0x04)==0x04);

            heartbeat.Beat();
        }   
    }

    void Enable(int v)
    {
        for(int i=0;i<8;i++)  send_frame.data[i]=0x00;

        halt_ctr=0;
        if(v)
        {
            send_frame.data[0]=0x2b,  send_frame.data[1]=0x40,  send_frame.data[2]=0x60;
            send_frame.data[4]=0x07;  
            can->send_frame=send_frame;
            can->Send();
            if(GetMsg(rec_frame))  {}  

            send_frame.data[4]=0x0F;
            can->send_frame=send_frame;
            can->Send();
            if(GetMsg(rec_frame))  {}
        }
        else 
        {
            
            send_frame.data[0]=0x2b,  send_frame.data[1]=0x40,  send_frame.data[2]=0x60;
            send_frame.data[4]=0x06;  
            can->send_frame=send_frame;
            can->Send();
            if(GetMsg(rec_frame))  {}  
        }  

        usleep(500000);
    }

    void CheckStatus()
    {
        CheckPos();
        CheckServo();
    }

};



