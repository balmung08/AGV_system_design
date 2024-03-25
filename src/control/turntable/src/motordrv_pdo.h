#include <common/can.h>
#include <common/public.h>

class TMotorDrv: public Thread
{
private:
    TCan *can;
    int can_id;
    int CntPerR=524288;
    vector<can_frame> send_frames;
    can_frame first_frame;
    int pos1_cnt, posoff_cnt, spd_cnt;
    float send_interval=0.001;    // unit s  
    float can_interval=0.1;
    int err_code=0;

  public:
    float pos_deg, spd_deg;
    int pos_cnt;
    bool servo_on = false;
    int heartbeat=0;
    char state[10];

    TMotorDrv(int ch_id, int f_id, bool e=false)
    {
        can=InitCan(ch_id,1000000);
        can_id=f_id;

        first_frame.can_dlc=8;
        first_frame.can_id=0;

        can_frame frame;
        frame.can_dlc=8;
        frame.can_id=0x600+can_id;

        start();

        Disable();
        usleep(200000);

        StopRemoteNode();
        ResetRemoteNode();
        SetOpMode(1);
        // SetAccSpd(900000,300000);
        SetCommCycle(4000);
        TPDO_Mapping();
        RPDO_Mapping();
        StartRemoteNode();
        
        usleep(200000);
        // if(e) Enable();
    }

    void StopRemoteNode()
    {
        can_frame frame;
        frame.can_id=0x00;
        frame.can_dlc=2;
        frame.data[0]=0x02,  frame.data[1]=can_id;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void ResetRemoteNode()
    {
        can_frame frame;
        frame.can_dlc=2;
        frame.can_id=0x00;
        frame.data[0]=0x82,  frame.data[1]=can_id;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void StartRemoteNode()
    {
        can_frame frame;
        frame.can_dlc=2;
        frame.can_id=0x00;
        frame.data[0]=0x01,  frame.data[1]=can_id;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.can_dlc=0;
        frame.can_id=0x700+can_id;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.can_dlc=8;
        frame.can_id=0x600+can_id;
        // get pos
        frame.data[0]=0x40,  frame.data[1]=0x64,  frame.data[2]=0x60,  frame.data[3]=0x00;
        frame.data[4]=0x00,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void SetOpMode(int mode)   //  CSP
    {
        can_frame frame;
        frame.can_dlc=8;
        frame.can_id=0x600+can_id;
        frame.data[0]=0x2F,  frame.data[1]=0x60,  frame.data[2]=0x60,  frame.data[3]=0x00;
        frame.data[4]=mode,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // just for confirm
        frame.data[0]=0x40,  frame.data[1]=0x61,  frame.data[2]=0x60,  frame.data[3]=0x00;
        frame.data[4]=0x00,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void SetAccSpd(int acc, int spd)
    {
        can_frame frame;
        frame.can_dlc=8;
        frame.can_id=0x600+can_id;
        frame.data[0]=0x23,  frame.data[1]=0x81,  frame.data[2]=0x60,  frame.data[3]=0x00;
        memcpy(&frame.data[4],&spd,4);
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.data[0]=0x23,  frame.data[1]=0x83,  frame.data[2]=0x60,  frame.data[3]=0x00;
        memcpy(&frame.data[4],&acc,4);
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.data[0]=0x23,  frame.data[1]=0x84,  frame.data[2]=0x60,  frame.data[3]=0x00;
        memcpy(&frame.data[4],&acc,4);
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    
    void SetCommCycle(int t)   //  us
    {
        can_frame frame;
        frame.can_dlc=8;
        frame.can_id=0x600+can_id;
        // close sync
        frame.data[0]=0x23,  frame.data[1]=0x05,  frame.data[2]=0x10,  frame.data[3]=0x00;
        frame.data[4]=0x80,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        //  set timeer  not less than 1ms  
        frame.data[0]=0x23,  frame.data[1]=0x06,  frame.data[2]=0x10,  frame.data[3]=0x00;
        memcpy(&frame.data[4],&t,4);
        // printf("%02x %02x %02x %02x\n", frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // start sync
        frame.data[0]=0x23,  frame.data[1]=0x05,  frame.data[2]=0x10,  frame.data[3]=0x00;
        frame.data[4]=0x80,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x40;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void Enable()
    {
        can_frame frame;
        frame.can_dlc=6;
        frame.can_id=0x200+can_id;
        // clear fault
        frame.data[0]=0x80,  frame.data[1]=0x00,  frame.data[2]=0x00;  
        frame.data[3]=0x00,  frame.data[4]=0x00,  frame.data[5]=0x00;  
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.data[0]=0x06,  frame.data[1]=0x00,  frame.data[2]=0x00;  
        frame.data[3]=0x00,  frame.data[4]=0x00,  frame.data[5]=0x00;  
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.data[0]=0x07,  frame.data[1]=0x00,  frame.data[2]=0x00;  
        frame.data[3]=0x00,  frame.data[4]=0x00,  frame.data[5]=0x00;  
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.data[0]=0x0F,  frame.data[1]=0x00;
        memcpy(&frame.data[2],&pos_cnt,4); 
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        frame.can_dlc=0;
        frame.can_id=0x80;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void Disable()
    {
        can_frame frame;
        frame.can_dlc=6;
        frame.can_id=0x200+can_id;
        frame.data[0]=0x06,  frame.data[1]=0x00,  frame.data[2]=0x00;  
        frame.data[3]=0x00,  frame.data[4]=0x00,  frame.data[5]=0x00;  
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }    

    void TPDO_Mapping()
    {
        can_frame frame;
        frame.can_dlc=8;
        frame.can_id=0x600+can_id;
        //  close TPDO
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x18,  frame.data[3]=0x01;
        frame.data[4]=0x80+can_id,  frame.data[5]=0x01,  frame.data[6]=0x00,  frame.data[7]=0x80;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // set trans mode
        frame.data[0]=0x2F,  frame.data[1]=0x00,  frame.data[2]=0x18,  frame.data[3]=0x02;
        frame.data[4]=0x01,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // clear mapping
        frame.data[0]=0x2F,  frame.data[1]=0x00,  frame.data[2]=0x1A,  frame.data[3]=0x00;
        frame.data[4]=0x00,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // 1A00h-01h  mapping 60410010h  state
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x1A,  frame.data[3]=0x01;
        frame.data[4]=0x10,  frame.data[5]=0x00,  frame.data[6]=0x41,  frame.data[7]=0x60;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // 1A00h-02h  mapping 60410020h  pos
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x1A,  frame.data[3]=0x02;
        frame.data[4]=0x20,  frame.data[5]=0x00,  frame.data[6]=0x64,  frame.data[7]=0x60;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // 1A00h-00h  write mapping 2
        frame.data[0]=0x2F,  frame.data[1]=0x00,  frame.data[2]=0x1A,  frame.data[3]=0x00;
        frame.data[4]=0x02,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // open TPDO
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x18,  frame.data[3]=0x01;
        frame.data[4]=0x80+can_id,  frame.data[5]=0x01,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void RPDO_Mapping()
    {
        can_frame frame;
        frame.can_dlc=8;
        frame.can_id=0x600+can_id;
        //  close RPDO
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x14,  frame.data[3]=0x01;
        frame.data[4]=can_id,  frame.data[5]=0x02,  frame.data[6]=0x00,  frame.data[7]=0x80;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        //  set trans mode
        frame.data[0]=0x2F,  frame.data[1]=0x00,  frame.data[2]=0x14,  frame.data[3]=0x02;
        frame.data[4]=0x01,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // clear mapping
        frame.data[0]=0x2F,  frame.data[1]=0x00,  frame.data[2]=0x16,  frame.data[3]=0x00;
        frame.data[4]=0x00,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // 1600h-01h  mapping 60400010h  ctr word
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x16,  frame.data[3]=0x01;
        frame.data[4]=0x10,  frame.data[5]=0x00,  frame.data[6]=0x40,  frame.data[7]=0x60;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // 1600h-02h  mapping 607A0020h  target pos
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x16,  frame.data[3]=0x02;
        frame.data[4]=0x20,  frame.data[5]=0x00,  frame.data[6]=0x7A,  frame.data[7]=0x60;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // 1600h-00h  write mapping 2
        frame.data[0]=0x2F,  frame.data[1]=0x00,  frame.data[2]=0x16,  frame.data[3]=0x00;
        frame.data[4]=0x02,  frame.data[5]=0x00,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);

        // open RPDO
        frame.data[0]=0x23,  frame.data[1]=0x00,  frame.data[2]=0x14,  frame.data[3]=0x01;
        frame.data[4]=can_id,  frame.data[5]=0x02,  frame.data[6]=0x00,  frame.data[7]=0x00;
        can->send_frame=frame;
        can->Send();
        usleep(can_interval*1000000);
    }

    void SetTargetPosCnt(int v)
    {
        can_frame frame;
        frame.can_dlc=6;
        frame.can_id=0x200+can_id;
        frame.data[0]=0x1F,  frame.data[1]=0x00;
        memcpy(&frame.data[2],&v,4);
        //can->send_frame=frame;
        can->Send(frame);
        usleep(2000);
    }

    void SetPosZero(int v)
    {
        posoff_cnt=v;
    }

    void SetTargetPosDeg(float v)
    {
        int p=v/360.0*CntPerR;
        p+=posoff_cnt;
        SetTargetPosCnt(p);
        // printf("%.1f\n",v);
    }

    void run()
    {   
        int sindex=0;
        int tmpcount=0;
        TTimer tmr;

        while(1)
        {
           if(tmr.GetValue()>1)
           {
               heartbeat=tmpcount/tmr.GetValue();
               tmr.Clear();
               tmpcount=0;
              // printf("%d=%d \n",can_id,heartbeat);
           }
                
           usleep(send_interval*1000000);

           for(int i=0;i<can->rec_frames.size();i++)
           {
               can_frame *frame=&can->rec_frames[i];
               if(frame->can_id==0x180+can_id)
               {
                   memcpy(&pos_cnt, &frame->data[2],4);
                   pos1_cnt=pos_cnt-posoff_cnt;
                   pos_deg=pos1_cnt*1.0/CntPerR*360;
                   tmpcount++;

                   frame->can_id=0;
                   state[0]=frame->data[0];
                   state[1]=frame->data[1];
                   servo_on=((state[1] & 0x04)==0x04); //(state[0]==0x37 && state[1]==0x16);
                //    printf("%x: %d %02x %02x\n",can_id, servo_on, frame->data[0], frame->data[1]);
                //    printf("%d  %d  %.1f\n",can_id, pos_cnt,pos_deg);

                //    printf("receive  %x: ",frame->can_id);
                //    for(int i=2;i<frame->can_dlc;i++)  printf("%02x ",frame->data[i]);
                //    printf("\n");
                    
               }
               //frame->can_id=0;
           }
        }
    }
};



