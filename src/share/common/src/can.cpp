#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>
#include "common/can.h"

#define PF_CAN 29
#define AF_CAN PF_CAN
#define SIOCSCANBAUDRATE        (SIOCDEVPRIVATE+0)
#define SIOCGCANBAUDRATE        (SIOCDEVPRIVATE+1)
#define SOL_CAN_RAW (SOL_CAN_BASE + CAN_RAW)
#define CAN_RAW_FILTER  1
#define CAN_RAW_RECV_OWN_MSGS 0x4 

struct ifreq ifr[4];

int Can_Init(int id, int rate)
{
    int sock=-1;
    struct sockaddr_can addr;

    char buf[100];
    // sprintf(buf,"ip link set can%d down",id);
    // system(buf);
    // sprintf(buf,"ip link set can%d up type can bitrate %d",id, rate);
    // system(buf);

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock < 0)
    {
        printf("error\n");
        return -1;
    }

    addr.can_family = AF_CAN;
    sprintf(ifr[id].ifr_name, "can%d",id);

    int ret;
    ret = ioctl(sock, SIOCGIFINDEX, &ifr[id]);  //get index
    if(ret && ifr[id].ifr_ifindex == 0)
    {
        printf("Can't get interface index for can0, code= %d, can0 ifr_ifindex value: %d, name: %s\n", ret, ifr[id].ifr_ifindex, ifr[id].ifr_name);
        close(sock);
        return -1;
    }

    //printf("%s can_ifindex = %x\n",ifr.ifr_name,ifr.ifr_ifindex);
    addr.can_ifindex = ifr[id].ifr_ifindex;
    //ioctl(sock,SIOCGIFNAME,&ifr);
    //printf("ret = %d can0 can_ifname = %s\n",ret,ifr.ifr_name);

    int recv_own_msgs = 0;//set loop back:  1 enable 0 disable

    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,&recv_own_msgs, sizeof(recv_own_msgs));
    if (bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0)
    {
        printf("bind error\n");
        close(sock);
        return -1;
    }

    return sock;
};

TCan::TCan(int id, int rate)
{
    can_id=id;
    sock=Can_Init(id, rate);
    if(sock<0) printf("can init error!\n");
    else start(), printf("can%d open\n",id);

    rec_flag=false;
    busy=false;
};

void TCan::Send(can_frame frame)
{
    if(sock<0) return;

    while(busy);
    busy=true;

    //pthread_mutex_lock(&mutex);
    usleep(200);
    write(sock, &frame, sizeof(struct can_frame));
    usleep(200);
    //pthread_mutex_unlock(&mutex);

    busy=false;
}

void TCan::Send()  //  发送指令
{
    if(sock<0) return;

    while(busy);
    busy=true;

    //pthread_mutex_lock(&mutex);
    usleep(100);
    write(sock, &send_frame, sizeof(struct can_frame));
    // printf("can_id=%04x  ",send_frame.can_id);
    // for(int i=0;i<send_frame.can_dlc;i++)
    //    printf("%02x ",send_frame.data[i]);
    // printf("\n");   
    usleep(100);
    busy=false;
    //pthread_mutex_unlock(&mutex);
};

void TCan::run()    //  线程函数
{
    can_frame frame;
    while(1)
    {
        usleep(2);

        memset(&frame, 0, sizeof(frame));
        read(sock, &frame, sizeof(frame));

        for(auto it = rec_frames.begin(); it != rec_frames.end();)
        {
            if (it->can_id==0) rec_frames.erase(it);
            else it++;
        }

        if (frame.can_dlc)
        {
            rec_flag=true;
            memcpy(&rec_frame, &frame, sizeof(can_frame));

            if(rec_frames.size()>=100)  rec_frames.erase(rec_frames.begin());
            rec_frames.push_back(frame);
            // printf("can_id=%d   frame_id=%04x \n",can_id, rec_frame.can_id);
        }
    }
};

TCan *InitCan(int id, int rate)
{
    static TCan *can[2] = {NULL, NULL};

    if (can[id] == NULL)
        can[id] = new TCan(id, rate);
    return can[id];
}
