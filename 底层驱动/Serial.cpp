//
// Created by ros on 7/8/21.
//

#include <termios.h>
#include <math.h>
#include "Serial.h"

using namespace std;

int speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200};//这其中的数字代表的便是波特率了
int name_arr[] = {115200, 57600, 38400,  19200,  9600,  4800,  2400, 1200};

void serial_setspeed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios Opt;   //串口的设置主要是设置termios 结构体各成员的值
    tcgetattr(fd, &Opt);  //获取与终端相关的参数，结果保存在fd中，成功返回0，失败返回-1，fd相当于是串口描述符了，以fd代指结构体Opt
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)//字符串中数据的个数，遍历比较是否有与speed相等的内容
    {
        if (speed == name_arr[i])  //匹配到相等的speed
        {
            tcflush(fd, TCIOFLUSH);  //清空输入输出缓冲区
            cfsetispeed(&Opt, speed_arr[i]);  //设置输入波特率
            cfsetospeed(&Opt, speed_arr[i]);  //设置输出波特率
            status = tcsetattr(fd, TCSANOW, &Opt); //应用新的设置
            if (status != 0)
                perror("tcsetattr fd1");
            return;
        }
        tcflush(fd,TCIOFLUSH);//再次清空输入输出缓冲区
    }
}

int serial_setparity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");  //这是一个函数，输出上一个函数出错的原因
        return(false);
    }
    options.c_cflag &= ~CSIZE;  //结构体设置对数据bit位使用掩码
    switch (databits)
    {
        case 7:
            options.c_cflag |= CS7;  //使用7位数据位
            break;
        case 8:
            options.c_cflag |= CS8;  //使用8位数据位
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return (false);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; //控制标志不进行奇偶校验
            options.c_iflag &= ~INPCK;  //输入标志不进行奇偶校验
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);  //控制标志启动奇校验
            options.c_iflag |= INPCK;  //打开输入奇偶校验
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 'S':
        case 's':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (false);
    }
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;  //一位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB; //两位停止位
            break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return (false);
    }

    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);

    /* Set input parity option */

    if (parity != 'n')
        options.c_iflag |= INPCK;
    options.c_cc[VTIME] = 150; // 定义了等待的时间，单位是百毫秒
    options.c_cc[VMIN] = 0; //定义了要求等待的最小字节数


    tcflush(fd,TCIFLUSH); //使以上配置生效
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (false);
    }
    return (false);

}


TCOM::TCOM(char *devname)
{
    fd = open(devname, O_RDWR); //打开串口，可读可写
    if (fd < 0)
    {
        printf("open device %s faild\n", devname);
        //    exit(0);
    }
    SetSpeed(115200);    //波特率115200，更改在此设置
    SetParity(8,1,'N');
    RecCount=0;

    start();  // 启动线程
}

void TCOM::SetSpeed(int speed)
{
    serial_setspeed(fd,speed);
}

int TCOM::SetParity(int databits, int stopbits, int parity)
{
    serial_setparity(fd,databits,stopbits,parity);
}

void TCOM::Send(unsigned char *buf,int count)
{
    int len = 0;
    //pthread_mutex_lock(&mutex);
    len = write(fd, buf, count);//实际写入的长度
    usleep(100);
    //pthread_mutex_lock(&mutex);
}

void TCOM::run()
{
    int tmp_bt=0;
    TTimer tmr;

    while(1)
    {
        if(tmr.GetValue()>=1)
        {
            Hearbeat=tmp_bt/tmr.GetValue();
            tmp_bt=0;
            tmr.Clear();
            Hearbeat=0;
        }

        unsigned char buf[200];
        int n=read(fd, buf, 200);
        if (n > 0 && n+RecCount<1000)
        {
            memcpy(RecBuf+RecCount,buf,n);
            RecCount+=n;
            RecBuf[RecCount]=0;

            tmp_bt++;

        }

        usleep(10);
    }
}

//attitude_pitch=channel_data[0];
//attitude_roll=channel_data[1];
//attitude_yaw=channel_data[2];
//batteryAverageCellVoltage=channel_data[3];
//currentMeterAmperage=channel_data[4];
//nowMAhDrawn=channel_data[5];
//batteryRemainingPercentage=channel_data[6];
//gps_lat=channel_data[7];
//gps_lon=channel_data[8];
//gps_groundSpeed=channel_data[9];
//gps_groundCourse=channel_data[10];
//gps_numSat=channel_data[11];
//gps_altitude=channel_data[12];

void TRemoteCtr::SendData()
{
    unsigned char buf[100];
    buf[0]=0xff;
//    SCH[3]=200;    // 电池电压
//    SCH[6]=80;     // 电池SOC
//    SCH[5]=0x0F;   // EnableState

    for(int i=0;i<12;i++)  buf[1+i*2]=SCH[i]>>8,  buf[2+i*2]=SCH[i];
    buf[25]=0x00;
    com->Send(buf,26);
}


TRemoteCtr::TRemoteCtr(int com_id)
{
    char buf[100];
    sprintf(buf,"/dev/ttyS%d",com_id);
    com=new TCOM(buf);
    com->SetSpeed(115200);
    heartbeat=0;

    start();
}

void TRemoteCtr::run()
{
    int count=0;

    //sleep(1);
    while(1)
    {
        usleep(1000);
        if(tmr_hb.GetValue()>=1)
        {
            heartbeat=count/tmr_hb.GetValue();
            tmr_hb.Clear();
            count=0;
            //printf("%d\n", heartbeat);
        }

        //printf("%d\n",com->RecCount);
        if(com->RecCount>=40)
        {
            for (int i = 0; i <= com->RecCount-35; ++i)
                if ( (com->RecBuf[i] == 0x0f && com->RecBuf[i+33] == 0x00) )  // SBUS
//                     if ((com->RecBuf[i] == 0xff && com->RecBuf[i+34] == 0x00))    // ELRS
                {
                    for(int k=0;k<max_ch_count;k++)
                    {
                        CH[k] = com->RecBuf[i+k*2+1]*256 + com->RecBuf[i+k*2+2];
                        CH[k] = (CH[k]-992)*100/820;
                    }
//                    for(int k=0;k<max_ch_count;k++)
//                        printf("CH%d:%d | ",k,CH[k]);
//                    printf("\n");
//                    usleep(10000);

                    count++;
                    com->RecCount=0;
                    SendData();
                    break;
                }
            if(com->RecCount>=200)  com->RecCount=0;
        }

    }
}

TIMU::TIMU()
{
    char buf[100];
    com=new TCOM("/dev/ttyS3");
    com->SetSpeed(115200);

    usleep(20000);
    //SetZero();

    start();
}

void TIMU::run()
{
    TTimer tmr_hb;
    int tmpcount=0;

    while(1)
    {
        if(tmr_hb.GetValue()>=1)
        {
            HeartBeat=tmpcount/tmr_hb.GetValue();
            tmpcount=0;
            tmr_hb.Clear();
            //printf("%d\n",HeartBeat);
        }

        unsigned char cmd[10]={0x68, 4, 0, 4, 8};
        com->Send(cmd,5);
        usleep(10000);

        if(com->RecCount>=14 && com->RecBuf[0]==0x68 && com->RecBuf[1]==0x0D)// && com->RecBuf[2]==0x24)
        {
//            for(int i=0;i<com->RecCount;i++)  printf("%02x ",com->RecBuf[i]);
//            printf("  S=%d \n",com->RecCount);

            for(int i=0;i<3;i++)
            {
                attiAng[i]=com->RecBuf[5+3*i]+com->RecBuf[6+3*i]*0.01;
                if(com->RecBuf[4+3*i]>0) attiAng[i]=-attiAng[i];
            }
            //printf("\n");
            tmpcount++;
        }
        com->RecCount=0;
    }
}
