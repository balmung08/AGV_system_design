
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termio.h>
#include <termios.h>
#include <math.h>


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


class TSerial
{
private:
    int fd;

public:
    int RecCount;
    unsigned char RecBuf[200];

    TSerial(char *devname="/dev/ttyS0")
    {
        fd = open(devname, O_RDWR); //打开串口，可读可写
        if (fd < 0)
        {
            printf("open device %s faild\n", devname);
        }

        SetSpeed(9600);   //波特率115200，更改在此设置
        SetParity(8,1,'N');
    }

    void SetSpeed(int speed)
    {
        serial_setspeed(fd,speed);
    }

    int SetParity(int databits, int stopbits, int parity)
    {
        serial_setparity(fd,databits,stopbits,parity);
    }

    void Send(unsigned char *buf,int count)
    {
        write(fd, buf, count);
    }

    int Read(unsigned char *buf,int count)
    {
        memset(buf,0, count);
	    count=read(fd,buf,count);
        return count;
    }
   
};