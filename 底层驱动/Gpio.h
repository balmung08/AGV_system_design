
#ifndef UGV_WSK_GPIO_H
#define UGV_WSK_GPIO_H

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
#include "public.h"

#define DEV_GPIO "/dev/user_gpio"

#define GPIO_IOC_MAGIC   'G'
#define IOCTL_GPIO_SETOUTPUT              _IOW(GPIO_IOC_MAGIC, 0, int)
#define IOCTL_GPIO_SETINPUT               _IOW(GPIO_IOC_MAGIC, 1, int)
#define IOCTL_GPIO_SETVALUE               _IOW(GPIO_IOC_MAGIC, 2, int)
#define IOCTL_GPIO_GETVALUE    		  _IOR(GPIO_IOC_MAGIC, 3, int)

typedef struct {
    int pin;
    int data;
    int usepullup;
}gpio_arg;


class TDigit: public Thread
{
private:
    int fd;

public:
    int v_in[6], v_out[4];

    TDigit()
    {
        fd = open(DEV_GPIO, O_RDWR);
        start();
        v_out[0]=v_out[1]=v_out[2]=v_out[3]=1;
    };

    void Ctr(int id1,int id2,int v)
    {
        gpio_arg g;
        g.pin = (id1-1)*0x20+id2;
        g.data = v;
        g.usepullup = 1;
        ioctl(fd, IOCTL_GPIO_SETOUTPUT, &g);
    }

    int Read(int id1,int id2)
    {
        gpio_arg g;
        g.pin = (id1-1)*0x20+id2;
        g.data = 0;
        ioctl(fd,IOCTL_GPIO_SETINPUT,&g);
        ioctl(fd,IOCTL_GPIO_GETVALUE,&g);
        return g.data;
    }

    void run()
    {
        int flag=0;

        while(1)
        {
            usleep(500000);
            flag++;
            if(flag%3==0)  Ctr(1,3,0), Ctr(5,18,0), Ctr(5,19,1);
            else if(flag%3==1)  Ctr(1,3,0), Ctr(5,18,1), Ctr(5,19,0);
            else if(flag%3==2)  Ctr(1,3,1), Ctr(5,18,0), Ctr(5,19,0);

            Ctr(6,31,v_out[0]);  Ctr(7,6,v_out[1]);
            Ctr(7,4,v_out[2]);   Ctr(7,5,v_out[3]);

            v_in[0]=Read(7,8);
            v_in[1]=Read(7,3);
            v_in[2]=Read(7,7);
            v_in[3]=Read(7,0);
            v_in[4]=Read(7,2);
            v_in[5]=Read(7,1);
        }
    }
};

#endif //UGV_WSK_GPIO_H
