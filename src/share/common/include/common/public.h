#ifndef PUBLIC_H
#define PUBLIC_H

#include <vector>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <sys/io.h>
#include <memory.h>
#include <dirent.h>
#include <math.h>

#include <stdio.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#define Rad2Deg 180 / 3.1415926
#define Deg2Rad 3.1415926 / 180

//#define rad2deg 57.295780490442
const float rad2deg = 57.295780490442;

using namespace std;

struct LS_Coeff //最小二乘法二次多项式的三个系数
{
    double a0, a1, a2;
};

void SetBit(unsigned int &value, int bit, int bit_v);
void SetErr(unsigned int &value, int bit, int bit_v);

vector<string> getFileList(char *basePath, char *sep);
void GetPackagePath(char *packname, char *path);
vector<string> split(const string &s, const string &sep);
LS_Coeff Least_Square(vector<geometry_msgs::Point> point);
vector<float> FittingCurve(vector<float> XData, vector<float> YData, int m);
uint16_t CRC16(uint8_t *ptr, uint32_t len);

vector<string> readFileList(char *basePath);
int FindMinID(vector<float> datalist);
string NowtimetoString();

double P2P(double x1, double y1, double x2, double y2);
float P2P(geometry_msgs::Point p1, geometry_msgs::Point p2);

#define HIG_UINT16(a) (((a) >> 8) & 0xFF)
#define LOW_UINT16(a) ((a)&0xFF)
#define HIG_UINT8(a) (((a) >> 4) & 0x0F)
#define LOW_UINT8(a) ((a)&0x0F)

uint16_t crctalbeabs[] =
    {
        0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
        0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400};

uint16_t CRC16(uint8_t *ptr, uint32_t len);

class TTimer
{
private:
    struct timeval tv1, tv2;
    struct timezone tz;
    float t_off;

public:
    float value;

    TTimer();
    void Clear(float t = 0);
    double GetValue(void);
};


class TDataFilter
{
private:
    vector<double> buf;
    int filternum;

public:
    float value;

    TDataFilter(int n);
    double GetValue(double v);
    void Clear();
};

class Thread
{
private:
    //当前线程的线程ID
    pthread_t tid;
    //线程的状态
    int threadStatus;
    //获取执行方法的指针
    static void *thread_proxy_func(void *args)
    {
        Thread *pThread = static_cast<Thread *>(args);
        pThread->run();
        return NULL;
    }

    //内部执行方法
    void *run1()
    {
        threadStatus = THREAD_STATUS_RUNNING;
        tid = pthread_self();
        run();
        threadStatus = THREAD_STATUS_EXIT;
        tid = 0;
        pthread_exit(NULL);
    }

public:
    //线程的状态－新建
    static const int THREAD_STATUS_NEW = 0;
    //线程的状态－正在运行
    static const int THREAD_STATUS_RUNNING = 1;
    //线程的状态－运行结束
    static const int THREAD_STATUS_EXIT = -1;

    //构造函数
    Thread()
    {
        tid = 0;
        threadStatus = THREAD_STATUS_NEW;
    }

    //线程的运行实体
    virtual void run() = 0;

    //开始执行线程
    bool start()
    {
        int iRet = 0;
        pthread_create(&tid, NULL, thread_proxy_func, this) == 0;
        return true;
    }

    //获取线程ID
    pthread_t getThreadID()
    {
        return tid;
    }

    //获取线程状态
    int getState()
    {
        return threadStatus;
    }

    //等待线程直至退出
    void join()
    {
        if (tid > 0)
        {
            pthread_join(tid, NULL);
        }
    }

    //等待线程退出或者超时
    void join(unsigned long millisTime)
    {
        if (tid == 0)
        {
            return;
        }
        if (millisTime == 0)
        {
            join();
        }
        else
        {
            unsigned long k = 0;
            while (threadStatus != THREAD_STATUS_EXIT && k <= millisTime)
            {
                usleep(100);
                k++;
            }
        }
    }
};

class THeartbeat : public Thread
{
private:
    float interval = 1.0;
    int mycounter = 0;

public:
    float value = 0;

    THeartbeat(float dt = 1.0);
    void Beat(int v = 1);
    void run();
};

class THeartbeat_ : public Thread
{
private:
    float interval = 1.0;
    int mycounter = 0;
    float min_value = 0.5, max_value = 1000.0;

public:
    float value = 0;
    string caption;
    ros::NodeHandle *nh = NULL;
    string err_msg = "";
    bool ok_flag = true;

    THeartbeat_(string s = "", float dt = 1.0);
    THeartbeat_(ros::NodeHandle *n, string c, float dt = 1.0);

    void Beat(int v = 1);
    void SetLimit(float min, float max = 1000);
    void run();
};

class TNodeCheck : public Thread
{
private:
    ros::NodeHandle *nh;
    vector<THeartbeat_ *> heartbeat_array;
    float check_dt;

public:
    int seq = 0;
    TNodeCheck(ros::NodeHandle *n, string caps, float dt = 1);
    THeartbeat_ *Find(string caption);
    void run();
};

#endif // PUBLIC_H
