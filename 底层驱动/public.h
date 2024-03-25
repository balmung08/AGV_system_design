#ifndef ABC_ABC_H
#define ABC_ABC_H

#include <vector>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

// #define rad2deg 57.295780490442
const float rad2deg = 57.295780490442;
using namespace std;

vector<string> split(const string &s, const string &sep);
vector<string> getFileList(char *basePath, char *sep);
double atan_yaw(double y, double x);
void satur(double *x, double th);
void satur(double *x, double lb, double ub);
void print_vecstr(vector<string> vs);

class TKalmanFilter
{
private:
    float x_last, p_last;

public:
    float gain, off;
    float Q, R;
    float value;

    TKalmanFilter()
    {
        x_last = 0, p_last = 10;
        // Q=0.000000175,  R=0.00145;
        Q = 0.02, R = 0.6;
        gain = 1, off = 0;
    }

    float GetValue(float v)
    {
        float x_mid, x_now;
        float p_mid, p_now;
        float kg;

        x_mid = x_last;
        p_mid = p_last + Q;

        kg = p_mid / (p_mid + R);
        x_now = x_mid + kg * (v - x_mid);
        p_now = (1.0 - kg) * p_mid;

        x_last = x_now;
        p_last = p_now;
        value = x_now * gain;
        return value;
    }
};

class TTimer
{
private:
    struct timeval tv1, tv2;
    struct timezone tz;
    float t_off;

public:
    float value;

    TTimer()
    {
        Clear();
        value = 0;
    }

    void Clear(float t = 0)
    {
        gettimeofday(&tv1, &tz);
        tv2 = tv1;
        t_off = t;
    }

    double GetValue(void)
    {
        double ret;
        gettimeofday(&tv2, &tz);
        ret = (tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec) * 0.000001 + t_off;
        value = ret;
        return ret;
    }
};

class TDataFilter
{
private:
    float buf[200];

public:
    int filternum;
    float value;

    TDataFilter()
    {
        filternum = 20;
        for (int i = 0; i < 200; i++)
            buf[i] = 0;
    }

    TDataFilter(int n)
    {
        TDataFilter();
        filternum = n;
    }

    float GetValue(float v)
    {
        int i;
        for (i = 0; i < filternum - 1; i++)
            buf[i] = buf[i + 1];
        buf[filternum - 1] = v;
        value = 0;
        for (i = 0; i < filternum; i++)
            value = value + buf[i];
        value = value / filternum;
        return value;
    }
};

class Thread
{
private:
    // 当前线程的线程ID
    pthread_t tid;
    // 线程的状态
    int threadStatus;
    // 获取执行方法的指针
    static void *thread_proxy_func(void *args)
    {
        Thread *pThread = static_cast<Thread *>(args);
        pThread->run();
        return NULL;
    }

    // 内部执行方法
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
    // 线程的状态－新建
    static const int THREAD_STATUS_NEW = 0;
    // 线程的状态－正在运行
    static const int THREAD_STATUS_RUNNING = 1;
    // 线程的状态－运行结束
    static const int THREAD_STATUS_EXIT = -1;

    // 构造函数
    Thread()
    {
        tid = 0;
        threadStatus = THREAD_STATUS_NEW;
    }

    // 线程的运行实体
    virtual void run() = 0;

    // 开始执行线程
    bool start()
    {
        // int iRet = 0;
        // pthread_create(&tid, NULL, thread_proxy_func, this) == 0;
        return pthread_create(&tid, NULL, thread_proxy_func, this) == 0;
    }

    // 获取线程ID
    pthread_t getThreadID()
    {
        return tid;
    }

    // 获取线程状态
    int getState()
    {
        return threadStatus;
    }

    // 等待线程直至退出
    void join()
    {
        if (tid > 0)
        {
            pthread_join(tid, NULL);
        }
    }

    // 等待线程退出或者超时
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

class limspeed_vessel
{
private:
    double value = 0;
    double vold = 0;
    double limspd = 0;
    TTimer *tmr;

public:
    bool useflag = false;
    limspeed_vessel();
    limspeed_vessel(double lim);
    limspeed_vessel(double val, double lim);
    void begin();
    double getValue(double val);
    void setLim(double lim);
    void reset();
};

#endif // ABC_ABC_H
