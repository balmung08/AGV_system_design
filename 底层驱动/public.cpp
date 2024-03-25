#include <iostream>
#include <sys/io.h>
#include <unistd.h>
#include <string>
#include <memory.h>
#include <vector>
#include <dirent.h>
#include "public.h"

using namespace std;

// 函数作用：将字符串按照特定字符分割成片段，
// s：字符串指针，sep:字符串中的分隔字符
vector<string> split(const string &s, const string &sep)
{
    // const限定一个变量不允许被改变，产生静态作用
    // string&现有变量起个别名
    // 传回数组= Split(原始字串, 要找的字串, 拆成几个数组)
    // vector是一个能够存放任意类型的动态数组，能够增加和压缩数据
    vector<string> v;
    string::size_type pos1, pos2;
    pos2 = s.find(sep);
    pos1 = 0;
    while (string::npos != pos2) // 查找字符串s是否包含子串sep
    {
        v.push_back(s.substr(pos1, pos2 - pos1)); // vector尾部加入一个数据
        // s.substr(pos1, pos2-pos1)pos1字符串起始位置，pos2-pos1复制的字符数
        pos1 = pos2 + sep.size();
        pos2 = s.find(sep, pos1); // 从pos开始查找字符sep在当前字符串的位置
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
}

vector<string> getFileList(char *basePath, char *sep)
{
    DIR *dir;
    struct dirent *ptr;
    vector<string> filelist;

    if ((dir = opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
    }

    filelist.clear();
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) /// current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) /// file
        {
            vector<string> ss = split(ptr->d_name, ".");
            if (ss.size() > 1 && ss[ss.size() - 1] == sep)
                filelist.push_back(ss[0]);
        }
        else if (ptr->d_type == 10) // link file
        {
            // printf("d_name:%s/%s\n",basePath,ptr->d_name);
        }
        else if (ptr->d_type == 4) /// dir
        {
        }
    }
    closedir(dir);

    return filelist;
}

double atan_yaw(double y, double x)
{
    double a;
    double pi = M_PI;
    if (fabs(x)<1e-5 && y > 0)
    {
        a = pi / 2;
    }
    else if (fabs(x)<1e-5 && y < 0)
    {
        a = -pi / 2;
    }
    else if (fabs(x)<1e-5 && fabs(y)<1e-5)
    {
        a = 0;
    }
    else
    {
        a = atan(y / x);
    }
    if (x < 0 && y > 0)
    {
        a += pi;
    }
    else if (x < 0 && y < 0)
    {
        a -= pi;
    }
    else if (x < 0 && fabs(y)<1e-5)
    {
        a = pi;
    }
    return a;
}

void satur(double *x, double th)
{
    if (*x > th)
        *x = th;
    if (*x < -th)
        *x = -th;
}

void satur(double *x, double lb, double ub)
{
    if (lb >= ub)
        return;
    if (*x > ub)
        *x = ub;
    if (*x < lb)
        *x = lb;
}

void print_vecstr(vector<string> vs)
{
    if (vs.size() == 0)
        return;
    for (int i = 0; i < vs.size(); i++)
    {
        cout << vs[i] << " \t";
    }
    cout << endl;
}

limspeed_vessel::limspeed_vessel()
{
    tmr = new TTimer();
    useflag = false;
    tmr->Clear();
}

limspeed_vessel::limspeed_vessel(double val, double lim)
{
    tmr = new TTimer();
    value = val;
    limspd = lim;
    useflag = false;
    tmr->Clear();
}

limspeed_vessel::limspeed_vessel(double lim)
{
    tmr = new TTimer();
    limspd = lim;
    useflag = false;
    tmr->Clear();
}

void limspeed_vessel::begin()
{
    if (!useflag)
    {
        useflag = true;
        tmr->Clear();
    }
}

double limspeed_vessel::getValue(double val)
{
    vold = value;
    value = val;
    if (fabs(value - vold) < 1e-10)
        return value;
    if (fabs(value - vold) > tmr->GetValue() * limspd)
        value = vold + tmr->GetValue() * limspd * (value - vold) / fabs(value - vold);
    tmr->Clear();
    return value;
}

void limspeed_vessel::setLim(double lim)
{
    limspd = lim;
    tmr->Clear();
}

void limspeed_vessel::reset()
{
    value = 0;
    vold = 0;
    useflag = false;
    tmr->Clear();
}