/*
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2023-10-03 09:34:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2023-10-16 19:44:53
 * @FilePath: /ros_ws/src/sensor/h264_server/src/push_acc_ffmpeg.cpp
 * @Description:
 *
 * Copyright (c) 2023 by RemnantCloude, All Rights Reserved.
 */

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <csignal>

using namespace std;

pid_t getProcessPidByName(const string &pid_name)
{
    FILE *fp;
    char buf[100];
    pid_t pid;
    if ((fp = popen(("pidof " + pid_name).c_str(), "r")) != NULL)
    {
        if (fgets(buf, sizeof(buf), fp) != NULL)
        {
            pid = atoi(buf);
        }
    }
    else
        return -1;

    pclose(fp);
    return pid;
}

int executeCMD(const string &cmd, pid_t &pid)
{
    FILE *ptr = NULL;
    int iRet = -1;

    // popen: 开启子进程，建立管道，并运行指令，'r':从子进程获取结果，'w':向子进程写数据
    if ((ptr = popen(cmd.c_str(), "r")) != NULL) // popen
    {
        pid = getProcessPidByName("ffmpeg");

        printf("Process ID: %d\n", pid);

        iRet = 0; // 处理成功
    }
    else
    {
        printf("popen %s error\n", cmd.c_str());
        iRet = -1; // 处理失败
    }

    return iRet;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "push_acc_ffmpeg");
    ros::NodeHandle *nh = new ros::NodeHandle("~");

    bool push_flag = false;
    bool push_status = false;
    
    string command;
    pid_t pid;

    nh->getParam("command", command);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        nh->getParam("push_flag", push_flag);
        if (push_flag ^ push_status) // 状态不同
        {
            if (push_flag)
            {
                ROS_INFO("Start push %s.", nh->getNamespace().c_str());
                executeCMD(command, pid);
            }
            else
            {
                ROS_INFO("Stop push %s.", nh->getNamespace().c_str());
                kill(pid, SIGKILL);
            }
            push_status = push_flag;
        }

        loop_rate.sleep();
    }

    if (pid != -1)
    {
        ROS_INFO("Stop push %s.", nh->getNamespace().c_str());
        kill(pid, SIGKILL);
    }
    delete nh;

    return 0;
}