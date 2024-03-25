#ifndef TEST1_H
#define TEST1_H

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <termio.h>
#include <stdio.h>
#include <deque>
#include "common/public.h"

using namespace std;

#define MAX_DATA_LENTH 1000

template <class T1, class T2>
class TSub
{
public:
    TSub(ros::NodeHandle &nh, string topic_name, size_t buff_size = 10);
    void GetData(deque<T1> &data);
    bool GetData(T1 &data);
    float GetHZ();
    void run();

private:
    void msg_callback(const T2 &msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    THeartbeat hb;

    deque<T1> data_;
};

class TUInt16_Sub : public TSub<std_msgs::UInt16, std_msgs::UInt16ConstPtr>
{
public:
    TUInt16_Sub(ros::NodeHandle &nh, string topic_name, size_t buff_size = 10);
};

class TString_Sub : public TSub<std_msgs::String, std_msgs::StringConstPtr>
{
public:
    TString_Sub(ros::NodeHandle &nh, string topic_name, size_t buff_size = 10);
};

class TPointCloud_Sub : public TSub<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2::ConstPtr>
{
public:
    TPointCloud_Sub(ros::NodeHandle &nh, string topic_name, size_t buff_size = 10);
};

class TPointStamped_Sub : public TSub<geometry_msgs::PointStamped, geometry_msgs::PointStamped::ConstPtr>
{
public:
    TPointStamped_Sub(ros::NodeHandle &nh, string topic_name, size_t buff_size = 10)
    : TSub(nh, topic_name, buff_size)
    {}
};

template <class T1, class T2>
TSub<T1, T2>::TSub(ros::NodeHandle &nh, string topic_name, size_t buff_size)
{
    nh_ = nh;
    sub_ = nh_.subscribe<T1>(topic_name, buff_size, &TSub::msg_callback, this);
    // start();
};

template <class T1, class T2>
void TSub<T1, T2>::GetData(deque<T1> &data)
{
    if (data_.size() > 0)
    {
        data.insert(data.end(), data_.begin(), data_.end());
        data_.clear();
    }
}

template <class T1, class T2>
bool TSub<T1, T2>::GetData(T1 &data)
{
    if (data_.size() > 0)
    {
        data = *(data_.end() - 1);
        data_.clear();
        return true;
    }
    else
        return false;
}

template <class T1, class T2>
void TSub<T1, T2>::msg_callback(const T2 &msg)
{
    data_.push_back(*msg);
    if (data_.size() > MAX_DATA_LENTH)  data_.pop_front();
    hb.Beat();
}

#endif