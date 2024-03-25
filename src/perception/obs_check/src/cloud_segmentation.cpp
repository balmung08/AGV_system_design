// 将点云转换为前后左右四个区域点云

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "pointcloud_type.h"
#include "common/public.h"
#include "cloud_preprocess.h"

using namespace std;

ros::NodeHandle *nh;

ros::Publisher pub_cloud_front, pub_cloud_back, pub_cloud_left, pub_cloud_right;

float max_xy_range=10, max_z=0.1, min_z=-0.6;  
float agv_width=0.6, agv_length=0.9;  
float lidar_dead_leftright=0.1, lidar_dead_frontback=0.1;

TNodeCheck *nodecheck;

bool CheckInRange(float x, float y, float x_min, float x_max, float y_min, float y_max)
{
    return (x>=x_min && x<=x_max && y>=y_min && y<=y_max);
}

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PointCloud all_cloud;
    pcl::fromROSMsg(*msg, all_cloud);

    PointCloud front_cloud,back_cloud,left_cloud,right_cloud;
    front_cloud.header=all_cloud.header;
    back_cloud.header=all_cloud.header;
    left_cloud.header=all_cloud.header;
    right_cloud.header=all_cloud.header;
    
    for(auto it:all_cloud)
    {
        bool front_flag=CheckInRange(it.x, it.y,  agv_length*0.5+lidar_dead_frontback, max_xy_range, -agv_width*0.5, agv_width*0.5);
        if (front_flag)  
        {
            front_cloud.push_back(it);
            continue;
        }
        bool back_flag=CheckInRange(it.x, it.y,  -max_xy_range, -agv_length*0.5-lidar_dead_frontback, -agv_width*0.5, agv_width*0.5);
        if (back_flag)  
        {
            back_cloud.push_back(it);
            continue;
        }  
        bool left_flag=CheckInRange(it.x, it.y,  -agv_length*0.5, agv_length*0.5, agv_width*0.5+lidar_dead_leftright, max_xy_range);
        if (left_flag)  
        {
            left_cloud.push_back(it);
            continue;
        } 
        bool right_flag=CheckInRange(it.x, it.y,  -agv_length*0.5, agv_length*0.5, -max_xy_range, -agv_width*0.5-lidar_dead_leftright);
        if (right_flag)  
        {
            right_cloud.push_back(it);
        } 
    }

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(front_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud_front.publish(msgx);

    pcl::toROSMsg(back_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp = ros::Time::now();
    pub_cloud_back.publish(msgx);

    pcl::toROSMsg(left_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp =ros::Time::now();
    pub_cloud_left.publish(msgx);

    pcl::toROSMsg(right_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud_right.publish(msgx);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_segmentation");
    nh=new ros::NodeHandle("~");
    nodecheck = new TNodeCheck(nh, "node_rate");

    pub_cloud_front = nh->advertise<sensor_msgs::PointCloud2>("agv_front_cloud", 10);
    pub_cloud_back = nh->advertise<sensor_msgs::PointCloud2>("agv_back_cloud", 10);
    pub_cloud_left = nh->advertise<sensor_msgs::PointCloud2>("agv_left_cloud", 10);
    pub_cloud_right = nh->advertise<sensor_msgs::PointCloud2>("agv_right_cloud", 10);

    ros::Subscriber cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_tf", 10, PointCloudCallback);    
   
    ros::Rate looprate(20);
    while (ros::ok())
    {    
        nodecheck->Find("node_rate")->Beat();

        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}