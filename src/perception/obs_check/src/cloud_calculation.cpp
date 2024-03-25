//本节点输入为前后左右四路激光点云,输出前后左右障碍物最近距离

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "std_msgs/Float32.h"
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
float agv_width=0.6, agv_length=0.9;
float obs_dis=999;

string dir="front";

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    obs_dis=999;
    PointCloud cloud;
    pcl::fromROSMsg(*c_msg, cloud);

    // int filter_num=10;
    // nh->getParam("filter_num", filter_num);
    // if(filter_num>1)  cloud=Cloud_FastFilter(cloud, filter_num, 0.2);

    if(cloud.size()<10)  return; 

    PointType cloud_min, cloud_max;
    pcl::getMinMax3D(cloud, cloud_min, cloud_max);
    if(dir=="front")  obs_dis=fabs(cloud_min.x)-0.5*agv_length;
    else if(dir=="back")  obs_dis=fabs(cloud_max.x)-0.5*agv_length;
    else if(dir=="left")  obs_dis=fabs(cloud_min.y)-0.5*agv_width;
    else  obs_dis=fabs(cloud_max.y)-0.5*agv_width;

    // if(dir=="front")  ROS_INFO("%.2f", obs_dis);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_calculation");
    nh=new ros::NodeHandle("~");
    
    nh->getParam("dir", dir);

    TNodeCheck *nodecheck=new TNodeCheck(nh, "node_rate");

    ros::Subscriber cloud_sub;
    if(dir=="front")  cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_front_cloud", 10, &PointCloudCallback);
    else if(dir=="back")  cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_back_cloud", 10, &PointCloudCallback);
    else if(dir=="left")  cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_left_cloud", 10, &PointCloudCallback);
    else cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_right_cloud", 10, &PointCloudCallback); 
    
    ros::Publisher pub_obs_dis = nh->advertise<std_msgs::Float32>("obs_dis", 10);
    TDataFilter f_obs_dis(3);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();
 
        obs_dis=f_obs_dis.GetValue(obs_dis);
        std_msgs::Float32 msg;
        msg.data=obs_dis;
        pub_obs_dis.publish(msg);
        
        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}