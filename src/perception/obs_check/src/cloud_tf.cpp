//  将原始点云转化为base_link坐标下的点云，前提是要建立好tf关系

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "pointcloud_type.h"
#include "common/public.h"

#include "cloud_preprocess.h"

using namespace std;

ros::Publisher pub_cloud,pub_cloud_filter;  

float max_xy_range=10; 
float max_z=0.2, min_z=-0.4; 

TNodeCheck *nodecheck;
string topicname;

ros::NodeHandle *nh;

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PointCloud cloud;    
    pcl::fromROSMsg(*msg, cloud);

    cloud = remove_infinite_points(cloud);
    cloud = Cloud_PassThrough(cloud, -max_xy_range, max_xy_range, -max_xy_range, max_xy_range, min_z, max_z);
    cloud= CloudTf(cloud,"base_link");
    // cloud=Cloud_FastFilter(cloud, 10, 0.2);
    sensor_msgs::PointCloud2 msgx;

    pcl::toROSMsg(cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud.publish(msgx);

    nodecheck->Find("rslidar_rate")->Beat();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_tf");
    nh=new ros::NodeHandle("~");

    nodecheck = new TNodeCheck(nh, "node_rate rslidar_rate");

    ros::Subscriber cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, PointCloudCallback);
    pub_cloud = nh->advertise<sensor_msgs::PointCloud2>("/cloud_tf", 10);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();
                       
        ros::spinOnce();
        looprate.sleep();
    }

    ros::shutdown();
    return 0;
};
