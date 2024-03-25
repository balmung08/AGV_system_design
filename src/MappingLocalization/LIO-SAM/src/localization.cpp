#include "utility.h"
#include "lio_sam/cloud_info.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <queue>
#include <pcl/registration/ndt.h>

class Localization : public ParamServer
{
private:
    ros::Subscriber subLaserCloudInfo_;
    ros::Subscriber subGpsOdometry_;

    ros::Publisher pubGlobalMap_;
    ros::Publisher pubLocalMap_;
    ros::Publisher pubCurrentScan_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalMap_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalMapDS_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr localMap_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputScan_;

    // 体素网格滤波器
    pcl::VoxelGrid<pcl::PointXYZ> downSizeGlobalMap_;

    // NDT配准器
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    Eigen::Matrix4f initial_pose_;
    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f prediect_pose_;
    Eigen::Matrix4f step_pose_;

    bool initial_;

    std::mutex gpsMutex_;
    std::queue<nav_msgs::Odometry> gpsQueue_;

public:
    // 构造函数
    Localization()
    {
        subLaserCloudInfo_ = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 5, &Localization::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subGpsOdometry_ = nh.subscribe<nav_msgs::Odometry>("/gps/odometry", 5, &Localization::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        pubGlobalMap_ = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/localization/global_map", 1);
        pubLocalMap_ = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/localization/local_map", 1);
        pubCurrentScan_ = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/localization/current_scan", 1);

        // 分配内存
        // allocateMemory();

        // 发布全局地图(只发布一次)
        publishGlobalMap();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    // ~Localization();

    // 分配内存
    void allocateMemory()
    {
        globalMap_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        globalMapDS_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        localMap_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        currentScan_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        outputScan_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        initial_pose_ = Eigen::Matrix4f::Identity();
        current_pose_ = Eigen::Matrix4f::Identity();
        prediect_pose_ = Eigen::Matrix4f::Identity();
        step_pose_ = Eigen::Matrix4f::Identity();

        ndt_.setTransformationEpsilon(0.01); // 为终止条件设置最小转换差异
        ndt_.setStepSize(0.1);
        ndt_.setResolution(1.0);
        ndt_.setMaximumIterations(35);
    }
    
    // publish global map
    void publishGlobalMap()
    {
        ROS_INFO("Loading global map...");
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *globalMap_) == -1)
        {
            ROS_ERROR("The map path is wrong,please configure the map path!");
            ros::shutdown();
        }
        downSizeGlobalMap_.setInputCloud(globalMap_);
        downSizeGlobalMap_.filter(*globalMapDS_);

        // 将点云存入目标点云中
        ndt_.setInputTarget(globalMap_);

        // publish global map
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*globalMapDS_, tempCloud);
        tempCloud.header.stamp = ros::Time::now();
        tempCloud.header.frame_id = "map";
        pubGlobalMap_.publish(tempCloud);

    }

    // gps回调函数
    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        nav_msgs::Odometry thisGps = *gpsMsg;
        gpsMutex_.lock();
        gpsQueue_.emplace(thisGps);
        gpsMutex_.unlock();
    }

    // cloud回调函数
    void cloudHandler(const lio_sam::cloud_infoConstPtr& laserCloudMsg)
    {
       lio_sam::cloud_info thisCloud = *laserCloudMsg;
        sensor_msgs::PointCloud2 thisCloud_Deskewed = thisCloud.cloud_deskewed;

       // 得到畸变校正过后的点云
       pcl::fromROSMsg( thisCloud_Deskewed, *currentScan_);

       if(!initial_)
       {
            if(!SetInitialPose())
                return;
            initial_ = true;
       }

        // gps目前只做初始化,初始化完即清空
       void ClearGpsQueue();

        // 不断更新
        void Update();

    }

    bool SetInitialPose()
    {
        if(gpsQueue_.empty())
            return false;
        nav_msgs::Odometry gpsOdom = gpsQueue_.front();

        initial_pose_(0, 3) = static_cast<float>(gpsOdom.pose.pose.position.x);
        initial_pose_(1, 3) = static_cast<float>(gpsOdom.pose.pose.position.y);
        initial_pose_(2, 3) = static_cast<float>(gpsOdom.pose.pose.position.z);

        // 进行NDT匹配，查看是否收敛
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        ndt_.setInputSource(currentScan_);
        ndt_.align(*output_cloud, initial_pose_);

        if(!ndt_.hasConverged())
        {
            ROS_WARN("Fail to initiial... return");
            
            return false;
        }

        // 得到当前帧坐标
        pcl::transformPointCloud(*currentScan_, *output_cloud, ndt_.getFinalTransformation());
        prediect_pose_ = current_pose_;

        ROS_DEBUG("Initial successfully ! The ndt score is %.5f", ndt_.getFitnessScore());

        return true;        
    }

    void Update()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_1(new pcl::PointCloud<pcl::PointXYZ>());
        ndt_.setInputSource(currentScan_);
        ndt_.align(*output_cloud_1, prediect_pose_);

        pcl::transformPointCloud(*currentScan_, *output_cloud_1, ndt_.getFinalTransformation());
    }

     void ClearGpsQueue()
     {
        gpsMutex_.lock();

        while(!gpsQueue_.empty())
            gpsQueue_.pop();

        gpsMutex_.unlock();
     }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lio_sam_localization");

    Localization Lo;

    ROS_INFO("\033[1;32m----> Localization Started.\033[0m");
    
    ros::spin();
    
    return 0;
}
