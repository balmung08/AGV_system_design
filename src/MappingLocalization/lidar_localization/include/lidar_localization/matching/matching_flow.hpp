/*
 * @Description: matching 模块任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_

#include <ros/ros.h>
#include <deque>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/path_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"
// matching
#include "lidar_localization/matching/matching.hpp"

#include <geometry_msgs/PolygonStamped.h>
#include <fstream>
// #include <yaml-cpp/yaml.h>

namespace lidar_localization {
class MatchingFlow {
  public:
    MatchingFlow(ros::NodeHandle& nh);
    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateMatching();
    bool PublishData();
    bool PublishData(double timeStamp, const Eigen::Matrix4f& pose);
    bool PublishDataFromBase(double timeStamp, Eigen::Matrix4f pose);
    bool FindGpsData(double curTime);
    void LinearInterpolation(const std::deque<PoseData>& pose_buff, PoseData& current_pose, const int& closestFrame_Index, const double& targetTime);
    void Trans_lidar2gps(const Eigen::Matrix4f& pose, Eigen::Matrix4f& respose);
    // bool PointInRect(const Eigen::Vector2f p, const std::vector<Eigen::Vector2f> rect);
    void GPS_or_Odo(const Eigen::Vector2f p);
    bool inInitialArea();
    bool enable_Matching();
    void clearState();
    bool isOdoDrift(bool &OdoAvailable);
    
  private:
    // subscriber 
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<PathPublisher> laser_path_pub_ptr_;
    std::shared_ptr<PathPublisher> fusion_path_pub_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;
    ros::Publisher OdomPub;
    ros::Publisher polygon_pub[10];
    // matching
    std::shared_ptr<Matching> matching_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_data_buff_;

    CloudData current_cloud_data_;
    PoseData current_gnss_data_;
    ros::Time gps_cal_time = ros::Time::now(); 

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f laser_odometry_last_ = Eigen::Matrix4f::Identity();
    std::deque<Eigen::Vector2f> laser_odometry_his_longterm;
    ros::NodeHandle nh_;

    double x_lidar_in_gps;
    double y_lidar_in_gps;
    double angle_lidar_in_gps;
    std::vector<Eigen::Vector2f> rect_garage;
    std::vector<Eigen::Vector2f> rect_chargePort;
    std::vector<geometry_msgs::PolygonStamped> polygons;
    Eigen::Vector3f init_pose;
    Eigen::Matrix4f init_pose_matrix;



    std::string config_file_path;
    YAML::Node config_node;
    bool odoAvailableFlag;
    bool needStableCheck;
    bool needFixInitial;
    
    void initPolygons();
    void savePosetoconfig(Eigen::Matrix4f & pose);
    bool initPosebyconfig();
    bool insideRanges(geometry_msgs::Point32 p);
};
}

#endif