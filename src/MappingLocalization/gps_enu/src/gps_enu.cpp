#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <gps_enu/MyGPS_msg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <mutex>
#include <tf/tf.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <fstream>

using std::cout;
using  std::endl;
using namespace Eigen;

class CoordinateTrans
{
private:
    ros::NodeHandle nh_, nh_local_;

    ros::Subscriber subGps_;
    ros::Subscriber subImu_;
    ros::Subscriber subGPSBase_, subSlamPath_;

    ros::Publisher pubGpsPath_;
    ros::Publisher pubGpsOdmetry_;

    GeographicLib::LocalCartesian geo_converter_;
    bool initial_;

    sensor_msgs::NavSatFix currentGpsMsg_;
    gps_enu::MyGPS_msg currentGpsBaseMsg_;
    double timeGpsCur_; // 当前帧GPS的时间戳

    float transformPos_[6];

    double longitude_;
    double latitude_;
    double altitude_;
    double local_E_;
    double local_N_;
    double local_U_;
    double angle_bias_ = 0, x_bias_ = 0, y_bias_ = 0;
    geometry_msgs::Quaternion current_q_;  

public:
    // 构造函数
    CoordinateTrans():initial_(false)
    {
        nh_local_ = ros::NodeHandle("~");
        subGps_ = nh_.subscribe<sensor_msgs::NavSatFix>("/gps_base/GPS_fix", 10, &CoordinateTrans::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subGPSBase_ = nh_.subscribe<gps_enu::MyGPS_msg>("/gps_base/GPS_Base", 10, &CoordinateTrans::gpsbaseHander, this, ros::TransportHints().tcpNoDelay());
        subSlamPath_ = nh_.subscribe<nav_msgs::Path>("/lio_sam/imu/path", 10, &CoordinateTrans::SlamPathHandler, this, ros::TransportHints().tcpNoDelay());
        
        pubGpsPath_ = nh_.advertise<nav_msgs::Path>("/gps/path", 10);
        pubGpsOdmetry_ = nh_.advertise<nav_msgs::Odometry>("/gps/odometry", 10);

        nh_local_.getParam("angle_bias", angle_bias_);
        nh_local_.getParam("x_bias", x_bias_);
        nh_local_.getParam("y_bias", y_bias_);

        // ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
    }
    
    void gpsbaseHander(const gps_enu::MyGPS_msg::ConstPtr& gpsbase_msg){
        currentGpsBaseMsg_ = *gpsbase_msg;
        timeGpsCur_ = currentGpsBaseMsg_.header.stamp.toSec();
        currentGpsBaseMsg_.header.frame_id = "map";
        if(currentGpsBaseMsg_.Quality < 4){
            cout << "The GPS quality is bad !!! " << endl;
            return;
        }
        AlignToTrajectory();
        PublishData();
    }

    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
    {
    }
    
    void SlamPathHandler(const nav_msgs::Path::ConstPtr& msg)
    {
        geometry_msgs::Pose p=msg->poses.back().pose;
        float dx=p.position.x-transformPos_[3],  dy=p.position.y-transformPos_[4];
        // ROS_INFO("err=%.2f %.2f", dx, dy);
    }

    // 初始化原点
    void initialOriginPosition()
    {
        longitude_ = currentGpsMsg_.longitude;
        latitude_ = currentGpsMsg_.latitude;
        altitude_ = currentGpsMsg_.altitude;
        longitude_ = 121.37118953333332;
        latitude_ = 37.561962386666664;
        altitude_ =  5.836;
        // 固定原点
        geo_converter_.Reset(latitude_, longitude_, altitude_);
        initial_ = true;
    }

    // 转换至ENU坐标下
    void TransformData()
    {
        longitude_ = currentGpsMsg_.longitude;
        latitude_ = currentGpsMsg_.latitude;
        altitude_ = currentGpsMsg_.altitude;
        geo_converter_.Forward(latitude_, longitude_, altitude_, local_E_, local_N_, local_U_);
    }

    // 弧度转角度
    double RadianToAngle(double radian)
    {
        return radian*180/M_PI;
    }

    // 角度转弧度
    double AngleToRadian(double angle)
    {
        return angle * M_PI/180;
    }

    // 对齐轨迹
    void AlignToTrajectory()
    {
        double angle = currentGpsBaseMsg_.Angle;
        // 转换为与东的夹角
        // if(angle > 180)     angle -= 180;
        // else if(angle < 180)    angle += 180;
        // angle = 90 - angle;
        // if(angle > 180)     angle = 360-angle;
        // else if(angle < -180)   angle += 360;

        current_q_ = tf::createQuaternionMsgFromYaw((angle + angle_bias_) / 180 * M_PI);
        double map_x = currentGpsBaseMsg_.map_x;
        double map_y = currentGpsBaseMsg_.map_y;
        double angle_ = (angle)  / 180 * M_PI;

        Eigen::Affine3f T_WG = pcl::getTransformation(map_x, map_y, 0, 0, 0, angle_);
        Eigen::Affine3f T_GL = pcl::getTransformation(x_bias_, y_bias_, 0, 0, 0, angle_bias_/180 * M_PI); //*NIU 2.375 左转偏左 0.025 10车最优：2.25 0.007 -3.24
        Eigen::Affine3f T_WL =  T_WG * T_GL;

        pcl::getTranslationAndEulerAngles(T_WL, transformPos_[3], transformPos_[4], transformPos_[5],
                                          transformPos_[0], transformPos_[1], transformPos_[2]);
        // 存储位姿
        Eigen::Quaternionf q_odom;
        q_odom.x() = current_q_.x;
        q_odom.y() = current_q_.y;
        q_odom.z() = current_q_.z;
        q_odom.w() = current_q_.w;
        Eigen::Matrix3f odom(q_odom);
        Eigen::Matrix4f gpsPose = T_WL.matrix();
        gpsPose.block<3, 3>(0, 0) = odom;
    }

    // 发布数据
    void PublishData()
    {
        // publish odometry
        nav_msgs::Odometry gpsOdom;
        gpsOdom.header.stamp = currentGpsBaseMsg_.header.stamp;
        gpsOdom.header.frame_id = currentGpsBaseMsg_.header.frame_id;
        gpsOdom.pose.pose.position.x = transformPos_[3];
        gpsOdom.pose.pose.position.y = transformPos_[4];
        gpsOdom.pose.pose.position.z = 0;
        gpsOdom.pose.pose.orientation = current_q_;
        gpsOdom.pose.covariance[0] = transformPos_[2]; //*NIU yaw角
        gpsOdom.pose.covariance[1] = currentGpsBaseMsg_.Lat; //*NIU 下面的代码负责保存所有原gpsbase信息
        gpsOdom.pose.covariance[2] = currentGpsBaseMsg_.Lon;
        gpsOdom.pose.covariance[3] = currentGpsBaseMsg_.raw_Angle;
        gpsOdom.pose.covariance[4] = currentGpsBaseMsg_.Angle;
        gpsOdom.pose.covariance[5] = currentGpsBaseMsg_.raw_map_x;
        gpsOdom.pose.covariance[6] = currentGpsBaseMsg_.raw_map_y;
        gpsOdom.pose.covariance[7] = currentGpsBaseMsg_.map_x;
        gpsOdom.pose.covariance[8] = currentGpsBaseMsg_.map_y;
        gpsOdom.pose.covariance[9] = currentGpsBaseMsg_.raw_UTM_X;
        gpsOdom.pose.covariance[10] = currentGpsBaseMsg_.raw_UTM_Y;
        gpsOdom.pose.covariance[11] = currentGpsBaseMsg_.UTM_X;
        gpsOdom.pose.covariance[12] = currentGpsBaseMsg_.UTM_Y;
        gpsOdom.pose.covariance[13] = currentGpsBaseMsg_.Vel;
        gpsOdom.pose.covariance[14] = currentGpsBaseMsg_.Quality;
        gpsOdom.pose.covariance[15] = currentGpsBaseMsg_.HeartBeat;
        gpsOdom.pose.covariance[16] = currentGpsBaseMsg_.mqtt_angle;
        if(pubGpsOdmetry_.getNumSubscribers() != 0){
            pubGpsOdmetry_.publish(gpsOdom);
        }

        static nav_msgs::Path gpsPath;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = currentGpsBaseMsg_.header.stamp;
        pose_stamped.header.frame_id = currentGpsBaseMsg_.header.frame_id;
        pose_stamped.pose.position.x = transformPos_[3];
        pose_stamped.pose.position.y = transformPos_[4];
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation = current_q_;

        if(pubGpsPath_.getNumSubscribers() != 0){
            gpsPath.header.stamp = currentGpsBaseMsg_.header.stamp;
            gpsPath.header.frame_id = currentGpsBaseMsg_.header.frame_id;
            gpsPath.poses.push_back(pose_stamped);
            pubGpsPath_.publish(gpsPath);
        }
    }
};

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "gps_enu");

    CoordinateTrans CT;

    ROS_INFO("\033[1;32m---> ENU Coordinate Transform Started. \033[0m");

    //ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
