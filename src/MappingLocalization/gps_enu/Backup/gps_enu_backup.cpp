#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
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

    ros::Publisher pubGpsPath_;
    ros::Publisher pubGpsOdmetry_;

    GeographicLib::LocalCartesian geo_converter_;
    bool initial_;

    sensor_msgs::NavSatFix currentGpsMsg_;
    double timeGpsCur_; // 当前帧GPS的时间戳

    float transformPos_[6];

    double longitude_;
    double latitude_;
    double altitude_;
    double local_E_;
    double local_N_;
    double local_U_;
    double angle_bias = 0, x_bias = 0, y_bias = 0;
    geometry_msgs::Quaternion current_q_; 

    // imu队列互斥锁
    std::mutex imuLock_;    
    sensor_msgs::Imu currentImuMsg_;
    std::queue<sensor_msgs::Imu> imuQueue_;
    bool has_imuData_;

    // 记录gps轨迹
    std::ofstream ofs_;
    std::string pose_path = "/home/apple/YSW/Car/GPS_ENU_ws/src/gps_enu/pose_data/gpsPose.txt";

public:
    // 构造函数
    CoordinateTrans():initial_(false)
    {
        nh_local_ = ros::NodeHandle("~");
        subGps_ = nh_.subscribe<sensor_msgs::NavSatFix>("/gps_base/GPS_fix", 10, &CoordinateTrans::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        // subImu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 2000, &CoordinateTrans::imuHandler, this, ros::TransportHints().tcpNoDelay());
        pubGpsPath_ = nh_.advertise<nav_msgs::Path>("/gps/path", 10);
        pubGpsOdmetry_ = nh_.advertise<nav_msgs::Odometry>("/gps/odometry", 10);

        ofs_.open(pose_path, std::ios::out);
        nh_local_.getParam("angle_bias", angle_bias);
        nh_local_.getParam("x_bias", x_bias);
        nh_local_.getParam("y_bias", y_bias);
        std::cout << angle_bias << " " << x_bias << " " << y_bias << std::endl;
    }
    
    // gps回调函数
    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
    {
        currentGpsMsg_ = *gpsMsg;
        timeGpsCur_ = currentGpsMsg_.header.stamp.toSec();

        if(currentGpsMsg_.status.status < 4)
        {
            cout << "The GPS quality is bad !!! " << endl;
            return;
        }
        
        if(!initial_)
            initialOriginPosition();

        TransformData();
        
        // 对齐GPS时间戳的imu数据
        // if(!FindImuData())
        // {
        //     ROS_INFO("Not found the imu data, return! ");
        //     return;
        // }

        // if(has_imuData_)
        //     PublishData();        

        AlignToTrajectory();

        PublishData();

    }

    // imu回调函数
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {   
        sensor_msgs::Imu thisImu = *imuMsg;

        imuLock_.lock();
        imuQueue_.push(thisImu);
        imuLock_.unlock();
    }

    // 初始化原点
    void initialOriginPosition()
    {
        longitude_ = currentGpsMsg_.longitude;
        latitude_ = currentGpsMsg_.latitude;
        altitude_ = currentGpsMsg_.altitude;

        // longitude_ = 116.304161412;
        // latitude_ = 39.9636500172;
        // altitude_ = 52.477;
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

        // cout << "local_E_ : " << local_E_ << endl;
        // cout << "local_N_ : " << local_N_ << endl;
        // cout << "local_U_ : " << local_U_ << endl;
    }

    bool FindImuData()
    {
        imuLock_.lock();

        // 若无imu数据或者首个imu时间晚于gps时间，则跳到下一帧GPS信息
        if(imuQueue_.empty() || imuQueue_.front().header.stamp.toSec() > timeGpsCur_)
        {
            ROS_DEBUG("Wating for IMU data ...");
            return false;
        }

        imuAlignGps();

        imuLock_.unlock();

        return true;
    }

    // 寻找IMU数据
    void imuAlignGps()
    {
        while(!imuQueue_.empty())
        {   
            // 取当前GPS后一帧imu为当前姿态
            if(imuQueue_.front().header.stamp.toSec() < timeGpsCur_ - 0.01)
                imuQueue_.pop();
            else
                break;
        }

        if(imuQueue_.empty())
        {
            ROS_DEBUG("The imu data is empty !");
            has_imuData_ = false;
            return;
        }

        currentImuMsg_ = imuQueue_.front();
        has_imuData_ = true;

        // 打印imu对应的欧拉角
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(currentImuMsg_.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        double roll_angle = RadianToAngle(roll);
        double pitch_angle = RadianToAngle(pitch);
        double yaw_angle = RadianToAngle(yaw);

        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << roll_angle << ", pitch: " << pitch_angle << ", yaw: " << yaw_angle << endl << endl;
        // cout <<"imuQueue_.size() : "<< imuQueue_.size() << endl;
       
        // 磁偏角补偿
        yaw_angle -=  100.5; // 100.5
        // cout << "补偿后的yaw角 : " << yaw_angle << endl;
        double roll_radian = AngleToRadian(roll_angle);
        double pitch_radian = AngleToRadian(pitch_angle);
        double yaw_radian = AngleToRadian(yaw_angle);
        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(roll_radian, pitch_radian, yaw_radian);

        currentImuMsg_.orientation = q;

        // ROS_INFO("The imu data is found!");
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
        // AngleAxisd rotation_gl( -50.5*M_PI/180, Vector3d(0, 0, 1));
        // Vector3d translantion_gl(0.16, 0.29, 0);
        // Isometry3d T_gl = Isometry3d::Identity();
        // T_gl.rotate(rotation_gl);
        // T_gl.pretranslate(translantion_gl);

        // AngleAxisd rotation_wg(angle*M_PI/180, Vector3d(0, 0, 1));
        //  Vector3d translantion_wg(Vector3d(local_E_, local_N_, 0));
        // Isometry3d T_wg = Isometry3d::Identity();
        // T_wg.rotate(rotation_wg);
        // T_wg.pretranslate(translantion_wg);

        // Isometry3d T_wl = T_wg * T_gl;

        // double gpsOdom_x = T_wl(0, 3);
        // double gpsOdom_y = T_wl(1, 3);
        // Eigen::Quaterniond quaterion(T_wg.rotation());

        // geometry_msgs::Quaternion q_w;
        // q_w.x = quaterion.x();
        // q_w.y = quaterion.y();
        // q_w.z = quaterion.z();
        // q_w.w = quaterion.w();

        double angle = currentGpsMsg_.position_covariance[0];
        // cout << "west_angle = " <<  angle << endl;  // angle : 0 ~ 360 西向为起始方向，顺时针为正

        // 转换为与东的夹角
        if(angle > 180)     angle -= 180;
        else if(angle < 180)    angle += 180;
        angle = 90 - angle;
        if(angle > 180)     angle = 360-angle;
        else if(angle < -180)   angle += 360;

        // angle_bias = -3.24; //*NIU 10车最优：-3.24


        current_q_ = tf::createQuaternionMsgFromYaw((angle + angle_bias) / 180 * M_PI);


        // static double first_angle = angle;  // 3.748

        // static double yaw_angle = (3.748 - first_angle ) / 180 * M_PI; // 7.648
        double angle_ = (angle )  / 180 * M_PI;
        Eigen::Affine3f T_WG = pcl::getTransformation(local_E_, local_N_, 0, 0, 0, angle_);
        Eigen::Affine3f T_GL = pcl::getTransformation(x_bias, y_bias, 0, 0, 0, angle_bias/180 * M_PI); //*NIU 2.375 左转偏左 0.025 10车最优：2.25 0.007 -3.24
        Eigen::Affine3f T_WL =  T_WG * T_GL;

        pcl::getTranslationAndEulerAngles(T_WL, transformPos_[3], transformPos_[4], transformPos_[5],
                                          transformPos_[0], transformPos_[1], transformPos_[2]);
        
        // cout << "transform_yaw : " << transformPos_[2] *180 / M_PI << endl; 

        // 存储位姿
        Eigen::Quaternionf q_odom;
        q_odom.x() = current_q_.x;
        q_odom.y() = current_q_.y;
        q_odom.z() = current_q_.z;
        q_odom.w() = current_q_.w;
        Eigen::Matrix3f odom(q_odom);
        Eigen::Matrix4f gpsPose = T_WL.matrix();
        gpsPose.block<3, 3>(0, 0) = odom;
        SavePose(gpsPose);


        // cout << "local_E_ - transformPos_[3] = " << local_E_ - transformPos_[3] << endl;
        // cout << "local_N_ - transformPos_[4] = " << local_N_ - transformPos_[4] << endl;

        // cout << "transformPos_[3] = " << transformPos_[3] << endl;
        // cout << "transformPos_[4] = " << transformPos_[4] << endl;
    }

    // 储存位姿
    void SavePose(const Eigen::Matrix4f& pose) {
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ofs_ << pose(i, j);
            
                if (i == 2 && j == 3) {
                    ofs_ << std::endl;
                } else {
                    ofs_ << " ";
                   // cout << "record gps data!" << endl;
                 }
            }
        }
    }

    // 发布数据
    void PublishData()
    {
        // publish odometry
        nav_msgs::Odometry gpsOdom;
        gpsOdom.header.stamp = currentGpsMsg_.header.stamp;
        gpsOdom.header.frame_id = currentGpsMsg_.header.frame_id;
        gpsOdom.pose.pose.position.x = transformPos_[3];
        gpsOdom.pose.pose.position.y = transformPos_[4];
        gpsOdom.pose.pose.position.z = 0;
        gpsOdom.pose.pose.orientation = current_q_;
        gpsOdom.pose.covariance[0] = transformPos_[2];
        if(pubGpsOdmetry_.getNumSubscribers() != 0)
        {
            pubGpsOdmetry_.publish(gpsOdom);
        }

        static nav_msgs::Path gpsPath;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = currentGpsMsg_.header.stamp;
        pose_stamped.header.frame_id = currentGpsMsg_.header.frame_id;
        pose_stamped.pose.position.x = transformPos_[3];
        pose_stamped.pose.position.y = transformPos_[4];
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation = current_q_;

        if(pubGpsPath_.getNumSubscribers() != 0)
        {
            gpsPath.header.stamp = currentGpsMsg_.header.stamp;
            gpsPath.header.frame_id = currentGpsMsg_.header.frame_id;
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
