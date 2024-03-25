/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
enum Verbosity { SILENT = 0, ERROR = 1, WARN = 2, INFO = 3, DEBUG = 4 };

#define global_verbosity Verbosity::INFO
#define DISTANCE_P2P(x1, y1, x2, y2) std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2))

// *NIU: 创建一个结构体来存储每个消息的打印时间戳和频率
struct MessagePrintInfo {
    ros::Time last_print_time;
    double print_interval;
};
// *NIU: 创建一个map，将消息名称映射到其打印信息
std::map<std::string, MessagePrintInfo> message_print_info;
bool rateControlledPrint(const std::string& message_name, double print_interval) {
    // 检查消息是否存在于映射中
    if (message_print_info.find(message_name) == message_print_info.end()) {
        // 如果消息不存在，添加它并初始化打印信息
        message_print_info[message_name] = {ros::Time::now(), print_interval};
    }
    // 获取消息的打印信息
    MessagePrintInfo& info = message_print_info[message_name];

    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - info.last_print_time).toSec();

    if (elapsed_time >= info.print_interval) {
        info.last_print_time = current_time;
        return true;
    }
    return false;
}

namespace lidar_localization {
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) :nh_(nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 1000);
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/gps/odometry", 100000);

    OdomPub = nh.advertise<nav_msgs::Odometry>("/fusion_loco", 10);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "map", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "map", 1000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "map", "lidar", 1);
    laser_path_pub_ptr_ = std::make_shared<PathPublisher>(nh, "/laser_path","map", 1);
    fusion_path_pub_ptr_ = std::make_shared<PathPublisher>(nh, "/fusion_path","map", 1);
    // laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("map", "base_link");
    matching_ptr_ = std::make_shared<Matching>();

    x_lidar_in_gps = 0;
    y_lidar_in_gps = 0;
    angle_lidar_in_gps= 0;

    // 标志位
    odoAvailableFlag = false;
    needStableCheck = true;
    needFixInitial = false;

    // 读取参数
    nh.getParam("/gps_enu/x_bias", x_lidar_in_gps);
    nh.getParam("/gps_enu/y_bias", y_lidar_in_gps);
    nh.getParam("/gps_enu/angle_bias", angle_lidar_in_gps);
    config_file_path = WORK_SPACE_PATH + "/config/matching/matching.yaml";
    config_node = YAML::LoadFile(config_file_path);
    initPolygons();
    initPosebyconfig();
    char str[100];
    for(int i=0;i<polygons.size();i++)
    {
        sprintf(str,"/odom_range%d",i+1);
        polygon_pub[i]=nh.advertise<geometry_msgs::PolygonStamped>(str, 10);
    } 
}


bool MatchingFlow::Run() 
{
    for(int i=0;i<polygons.size();i++)
    {
        // polygons[i].header.stamp=ros::Time::now();
        polygon_pub[i].publish(polygons[i]);
    }
    
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    // *NIU：当到达需要使用激光里程计的区域时，才对NDT匹配算法进行初始化，并开始定位；否则清除当前状态，一直等待(不使用该模式，直接注释即可)
    if(!enable_Matching())  return false; //注释1

    //*NIU: 读取传入的点云和gnss队列
    ReadData(); 

    while(HasData()) {
        if (!ValidData())  continue;
        if (UpdateMatching()) {
            // *NIU：在GPS信号不好的地方做初始化时，里程计出现漂移，此时应该重新进行初始化，直到定位恢复稳定(不需要时可注释)
            if(isOdoDrift(odoAvailableFlag)) break; //注释1

            Eigen::Vector2f p;
            if(FindGpsData(current_cloud_data_.time)){
                p.x() = current_gnss_data_.pose(0,3);
                p.y() = current_gnss_data_.pose(1,3);
            } else {
                p.x() = laser_odometry_(0,3);
                p.y() = laser_odometry_(1,3);
            }
            GPS_or_Odo(p);
        }
    }

    return true;
}

bool MatchingFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0) return false;
    
    if (matching_ptr_->HasInited()) return true;
    
    // 注释1
    if(needFixInitial) {
        matching_ptr_->SetGNSSPose(init_pose_matrix);
        ROS_INFO( "\033[1;32m         Initial Successfully By FixPose.\033[0m");
        return true;
    }

    if (gnss_data_buff_.size() == 0) {
        return false;
    }
    
    
    return true;
}

//*NIU: 从点云队列取出下一帧点云数据
bool MatchingFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    if (matching_ptr_->HasInited()) 
    {
        cloud_data_buff_.pop_front();
        // gnss_data_buff_.clear();
        return true;
    }
    ROS_WARN( "\033[1;33m         NOT INITTED.\033[0m");
    current_gnss_data_ = gnss_data_buff_.front();
    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.1) {
        cloud_data_buff_.pop_front();
        return false;
    }
    if (diff_time > 0.1) {
        gnss_data_buff_.pop_front();
        return false;
    }
    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    return true;
}

bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {
        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);
        ROS_INFO( "\033[1;32m         Initial Successfully By GPS.\033[0m");
    }
    laser_odometry_last_ = laser_odometry_;
    Eigen::Vector2f cur_odo;
    cur_odo << laser_odometry_(0,3), laser_odometry_(1,3);
    laser_odometry_his_longterm.push_back(cur_odo);
    if (laser_odometry_his_longterm.size() > 20)
        laser_odometry_his_longterm.pop_front();
    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool MatchingFlow::FindGpsData(double curTime) {
    static PoseData last_gnss_data = {Eigen::Matrix4f::Identity(), 0.0};
    double frame_dis_left = 0, frame_dis_right = 0;
    
    if(gnss_data_buff_.size() < 1)
        return false;
    while(gnss_data_buff_.size() > 0){
        if((gnss_data_buff_.front().time - curTime) > 0.2 ) {
            ROS_WARN("Gnss Info too New");
            return false;
        }
        if(curTime - gnss_data_buff_.front().time > 0.2){
            last_gnss_data = gnss_data_buff_.front();
            gnss_data_buff_.pop_front();
        } 
        else break;
    }

    if (gnss_data_buff_.size() == 0) return false;
    else needFixInitial = false;

    int closestFrame_Index = 0;
    double minTimeDis = 999;
    // *NIU 找到最近一帧GPS索引
    for(size_t i = 0; i < gnss_data_buff_.size(); i++)
    {
        if (abs(gnss_data_buff_[i].time - curTime) < minTimeDis){
            minTimeDis = abs(gnss_data_buff_[i].time - curTime);
            closestFrame_Index = i;
        }    
    }
    current_gnss_data_ = gnss_data_buff_[closestFrame_Index];
    if(gnss_data_buff_.size() > 1){
        // *NIU 利用左右帧判断GPS是否unstable
        if(closestFrame_Index - 1 >= 0){
            int left_Index = closestFrame_Index - 1;
            frame_dis_left = DISTANCE_P2P(gnss_data_buff_[left_Index].pose(0,3), gnss_data_buff_[left_Index].pose(1,3), 
                                          current_gnss_data_.pose(0,3), current_gnss_data_.pose(1,3));
        } 
        if (closestFrame_Index + 1 < gnss_data_buff_.size()){
            int right_Index = closestFrame_Index + 1;
            frame_dis_right = DISTANCE_P2P(gnss_data_buff_[right_Index].pose(0,3), gnss_data_buff_[right_Index].pose(1,3), 
                                           current_gnss_data_.pose(0,3), current_gnss_data_.pose(1,3));
        }

        if(rateControlledPrint("topic1", 0.5)) 
            ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "frame_left_dis = [%.2f]m  frame_right_dis = [%.2f]m",
                                            frame_dis_left, frame_dis_right);

        // 车速1m/s左右 小于0.1
        if(frame_dis_left > 0.08 || frame_dis_right > 0.08) {
            gps_cal_time = ros::Time::now();
                ROS_WARN("\033[1;33mGPS Not Stable. Fluctuating Signal. frame_left_dis = [%.2f]m  frame_right_dis = [%.2f]m\033[0m",
                                            frame_dis_left, frame_dis_right);
            return false;
        }

        if ((ros::Time::now() - gps_cal_time).toSec() < 3) {
            if(rateControlledPrint("topic3", 0.5))
                ROS_WARN( "\033[1;33m         Waiting for GPS to Regain Stability.\033[0m");
            return false;
        }
          
    }
    // *NIU: 对GPS做插值 必须保证持有两帧GPS
    // if(gnss_data_buff_.size() > 1)
    //     LinearInterpolation(gnss_data_buff_, current_gnss_data_, closestFrame_Index, curTime);
    last_gnss_data = current_gnss_data_;
    return true;

}

bool MatchingFlow::PublishData(double timeStamp, const Eigen::Matrix4f& pose) {
    double timeStamp_ = timeStamp;
    Eigen::Matrix4f pose_ = pose;
    PublishDataFromBase(timeStamp_, pose_);
    // laser_tf_pub_ptr_->SendTransform(pose);

    // *NIU：里程计信息位于lidar坐标系下，需转换到gps坐标系下，和gps_base保持一致
    Eigen::Matrix4f odo_trans2gpsframe;
    Trans_lidar2gps(laser_odometry_, odo_trans2gpsframe);
    savePosetoconfig(laser_odometry_);
    laser_odom_pub_ptr_->Publish(odo_trans2gpsframe, timeStamp);

    laser_path_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    fusion_path_pub_ptr_->Publish(pose, timeStamp);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());
    float error = DISTANCE_P2P(laser_odometry_(0, 3), laser_odometry_(1, 3), pose(0, 3), pose(1, 3));
    // if(rateControlledPrint("topic2", 2))
    //     ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "The Euclidean distance between odom and GPS: [%.2f] m", error);
    if(error > 1) 
        ROS_INFO_COND(global_verbosity >= Verbosity::ERROR, "\033[1;31m Emergency! Stop the Car! Too large error (%.2f) between GPS and LidarOdom!\033[0m",error);
    return true;
    
}

 //*NIU：现在定位信息位于激光雷达，需转换到车体中心
void MatchingFlow::Trans_lidar2gps(const Eigen::Matrix4f& pose, Eigen::Matrix4f& respose){
    Eigen::Transform<float, 3, Eigen::Affine> T_WL (pose);
    Eigen::Affine3f T_LG = pcl::getTransformation(-x_lidar_in_gps, -y_lidar_in_gps, 0, 0, 0, -angle_lidar_in_gps/180 * M_PI);
    Eigen::Affine3f T_WG = T_WL * T_LG;
    respose = T_WG.matrix();
}

// 面积法：点与多边形任意相邻两点间三角面积之和=多边形面积
bool InsidePolygon(geometry_msgs::Point32 p, geometry_msgs::Polygon polygon)
{
    int i,j;
    bool inside = false;
    double polygon_area = 0;
    double trigon_area = 0;
    int N=polygon.points.size();

    for (i=0,j=N-1;i<N;j=i++) 
    {
        geometry_msgs::Point32 p1=polygon.points[i], p2=polygon.points[j];
        polygon_area+= p1.x*p2.y-p2.x*p1.y; 
        trigon_area+=abs(p.x*p1.y-p.x*p2.y-p1.x*p.y+p1.x*p2.y+p2.x*p.y-p2.x*p1.y);
    }
 
    trigon_area*=0.5;
    polygon_area = abs(polygon_area * 0.5);
    if ( fabs(trigon_area - polygon_area) < 0.001)  inside=true;
 
    return inside;
}

bool MatchingFlow::insideRanges(geometry_msgs::Point32 p)
{
    bool res=false;
    if(polygons.size()==0)  res=true;   //  如何没有设定区域，默认无穷大区域，在区域内
    for(auto polygon:polygons)
        if(InsidePolygon(p, polygon.polygon))  
        {
            res=true; break;
        }
    return res;    
}

void MatchingFlow::GPS_or_Odo(const Eigen::Vector2f p)
{   
    // 注释1
    if(!odoAvailableFlag && !needFixInitial){
        return; 
    }

    nh_.setParam("matching_loc_enable", true);
    PublishData(current_cloud_data_.time, laser_odometry_);
    if(rateControlledPrint("topic5", 2))
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "\033[1;34m<<<< NO GPS, ODOM IN CHARGE NOW >>>>\033[0m");
}

//*NIU:判断是否进入定位初始化区域
bool MatchingFlow::inInitialArea(){
    geometry_msgs::Point32 p;
    //*NIU：使用固定位姿初始化
    // static bool checkOnceOnly = true;
    // if(checkOnceOnly){
    //     // checkOnceOnly = false;
    //     p.x = init_pose.x(), p.y = init_pose.y();
    //     // ROS_INFO("is inside %d", insideRanges(p));
    //     if(insideRanges(p)) {
    //         return true;
    //     }
    // }

    nav_msgs::Odometry gps_odom;
    if(gnss_sub_ptr_== NULL) 
        return false;
    gps_odom = gnss_sub_ptr_->GetData();
    if (gps_odom.pose.covariance[14] < 4) 
        return false;
    
    p.x=gps_odom.pose.pose.position.x,  p.y=gps_odom.pose.pose.position.y;  
    if(insideRanges(p))  return true;
    else 
    {
        needStableCheck = true;
        return false;
    }  

}

//*NIU：判断是否在需要里程计定位的区域内
bool MatchingFlow::enable_Matching()
{
    if(needFixInitial) return true;
    if(!inInitialArea())
    {
        //*NIU: 不在区域内时，清除所有状态
        clearState();
        return false;
    }
    return true;
}

void MatchingFlow::clearState()
{
    cloud_data_buff_.clear();
    gnss_data_buff_.clear();
    cloud_sub_ptr_->new_cloud_data_.clear();
    gnss_sub_ptr_->new_pose_data_.clear();
    matching_ptr_->has_inited_ = false;
    needStableCheck = true;
    nh_.setParam("matching_loc_enable", false);
}

//*NIU:判断里程计是否初始化时出现了漂移，漂移则重新进行初始化；此外判断多帧定位信号是否恢复了稳定，否则不会发布定位信息。
bool MatchingFlow::isOdoDrift(bool &odoAvailableFlag){
    if(needFixInitial) return false;
    //*NIU：利用连续的10帧信号判断当前定位是否稳定可用
    if(laser_odometry_his_longterm.size() < 20){
        odoAvailableFlag = false;
        // ROS_INFO("laser_odometry_his_longterm.size() = %d", laser_odometry_his_longterm.size());
    }
    else{
        for(size_t i = 1; i < laser_odometry_his_longterm.size(); i++){
            odoAvailableFlag = true;
            float dis = DISTANCE_P2P(laser_odometry_his_longterm.at(i-1).x(), laser_odometry_his_longterm.at(i-1).y(),
                                     laser_odometry_his_longterm.at(i).x(), laser_odometry_his_longterm.at(i).y());
            if(dis > 0.08){
                odoAvailableFlag = false;
                ROS_WARN( "\033[1;33m         Unstable Lidar Odometry. Waiting...\033[0m");
                break;
            }
        }
        if(odoAvailableFlag && needStableCheck){
            float dis = DISTANCE_P2P(laser_odometry_(0,3), laser_odometry_(1,3), 
                                     current_gnss_data_.pose(0,3), current_gnss_data_.pose(1,3));
            if(dis > 0.3) {
                ROS_WARN( "\033[1;33m         Too large dis[%.2f] between GPS and lidarOdo. Waiting \033[0m", dis);
                odoAvailableFlag = false;
            }
            else {
                needStableCheck = false;
                ROS_WARN( "\033[1;32m         LidarOdo Stable Now. \033[0m");
            }
        }
        
    }

    std::size_t last_index = laser_odometry_his_longterm.size() - 1;
    if(laser_odometry_his_longterm.size() < 2) return false;
    float dis_frame = DISTANCE_P2P(laser_odometry_his_longterm[last_index].x(), laser_odometry_his_longterm[last_index].y(), 
                                   laser_odometry_his_longterm[last_index-1].x(), laser_odometry_his_longterm[last_index-1].y());
    // *NIU：最近两帧定位出现漂移，重新进行初始化
    if(dis_frame > 0.08) {
        clearState();
        laser_odometry_his_longterm.clear();
        ROS_WARN( "\033[1;33m         Matching Initial Failed. Dis Between Frames is %.2f. Try again.\033[0m", dis_frame);
        return true;
    }
    return false;
}

bool MatchingFlow::initPosebyconfig(){
    if(!config_node["InitialPose"].IsDefined())  return false;
    needFixInitial = config_node["needFixInitial"].as<bool>();
    for(size_t i = 0;i < 4; i++){
        for(size_t j = 0;j < 4; j++){
            init_pose_matrix(i,j) = config_node["InitialPose"][i][j].as<float>();
        }
        
    }
    // std::cout << "init_pose_matrix: "<< init_pose_matrix << std::endl;
    return true;
}   

void MatchingFlow::savePosetoconfig(Eigen::Matrix4f & pose)
{   
    std::ofstream fout(config_file_path);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            config_node["InitialPose"][i][j] = pose(i,j);
        }
    }
    fout << config_node;
    fout.close();

    // std::cout << pose << std::endl;
    // Eigen::Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    // Eigen::Vector3f eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
    // ROS_INFO("x = %.2f, y = %.2f, yaw = %.2f ", pose(0, 3), pose(1, 3), eulerAngles(2));
    // std::ofstream fout(config_file_path);
    // config_node["InitialPose"] = pose;
    // config_node["InitialPose"]["Pose"][1] =float(0);
    // config_node["InitialPose"]["Pose"][2] =float(0);
    // fout << config_node;
    // fout.close();
    // ROS_INFO("Save init pose to config file");//75.45 -92.92 -51.88 1.63
}


void MatchingFlow::initPolygons()
{
    YAML::Node polygon_node = config_node["polygons"];
    if(!polygon_node.IsDefined())  return;

    polygons.clear();
    
    int range_n=polygon_node.size();
    for(int i=0;i<range_n;i++)
    {
        geometry_msgs::PolygonStamped polygon;
        polygon.header.frame_id="map";
        polygon.header.stamp=ros::Time::now();

        char str[100];
        sprintf(str,"range%d", i+1);
        YAML::Node range_node=polygon_node[str];
        int point_n=range_node.size();
        for(int j=0;j<point_n;j++)
        {
            sprintf(str,"p%d", j+1);
            YAML::Node point_node=range_node[str];
           
            geometry_msgs::Point32 point;
            point.x=point_node[0].as<float>();
            point.y=point_node[1].as<float>(); 
            polygon.polygon.points.push_back(point);

            // ROS_INFO("SSSSSSSSSSSS %.1f %.1f", point_node[0].as<float>(), point_node[1].as<float>());
        }        

        polygons.push_back(polygon);
    }
}

void MatchingFlow::LinearInterpolation(const std::deque<PoseData>& pose_buff, PoseData& current_pose, const int& closestFrame_Index, const double &targetTime){
    double closestFrame_time = pose_buff[closestFrame_Index].time;
    double dur_time = abs(targetTime - closestFrame_time);
    if (targetTime > closestFrame_time && (closestFrame_Index + 1) < pose_buff.size() 
            && targetTime < pose_buff.at(closestFrame_Index + 1).time){
        current_pose.pose = pose_buff[closestFrame_Index].pose + dur_time * (pose_buff[closestFrame_Index + 1].pose - pose_buff[closestFrame_Index].pose)/
                                            (pose_buff[closestFrame_Index + 1].time - pose_buff[closestFrame_Index].time);
    }else if((targetTime < closestFrame_time) && (closestFrame_Index - 1) >= 0 
                && targetTime > pose_buff.at(closestFrame_Index - 1).time){
        current_pose.pose = pose_buff[closestFrame_Index].pose - dur_time * (pose_buff[closestFrame_Index].pose - pose_buff[closestFrame_Index - 1].pose)/
                                (pose_buff[closestFrame_Index].time - pose_buff[closestFrame_Index -1].time);
    }
}

bool MatchingFlow::PublishData() {
    // laser_tf_pub_ptr_->SendTransform(laser_odometry_);

    // *NIU：里程计信息位于lidar坐标系下，需转换到gps坐标系下，和gps_base保持一致
    Eigen::Matrix4f odo_trans2gpsframe;
    Trans_lidar2gps(laser_odometry_, odo_trans2gpsframe);
    laser_odom_pub_ptr_->Publish(odo_trans2gpsframe, current_cloud_data_.time);
    
    laser_path_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());
  
    return true;
}

bool MatchingFlow::PublishDataFromBase(double timeStamp, Eigen::Matrix4f pose){
    nav_msgs::Odometry odom_gps;
    if(gnss_sub_ptr_== NULL) 
        return false;
    odom_gps = gnss_sub_ptr_->GetData();
    odom_gps.pose.pose.position.x = pose(0,3);
    odom_gps.pose.pose.position.y = pose(1,3);
    odom_gps.pose.pose.position.z = pose(2,3);
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    odom_gps.pose.pose.orientation.x = q.x();
    odom_gps.pose.pose.orientation.y = q.y();
    odom_gps.pose.pose.orientation.z = q.z();
    odom_gps.pose.pose.orientation.w = q.w();
    OdomPub.publish(odom_gps);
    return true;
}


}