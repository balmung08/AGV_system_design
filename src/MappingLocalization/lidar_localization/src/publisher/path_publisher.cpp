/*
 * @Description: Path信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/path_publisher.hpp"

namespace lidar_localization{
PathPublisher::PathPublisher(ros::NodeHandle& nh,
                        std::string topic_name,
                        std::string frame_id,
                        int buff_size)
    :nh_(nh){
        publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
        path_.header.frame_id = frame_id;
        poseStamped_.header.frame_id = frame_id;
    }

void PathPublisher::Publish(const Eigen::Matrix4f& transform_matrix, double time){
    ros::Time ros_time((float)time);
    PublishData(transform_matrix, ros_time);
}

void PathPublisher::PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time){
    poseStamped_.header.stamp = time;

    // poseStamped_
    poseStamped_.pose.position.x = transform_matrix(0, 3);
    poseStamped_.pose.position.y = transform_matrix(1, 3);
    poseStamped_.pose.position.z = transform_matrix(2, 3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    poseStamped_.pose.orientation.x = q.x();
    poseStamped_.pose.orientation.y = q.y();
    poseStamped_.pose.orientation.z = q.z();
    poseStamped_.pose.orientation.w = q.w();

    //path_
    path_.header.stamp = time;
    path_.poses.push_back(poseStamped_);

    publisher_.publish(path_);

}

bool PathPublisher::HasSubscribers(){
    return publisher_.getNumSubscribers() != 0 ;
}
}