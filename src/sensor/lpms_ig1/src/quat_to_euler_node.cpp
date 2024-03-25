
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <tf/transform_datatypes.h>

ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;

const float r2d = 57.29577951f;

void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    rpy_publisher.publish(rpy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener");
    ros::NodeHandle n;
    rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_angles", 1000);
    quat_subscriber = n.subscribe("/imu/data", 1000, MsgCallback);

    ROS_INFO("waiting for imu data");
    ros::spin();
    return 0;
}
