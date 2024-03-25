#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <tf/transform_datatypes.h>

#include "common/public.h"

ros::Publisher angular_vel_deg_publisher;
ros::Publisher rpy_deg_publisher;
ros::Subscriber imudata_subscriber;

const float r2d = 57.29577951f;

TNodeCheck *nodecheck;

void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    geometry_msgs::Vector3 angular_vel;
    angular_vel.x = msg->angular_velocity.x*r2d;
    angular_vel.y = msg->angular_velocity.y*r2d;
    angular_vel.z = msg->angular_velocity.z*r2d;

    angular_vel_deg_publisher.publish(angular_vel);


    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::Vector3 rpy;
    rpy.x = roll*r2d;
    rpy.y = pitch*r2d;
    rpy.z = yaw*r2d;

    rpy_deg_publisher.publish(rpy);

    nodecheck->Find("node_rate")->Beat();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener");
    ros::NodeHandle nh("~");
    angular_vel_deg_publisher = nh.advertise<geometry_msgs::Vector3>("angular_vel_deg", 1000);
    rpy_deg_publisher = nh.advertise<geometry_msgs::Vector3>("rpy_deg", 1000);
    imudata_subscriber = nh.subscribe("/imu/data", 1000, MsgCallback);

    nodecheck=new TNodeCheck(&nh, "node_rate");
	nodecheck->Find("node_rate")->SetLimit(80);

    ROS_INFO("waiting for imu data");
    ros::spin();
    return 0;
}
