#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lpms_ig1_rs485_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("/imu/get_imu_data");

    ros::Rate loop_rate(50);
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;

    int count = 0;
    while (ros::ok())
    {

        if (client.call(req, res))
        {
            ROS_INFO("get_imu_data: %d", count++);
        }
        else
        {
            ROS_ERROR("Failed to call service get_imu_data");
            return 1;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}