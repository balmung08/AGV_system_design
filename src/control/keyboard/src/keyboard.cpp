#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <termio.h>
#include <stdio.h>

int key_command = 0x00;

void scanKeyboard()
{
    struct termios new_settings;
    struct termios stored_settings;
    //设置终端参数
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &=(~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    key_command = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;

    ros::Publisher key_publisher = nh.advertise<std_msgs::UInt16>("/keyboard", 10);
    std_msgs::UInt16 key_c;   // w:1  a:2  s:3  d:4  i:5  j:6  k:7  l:8

    ros::Rate rate(20);
    while (ros::ok())
    {
        scanKeyboard();
        // printf("%02x\n", key_command);
        key_c.data=key_command;
        key_publisher.publish(key_c);

        ros::spinOnce();
        rate.sleep();
    }
}