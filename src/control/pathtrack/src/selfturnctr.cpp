#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <common/public.h>
#include <car_ctr/car_ctr.h>
#include <car_ctr/car_state.h>
// #include <common/mydisplay.h>

using namespace std;

class TSelfTurnCtr
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher selfturnctr_pub, carctr_pub; 
    ros::Subscriber track_angle_err_sub, speedlimit_sub, movedir_sub;
    float track_angle_err=0;
    float speedlimit=999;
    car_ctr::car_ctr car_ctr;
    string move_dir="back";

    TNodeCheck *nodecheck;
    

public:
    TSelfTurnCtr()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();

        // aimpoint_pub = nh_local->advertise<geometry_msgs::PointStamped>("aimpoint", 10); // 发布预瞄点
        carctr_pub = nh_local->advertise<car_ctr::car_ctr>("/car_cmd", 10); // 发布控制信息
        selfturnctr_pub=nh_local->advertise<std_msgs::Int32>("self_turn_ctr", 10);
        speedlimit_sub = nh->subscribe<std_msgs::Float32>("/speed_limit", 10, &TSelfTurnCtr::SpeedLimitCallback, this);
        track_angle_err_sub = nh->subscribe<std_msgs::Float32>("/pathtrack/track_angle_err", 10, &TSelfTurnCtr::TrackErrCallback, this);
        movedir_sub = nh->subscribe<std_msgs::String>("/pathtrack/move_dir", 10, &TSelfTurnCtr::MoveDirCallback, this);

        nodecheck = new TNodeCheck(nh_local, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);

        // nh_local->getParam("angle_offset", angle_offset);
        // nh_local->getParam("test_speed", test_speed);
    };

    void TrackErrCallback(const std_msgs::Float32::ConstPtr &msg); 
    void SpeedLimitCallback(const std_msgs::Float32::ConstPtr &msg);
    void MoveDirCallback(const std_msgs::String::ConstPtr &msg);
    void Run();
    void PubCarCtr(car_ctr::car_ctr ctr);
 
};

void TSelfTurnCtr::SpeedLimitCallback(const std_msgs::Float32::ConstPtr &msg) 
{
    speedlimit = msg->data;
}

void TSelfTurnCtr::PubCarCtr(car_ctr::car_ctr ctr)
{
    carctr_pub.publish(ctr);
}


void TSelfTurnCtr::TrackErrCallback(const std_msgs::Float32::ConstPtr &msg) 
{
    track_angle_err = msg->data;
    // ROS_INFO("%.2f", track_angle_err);
}

void TSelfTurnCtr::MoveDirCallback(const std_msgs::String::ConstPtr &msg) 
{
    move_dir = msg->data;
    // ROS_INFO("%.2f", track_angle_err);
}

void TSelfTurnCtr::Run() //  主运行函数
{
    nodecheck->Find("node_rate")->Beat();

    static TTimer turn_tmr;
    static int self_turn_ctr=0;
    float selfturn_angle_err=30;
    
    if (self_turn_ctr==0 && fabs(track_angle_err) > selfturn_angle_err)
    {
        self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 1) //  转动至角度误差小于阈值
    {
        car_ctr.enable=true;
        car_ctr.turnmode=4;
        car_ctr.speed=0;
        car_ctr.angle=0;
        
        float max_err = 1;      // 最小误差角
        float turn_speed = 0.1;
        if(move_dir=="back")  turn_speed*=-1;
        if (speedlimit < 0.01) car_ctr.speed = 0;
        if (track_angle_err> max_err)  car_ctr.speed = -turn_speed;
        else if (track_angle_err< -max_err)  car_ctr.speed = turn_speed;
        else 
        {
            car_ctr.turnmode=0;
            self_turn_ctr++; turn_tmr.Clear();      
        }
    }
    else if (self_turn_ctr==2 && turn_tmr.GetValue()>4) // stop turn
    {
        self_turn_ctr=0;
    }

    std_msgs::Int32 msg;
    msg.data=self_turn_ctr;
    selfturnctr_pub.publish(msg);

    // printf("%d\n", car_ctr.turnmode);
    if(self_turn_ctr>0)  PubCarCtr(car_ctr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "selfturnctr");
    TSelfTurnCtr selfturnctr;

    ros::Rate rate(20);
    while (ros::ok())
    {
        selfturnctr.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
