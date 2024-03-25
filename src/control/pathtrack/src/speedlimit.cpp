#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <common/public.h>
#include <car_ctr/car_ctr.h>
#include <car_ctr/car_state.h>
#include <common/mydisplay.h>

using namespace std;

float GetVelByDistance(float max_d, float max_vel, float min_d, float min_vel, float cur_d)
{
    float res;
    float Vmid=1; 
    float k=(max_vel-min_vel)/(max_d-min_d);
    float b=0.05-k*min_d;
    float sc=(Vmid-b)/k;
    float a=(max_vel-Vmid)/(pow(max_d,2.5)-pow(sc,2.5));
    float b2=max_vel-a*pow(max_d,2.5);

    if(cur_d>=max_d)  res=max_vel;
    else if (cur_d>=sc)  res=a*pow(cur_d,2.5)+b2;
    else if (cur_d>=min_d)  res=k*cur_d+b;
    else  res=min_vel;

    return res;
}

void GetObsSpeedLimit(float obs_dis, float &speedlimit)
{
    float stop_dis=0.6;
    float speed=GetVelByDistance(15,3,stop_dis,0,obs_dis);  

    // 缓慢加速
    float dspeed=speed-speedlimit;
    if(dspeed>0.01) dspeed=0.01;
    speedlimit+=dspeed;
}
 
void GetCurveSpeedLimit(nav_msgs::Path path, float len, float &speedlimit)
{
    speedlimit=999;
    if(path.poses.size()<5) return;

    float a0=GetYawFromPose(path.poses.front())*180/M_PI;
    float d_angle=0;
    float s=0;
    for(int i=0;i<path.poses.size()-3;i++)
    {
        float ds=GetDistanceXY(path.poses[i].pose.position, path.poses[i+1].pose.position);
        s+=ds;

        float a1=GetYawFromPose(path.poses[i])*180/M_PI;
        d_angle=max(d_angle, fabs(a1-a0));
        if(s>len)  break;
    }
        
    if(d_angle>50)  speedlimit=0.8;
    else if(d_angle>30) speedlimit=1.0;
    else if(d_angle>20) speedlimit=1.5;
    else if(d_angle>10) speedlimit=2.0; 
}

class TSpeedLimit
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher speedlimit_pub; 
    ros::Subscriber localpath_sub, 
                    obs_dis_front_sub, 
                    obs_dis_right_sub, 
                    obs_dis_left_sub, 
                    obs_dis_back_sub, 
                    work_speedlimit_sub;

    nav_msgs::Path localpath;

    TNodeCheck *nodecheck;
    float front_obs_dis=999, back_obs_dis=999, left_obs_dis=999, right_obs_dis=999;
    float work_speedlimit=999;

public:
    TSpeedLimit()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();

        speedlimit_pub = nh_local->advertise<std_msgs::Float32>("/speed_limit", 10);

        localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &TSpeedLimit::LocalPathCallback, this);
        obs_dis_front_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_front/obs_dis", 10, &TSpeedLimit::ObsDisCallback_front, this);
        obs_dis_back_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_back/obs_dis", 10, &TSpeedLimit::ObsDisCallback_back, this);
        obs_dis_left_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_left/obs_dis", 10, &TSpeedLimit::ObsDisCallback_left, this);
        obs_dis_right_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_right/obs_dis", 10, &TSpeedLimit::ObsDisCallback_right, this);
        
        work_speedlimit_sub = nh->subscribe<std_msgs::Float32>("/workctr/speedlimit", 10, &TSpeedLimit::WorkSpeedLimitCallback, this);
        
        nodecheck = new TNodeCheck(nh_local, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);
    };

    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void ObsDisCallback_front(const std_msgs::Float32::ConstPtr &msg); 
    void ObsDisCallback_back(const std_msgs::Float32::ConstPtr &msg); 
    void ObsDisCallback_left(const std_msgs::Float32::ConstPtr &msg); 
    void ObsDisCallback_right(const std_msgs::Float32::ConstPtr &msg); 
    void WorkSpeedLimitCallback(const std_msgs::Float32::ConstPtr &msg); 
    void Run();   
};


void TSpeedLimit::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) // 接收局部路径
{
    localpath = *msg;
}

void TSpeedLimit::ObsDisCallback_front(const std_msgs::Float32::ConstPtr &msg)
    {
        front_obs_dis = msg->data;
    }

void TSpeedLimit::ObsDisCallback_back(const std_msgs::Float32::ConstPtr &msg)
{
    back_obs_dis = msg->data;
}

void TSpeedLimit::ObsDisCallback_left(const std_msgs::Float32::ConstPtr &msg)
{
    left_obs_dis = msg->data;
}

void TSpeedLimit::ObsDisCallback_right(const std_msgs::Float32::ConstPtr &msg)
{
    right_obs_dis = msg->data;
}

void TSpeedLimit::WorkSpeedLimitCallback(const std_msgs::Float32::ConstPtr &msg)
{
    work_speedlimit=msg->data;
}

void TSpeedLimit::Run() //  主运行函数
{
    nodecheck->Find("node_rate")->Beat();

    static float front_speedlimit=999, back_speedlimit=999, left_speedlimit=999, right_speedlimit=999;
    GetObsSpeedLimit(front_obs_dis, front_speedlimit);
    GetObsSpeedLimit(back_obs_dis, back_speedlimit);
    GetObsSpeedLimit(left_obs_dis, left_speedlimit);
    GetObsSpeedLimit(right_obs_dis, right_speedlimit);

    string move_dir=CheckMoveDir(localpath, 0);
    float obs_speedlimit=999;
    if (move_dir == "front")  obs_speedlimit=front_speedlimit;
    else if(move_dir == "back")  obs_speedlimit=back_speedlimit; 
    else if(move_dir == "left")  obs_speedlimit = left_speedlimit;
    else if(move_dir == "right")  obs_speedlimit = right_speedlimit;

    float curve_speedlimit=999;
    GetCurveSpeedLimit(localpath, 5, curve_speedlimit);

    float speedlimit=min(obs_speedlimit, curve_speedlimit);
    speedlimit=min(speedlimit, work_speedlimit);
    std_msgs::Float32 msg;
    msg.data=speedlimit;
    speedlimit_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speedlimit");
    TSpeedLimit sl;

    ros::Rate rate(20);
    while (ros::ok())
    {
        sl.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
