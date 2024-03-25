#include <ros/ros.h>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <common/public.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <car_ctr/car_ctr.h>
#include <car_ctr/car_state.h>
#include <mqtt_comm/task.h>
#include <stdlib.h>
#include <common/mydisplay.h>

using namespace std;

// #define selfturn_angle_err 40

class TPathTrack
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher aimpoint_pub, carctr_pub, track_state_pub,move_dir_pub,obs_speedlimit_pub; 
    ros::Subscriber localpath_sub, carstate_sub, remainpath_sub, passedpath_sub, task_sub, obs_speedlimit_sub, obs_dis_sub;
    ros::Subscriber obs_front_speedlimit_sub,obs_back_speedlimit_sub,obs_left_speedlimit_sub,obs_right_speedlimit_sub, work_speedlimit_sub;
    ros::Subscriber front_obs_dis_sub,back_obs_dis_sub;

    nav_msgs::Path localpath;
    int self_turn_ctr = 0, turnmode_ctr = 0;
    car_ctr::car_state cur_carstate;
    
    mqtt_comm::task cur_task;

    TNodeCheck *nodecheck;
    
    float angle_offset = 0;
    bool test_flag = false;
    float test_speed = 0;

    geometry_msgs::PointStamped aimpoint;
    float aim_range = 2, ref_speed = 0.6; 
    float steering_property = 1;
    float track_angle_err = 0;
    bool run_enable = false;

    float remain_path_length = 0;
    float passed_path_length = 0;
    float turn_speed_max = 0;

    bool obs_stop_enable=true, laser_work_enable = true;
    float speedlimit = 999, obstacle_speedlimit = 999;
    float work_speedlimit = 999, front_speedlimit = 999, back_speedlimit = 999;
    float left_speedlimit = 999, right_speedlimit = 999;
    float front_car_distance=999,back_car_distance=999,car_distance=999;
    
    car_ctr::car_ctr car_ctr;
    string move_dir="back", stop_str="";

public:
    TPathTrack()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        aimpoint_pub = nh_local->advertise<geometry_msgs::PointStamped>("aimpoint", 10); // 发布预瞄点
        
        nodecheck = new TNodeCheck(nh_local, "node_rate track_rate");
        nodecheck->Find("node_rate")->SetLimit(10);

        carctr_pub = nh_local->advertise<car_ctr::car_ctr>("ctr_cmd", 10); // 发布控制信息
        move_dir_pub = nh_local->advertise<std_msgs::String>("/move_dir",10);

        obs_speedlimit_pub = nh_local->advertise<std_msgs::Float64>("/obs_speedlimit",10);

        localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &TPathTrack::LocalPathCallback, this);
        carstate_sub = nh->subscribe<car_ctr::car_state>("/can_comm/car_state", 10, &TPathTrack::CarStateCallback, this);
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TPathTrack::TaskCallback, this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &TPathTrack::RemainPathCallback, this);
        passedpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/passedpath", 10, &TPathTrack::PassedPathCallback, this);

        obs_front_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_speedlimit", 10, &TPathTrack::Front_ObsSpeedlimitCallback, this);
        obs_back_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/obs_speedlimit", 10, &TPathTrack::Back_ObsSpeedlimitCallback, this);
        obs_left_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_left/obs_speedlimit", 10, &TPathTrack::Left_ObsSpeedlimitCallback, this);
        obs_right_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_right/obs_speedlimit", 10, &TPathTrack::Right_ObsSpeedlimitCallback, this);
        work_speedlimit_sub = nh->subscribe<std_msgs::Float64>("/pawcontrol/work_speedlimit", 10, &TPathTrack::WorkSpeedlimitCallback, this);
        // obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_dis", 10, &TPathTrack::ObsDisCallback, this);
        front_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_dis", 10, &TPathTrack::FrontObsDisCallback, this);
        back_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/obs_dis", 10, &TPathTrack::BackObsDisCallback, this);
       
        aimpoint.header.frame_id = "";

        nh_local->getParam("angle_offset", angle_offset);
        nh_local->getParam("test_speed", test_speed);
    };

    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void CarStateCallback(const car_ctr::car_state::ConstPtr &msg);
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg);
    
    void Front_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Back_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Left_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Right_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void WorkSpeedlimitCallback(const std_msgs::Float64::ConstPtr &msg);

    // void ObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    void Run();
    int SelfTurnCtr();
    int MoveModeCtr();
    void UpdateCtrParam(float vel);
    float GetAimAngleErr(geometry_msgs::PointStamped aimp, string move_dir);
    int StopCtr();
    int StopCheck();
    string CheckMoveDir(nav_msgs::Path path, int id=0);
    void CheckSpeedLimit();
    void CheckObsSpeedLimit();
    void PubCarCtr(car_ctr::car_ctr ctr);
    float SpeedLimitByCurve(float len);
    
    void PurePursuit(car_ctr::car_ctr &car_ctr);
    int FaultCheck();

    float CheckTrackErr()
    {
        float res=0;
        if(localpath.poses.size()>0)  res=GetDistance(*localpath.poses.begin());
        return res;
    }
};

float TPathTrack::SpeedLimitByCurve(float len)
{
    float res=999;
    if(localpath.poses.size()<5) return res;

    float a0=GetYawFromPose(localpath.poses.front())*180/M_PI;
    float d_angle=0;
    float s=0;
    for(int i=0;i<localpath.poses.size()-3;i++)
    {
        float ds=GetDistanceXY(localpath.poses[i].pose.position, localpath.poses[i+1].pose.position);
        s+=ds;

        float a1=GetYawFromPose(localpath.poses[i])*180/M_PI;
        d_angle=max(d_angle, fabs(a1-a0));
        if(s>len)  break;
    }
        
    if(d_angle>50)  res=0.8;
    else if(d_angle>30) res=1.0;
    else if(d_angle>20) res=1.5;
    else if(d_angle>10) res=2.0; 

    return res;
}


void TPathTrack::PubCarCtr(car_ctr::car_ctr ctr)
{
    if (ctr.turnmode == 0 || ctr.turnmode == 3)
    {
        static car_ctr::car_ctr last_ctr;
        float dvel = ctr.speed - last_ctr.speed;
        float max_dvel = 0.05;
        if (dvel > max_dvel)
            ctr.speed = last_ctr.speed + max_dvel;
        else if (dvel < -max_dvel)
            ctr.speed = last_ctr.speed - max_dvel;
        last_ctr = ctr;

        if (ctr.turnmode == 0)
            ctr.angle += angle_offset;
    } 
    // printf("ctr.angle=%.2f\n", ctr.angle);

    carctr_pub.publish(ctr);
}

void TPathTrack::Front_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // if(msg->data.size()>=2)
    // {
    //     front_middle_speedlimit=msg->data[0];
    //     front_side_speedlimit=msg->data[1];
    // }
}

void TPathTrack::Back_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // if(msg->data.size()>=2)
    // {
    //     back_middle_speedlimit=msg->data[0];
    //     back_side_speedlimit=msg->data[1];
    // }
}

void TPathTrack::Left_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=1)
    {
        left_speedlimit=msg->data[0];
    }
}

void TPathTrack::Right_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=1)
    {
       right_speedlimit=msg->data[0];
    }
}

void TPathTrack::WorkSpeedlimitCallback(const std_msgs::Float64::ConstPtr &msg)
{
    work_speedlimit=msg->data;
}

void TPathTrack::FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=3)
    {
        front_car_distance=msg->data[2];
    }
}

void TPathTrack::BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=3)
    {
        back_car_distance=msg->data[2];
    }
}

//  接收任务指令
void TPathTrack::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        // printf("aaaacmd=%s\n",cur_task.cmd.c_str());
        // self_turn_ctr = turnmode_ctr = 0;   //  退出自转和模式切换循环 20231213
    }
}

void TPathTrack::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
}

void TPathTrack::PassedPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    passed_path_length = msg->data;
}    

void TPathTrack::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) // 接收局部路径 获取预瞄点
{
    localpath = *msg;
    // ROS_INFO("%d\n", localpath.poses.size());

    CheckSpeedLimit();

    aimpoint.point.x = aimpoint.point.y = 0;    
    aimpoint.header = msg->header;
    aimpoint.header.stamp = ros::Time();
    if (localpath.poses.size() > 1)
    {
        aimpoint.point = (localpath.poses.end() - 1)->pose.position;
    }
    else
    {
        aimpoint.header.frame_id = "";
        localpath.poses.clear();
        self_turn_ctr = turnmode_ctr = 0;
        return;
    }

    float dd = 0;
    for (auto it = localpath.poses.begin(); it != localpath.poses.end() - 2; ++it) // 寻找预瞄点
    {
        float ds = GetDistanceXY(it->pose.position, (it + 1)->pose.position);
        dd += ds;
        if (dd >= aim_range)
        {
            aimpoint.point = it->pose.position; // float wheel_err_front_external = 999, wheel_err_rear_external = 999;
            break;
        }
    }

    if (dd < aim_range)
    {
        // geometry_msgs::PoseStamped target_			char xx=0xff;pose = *(localpath.poses.end() - 2);
        float L1 = aim_range - dd + 0.05; //  最后引导距离
        geometry_msgs::PoseStamped p1 = *(localpath.poses.end() - 1);
        geometry_msgs::PoseStamped p2 = *(localpath.poses.end() - 2);
        p1.pose.orientation = GetQuaternionMsgByPoints(p2.pose.position, p1.pose.position);
        geometry_msgs::PoseStamped p = GetExtendPoseByPose(p1, L1);
        aimpoint.point = p.pose.position;
    }

    ref_speed = localpath.poses[0].pose.position.z; //  速度为首点Z数据
    if (ref_speed > speedlimit)  ref_speed = speedlimit;

    float speedlimit_curve=SpeedLimitByCurve(13);
    if(ref_speed>speedlimit_curve)  ref_speed=speedlimit_curve;

    // printf("vel0=%.2f\n", localpath.poses[0].pose.positio    
    // printf("vel1=%.2f\n", localpath.poses[1].pose.position.z);
}

void TPathTrack::CarStateCallback(const car_ctr::car_state::ConstPtr &msg) //  接收车体状态信息
{
    cur_carstate = *msg;
}

int TPathTrack::SelfTurnCtr() //  自身转动控制   
{        
    int set_movemode = 0;   
    if(move_dir=="left" || move_dir=="right")  set_movemode=3;

    static float last_angle_err = 0;
    last_angle_err = track_angle_err;
    
    // printf("track_angle_err=%.2f\n",track_angle_err);

    static TTimer turn_tmr;
    float selfturn_angle_err=60;
    nh_local->getParam("selfturn_angle_err", selfturn_angle_err);
        
    if (self_turn_ctr == 0 && fabs(track_angle_err) > selfturn_angle_err && speedlimit > 0.1 )
        self_turn_ctr = 1, turn_tmr.Clear();

    // if(self_turn_ctr) ROS_INFO("%.1f  %d\n", track_angle_err, self_turn_ctr);

    if (self_turn_ctr == 0)  return 0;
    turnmode_ctr = 0;
 
    car_ctr.speed = car_ctr.angle = 0;
    if (self_turn_ctr == 1) // 停止运动
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        if (turn_tmr.GetValue() > 0.2 && fabs(cur_carstate.speed[0]) < 0.01)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 2) // 设置自转动
    {
        car_ctr.turnmode = 2;
        if (turn_tmr.GetValue() > 0.2 && cur_carstate.turnmode == 2)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 3) //  转动至角度误差小于阈值
    {
        float max_err = 0.1;      // 最小误差角（由于线性状态机的继续前进的判断有误差变号条件，不需要依靠较大的最小误差保证状态机前进）
        float slow_down_err = 30; // 开始减速的误差角

        // 最大旋转速度             //////////////
        float turn_speed_min = 0.03; // 最小旋转速度
        car_ctr.turnmode = 2;        // 设置自传模式

        float err_abs = fabs(track_angle_err); // 误差的绝对值
        // ROS_INFO("err_angle=%.2f", track_angle_err);
        if (err_abs > max_err && err_abs < slow_down_err) // 在开始减速的误差角度和最小误差角之间，速度线性地由最大转向速度递减为最小转向速度
            car_ctr.speed = turn_speed_min + (err_abs - max_err) / (slow_down_err - max_err) * (turn_speed_max - turn_speed_min);
        else if (err_abs > slow_down_err) // 在开始减速误差角之外采用最大旋转速度
            car_ctr.speed = turn_speed_max;
        else if (err_abs <= max_err || last_angle_err * track_angle_err < 0)
        {    
            car_ctr.speed = 0;
            self_turn_ctr++, turn_tmr.Clear();
        }    

        if (car_ctr.speed > speedlimit)  car_ctr.speed = speedlimit;

        if (track_angle_err < 0) // 如果是反转加负号
            car_ctr.speed *= -1;
        if (ref_speed < 0)
            car_ctr.speed *= -1;

        // ROS_INFO("err=%.1f  self_turn_ctr=%d\n", track_angle_err, self_turn_ctr);    
    }
    else if (self_turn_ctr == 4) // stop turn
    {
        car_ctr.turnmode = 2;
        if (turn_tmr.GetValue() > 0.5)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 5) // 设置为阿克曼转向
    {
        car_ctr.turnmode = set_movemode;
        if (turn_tmr.GetValue() > 2 && car_ctr.turnmode == cur_carstate.turnmode)
            self_turn_ctr = 0;
    }

    // printf("%d\n", car_ctr.turnmode);
    PubCarCtr(car_ctr);

    return self_turn_ctr;
}

void TPathTrack::UpdateCtrParam(float vel) //  根据速度规划预瞄点和转向控制系数
{
    vel = fabs(vel);
    if (vel > 5 )  // speedlimit < 1)
        aim_range = 20, steering_property = 0.6;
    else if (vel > 4.7)
        aim_range = 15, steering_property = 0.7;
    else if (vel > 3.6)
        aim_range = 15, steering_property = 0.8;
    else if (vel > 2.8)
        aim_range = 15, steering_property = 1.0;
    else if (vel > 1.5)
        aim_range = 10, steering_property = 1.2;
    else if (vel > 0.9)
        aim_range = 4, steering_property = 2;
    else 
        aim_range = 3, steering_property = 4;

    if (cur_carstate.turnmode == 4)  aim_range=5;
    // printf("speedlimit=%.2f\n",speedlimit);

    //  预瞄距离滤波
    static TDataFilter f_aimrange(40);
    aim_range=f_aimrange.GetValue(aim_range);
}   

float TPathTrack::GetAimAngleErr(geometry_msgs::PointStamped aimp, string move_dir) //  根据横移状态和运动速度判断跟踪角度误差
{
    float err=0;
    if(aimp.header.frame_id=="" || move_dir=="")  return err;
    
    geometry_msgs::PointStamped dst_p;
    transformPoint("base_link", aimp, dst_p, "");

    if(move_dir=="front")  err = atan2(dst_p.point.y, dst_p.point.x);
    else if(move_dir=="back")  err = atan2(dst_p.point.y, -dst_p.point.x);
    else if(move_dir=="left")  err = -atan2(dst_p.point.x, dst_p.point.y);
    else if(move_dir=="right")  err = -atan2(dst_p.point.x, -dst_p.point.y);

    err*=180/M_PI;
}

string TPathTrack::CheckMoveDir(nav_msgs::Path path, int id)
{
    string move_dir="";
    if (localpath.poses.size()<2 || id+1>=path.poses.size())  return move_dir;

    geometry_msgs::PoseStamped p1 = path.poses[0];
    geometry_msgs::PoseStamped p2 = path.poses[1];
    float pose_angle = GetYawFromPose(p1);
    float path_angle = GetAngleByPoints(p1.pose.position, p2.pose.position);
    float anglex = (path_angle - pose_angle) * 180 / M_PI;

    // printf("%.2f %.2f %.2f\n", pose_angle * 180 / M_PI, path_angle * 180 / M_PI,  anglex);

    if (anglex > 180)  anglex -= 360;
    else if (anglex < -180)  anglex += 360;

    if (fabs(anglex) > 160)   move_dir="back";  // ref_speed = -fabs(ref_speed)   
    else if (abs(anglex) < 20)   move_dir="front";  // ref_speed = fabs(ref_speed),
    else if (fabs(anglex + 90) < 20)  move_dir="right"; //  ref_speed = -fabs(ref_speed)
    else if (fabs(anglex - 90) < 20)  move_dir="left";  // ref_speed = fabs(ref_speed)

    return move_dir;
}

int TPathTrack::MoveModeCtr() // 切换turnmode
{
    int set_turnmode = 0;
    if(move_dir=="left" || move_dir=="right") set_turnmode=3;
    
    // printf("hengyi=%d\n",set_turnmode);

    static TTimer ctr_tmr;
    // printf("set_turnmode=%d self_turn=%d turn_ctr=%d\n", set_turnmode, self_turn_ctr, turnmode_ctr);

    if (turnmode_ctr == 0 && self_turn_ctr == 0 && set_turnmode != cur_carstate.turnmode)
        turnmode_ctr = 1, ctr_tmr.Clear();

    // ROS_INFO("%d %d %d %d\n", set_turnmode, cur_carstate.turnmode, turnmode_ctr, self_turn_ctr);
    if (turnmode_ctr == 0)
        return 0;

    // ROS_INFO("turnmode_ctr=%d\n", turnmode_ctr);

    car_ctr.speed = car_ctr.angle = 0;
    if (turnmode_ctr == 1) // 停止运动
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        if (ctr_tmr.GetValue() > 0.2 && fabs(cur_carstate.speed[0] < 0.01))
            turnmode_ctr++, ctr_tmr.Clear(); // 等待停止
    }
    if (turnmode_ctr == 2) // 设置turnmode
    {
        car_ctr.turnmode = set_turnmode;
        // printf("turnmode: state=%d  set=%d\n", cur_carstate.turnmode, car_ctr.turnmode);
        if (set_turnmode == cur_carstate.turnmode)
            turnmode_ctr = 0;
    }
    PubCarCtr(car_ctr);
    // carctr_pub.publish(car_ctr);
    return turnmode_ctr;
}

int TPathTrack::StopCtr() //  停车控制
{
    int stop_flag = StopCheck();
    if (stop_flag)
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        car_ctr.speed = car_ctr.angle = 0;
        PubCarCtr(car_ctr);
        // printf("stop_flag=%d ----car_ctr: %d\n", stop_flag, car_ctr.workmode);
        // carctr_pub.publish(car_ctr);
    }

    return stop_flag;
}

int TPathTrack::FaultCheck()
{
    static TTimer tmr;
    string errstr="";
    nh->getParam("/iot_comm/err", errstr);
    if(errstr=="") tmr.Clear();  
    
    if(tmr.GetValue()>3)  return 1;
    else return 0;
}

int TPathTrack::StopCheck() //  综合判断是否停车  1 传感器与节点故障 2 局部路径 3 运动使能 4 抱夹作业 5 雷达定位
{
    stop_str="";
    if(FaultCheck())
    {
        stop_str = "fault stop";
    }

    // ROS_INFO("%d", localpath.poses.size());
    if (localpath.poses.size() == 0) //  无有cur_task.final_path效路径
    {
        stop_str = "no path";
    }

    if (!run_enable) //  自动导航断使能
        stop_str = "run disable";

    // // 夹爪作业
    // if (paw_state == "car_stop" || paw_state == "length_change_start" || 
    //     paw_state == "length_change_done" || paw_state == "baojia_start" || 
    //     (paw_state == "paw_dropping" && agv_in_flag)) // || paw_state_msg.paw_state==2)  //  夹爪在张合过程中不能动
    //     stop_str = "paw working";

    if (cur_carstate.ctrmode == 0) //  手动按钮
        stop_str = "manual";

    if (speedlimit<0.01)
    {
        if(obstacle_speedlimit<0.01)   stop_str="obstacle_stop";
    }

    if (stop_str != "")  return 1;
    else  return 0;
}

void TPathTrack::CheckObsSpeedLimit()
{
    obstacle_speedlimit = 999;
    if(!obs_stop_enable)  return;

    // printf("move_dir=%s\n",move_dir.c_str());
    // if (cur_carstate.turnmode == 2) //  自转运动
    // {
    //     obstacle_speedlimit = min(right_speedlimit,left_speedlimit);
    //     float front_speedlimit=min(front_middle_speedlimit,front_side_speedlimit);
    //     float back_speedlimit=min(back_middle_speedlimit,back_side_speedlimit);
    //     obstacle_speedlimit = min(obstacle_speedlimit,front_speedlimit);
    //     obstacle_speedlimit = min(obstacle_speedlimit,back_speedlimit);
    //     // printf("selfturn_obstacle_speedlimit=%.3f\n", obstacle_speedlimit);
    // }
    // else if (move_dir == "front" || move_dir=="back") //  前向运动
    // {
    //     float middle_speedlimit, side_speedlimit;
    //     if(move_dir=="front")  middle_speedlimit=front_middle_speedlimit,  side_speedlimit=front_side_speedlimit;
    //     else middle_speedlimit=back_middle_speedlimit,  side_speedlimit=back_side_speedlimit;

    //     if(iterative_ctr_flag>0 && car_ctr.speed>0)  middle_speedlimit=front_middle_speedlimit,  side_speedlimit=front_side_speedlimit;
    //     else if(iterative_ctr_flag>0 && car_ctr.speed<0)   middle_speedlimit=back_middle_speedlimit,  side_speedlimit=back_side_speedlimit;
    //         // printf("%s middle=%.2f\n",move_dir.c_str(), middle_speedlimit);
    //         // printf("remain_path=%.2f\n",remain_path_length);
    //         //  取车最后路程, AGV无车,速度受限边沿障碍物
    //     if(cur_task.cmd=="pick task" && remain_path_length<12 && cur_task.final_path)  
    //     {
    //         obstacle_speedlimit=side_speedlimit;//20231120
    //         // printf("speedlimit=%.2f\6n",obstacle_speedlimit);
    //         static TTimer tmr;
    //         if(obstacle_speedlimit>0.05)  tmr.Clear();
    //         else if(tmr.GetValue()>2 && iterative_ctr_enable && paw_state_msg.paw_state==0)  iterative_ctr_flag=1; 
    //         // printf("ctr_flag=%d\n",iterative_ctr_flag);
    //     }
    //     //  密停放车最后路程, AGV有车, 速度受限密停距离和边沿障碍物    
    //     else if(cur_task.cmd=="release task" && cur_task.subcmd=="1" && remain_path_length<12 && cur_task.final_path)
    //         obstacle_speedlimit=min(mistop_speedlimit, side_speedlimit);
    //     else 
    //         obstacle_speedlimit=min(middle_speedlimit, side_speedlimit);      

    //     if(fabs(car_ctr.angle)>10)  //  大转向考虑左右障碍物
    //     {
    //         obstacle_speedlimit=min(obstacle_speedlimit, left_speedlimit);
    //         obstacle_speedlimit=min(obstacle_speedlimit, right_speedlimit);
    //     }    
    // }
    // else if (move_dir == "left") //  左侧横移运动
    // {
    //     obstacle_speedlimit = left_speedlimit;
    // }
    // else if (move_dir == "right") //  右侧横移运动
    // {
    //     obstacle_speedlimit = right_speedlimit;
    // }

    // printf("obs=%.2f\n",obstacle_speedlimit);
    if(obstacle_speedlimit<0.04) obstacle_speedlimit=0;
}

void TPathTrack::CheckSpeedLimit()
{
    // printf("speedlimit=%.2f\n", mistop_speedlimit);
    speedlimit = 999;
    if (speedlimit >= work_speedlimit)  speedlimit = work_speedlimit;

    // ROS_INFO("mode=%d SPD=%.2f",car_ctr.turnmode, car_ctr.speed);
    CheckObsSpeedLimit();   
    if (speedlimit >= obstacle_speedlimit)  
    {
        speedlimit = obstacle_speedlimit;
        // printf("obs=%.2f\n",obstacle_speedlimit);
        std_msgs::Float64 obs_dis;
        obs_dis.data=obstacle_speedlimit; 
        obs_speedlimit_pub.publish(obs_dis);
    }
    if (speedlimit < 0.001)  speedlimit = 0.001;
    // printf("speedlimit=%.2f obs_speedlimit=%.2f  work_speedlimit=%.2f\n", speedlimit, obstacle_speedlimit, work_speedlimit);
}


// 纯轨迹跟踪控制
void TPathTrack::PurePursuit(car_ctr::car_ctr &car_ctr)
{
    car_ctr.turnmode = cur_carstate.turnmode;
    car_ctr.speed = ref_speed;
    if(move_dir=="back" || move_dir=="right")  car_ctr.speed*=-1;
    car_ctr.angle = steering_property*track_angle_err;
    // printf("car_ctr=%.2f\n",car_ctr.angle);
}

void TPathTrack::Run() //  主运行函数
{
    nodecheck->Find("node_rate")->Beat();
    if(CheckTrackErr()<0.5)  nodecheck->Find("track_rate")->Beat();

    move_dir=CheckMoveDir(localpath);
    track_angle_err=GetAimAngleErr(aimpoint, move_dir);

    if (!test_flag)   //  正常模式
    {
        nh_local->getParam("run_enable", run_enable);
        nh_local->getParam("obs_stop_enable", obs_stop_enable);
        nh_local->getParam("turn_speed_max", turn_speed_max);

        static TTimer tmr;

        std_msgs::String move_msgs;
        move_msgs.data=move_dir;
        move_dir_pub.publish(move_msgs);

        if(tmr.GetValue()>0.5)  // 降低写参数耗时
        {
            tmr.Clear();
            nh_local->setParam("stop_reason", stop_str);
            nh_local->setParam("move_dir", move_dir);
        }
        // printf("%x\n", stop_code);

        UpdateCtrParam(ref_speed);
        if (localpath.poses.size())  aimpoint_pub.publish(aimpoint);

        if (SelfTurnCtr())
        {
            stop_str="self turning";
            return;
        }

        if (cur_task.cmd.find("task") != string::npos && MoveModeCtr())
        {
            stop_str="movemode changing";
            return;
        }
        if (StopCtr())  return;

        stop_str="";
        PurePursuit(car_ctr);      // 纯跟踪控制
    }
    else  // 开环测试模式
    {
        car_ctr.turnmode = 0;
        car_ctr.angle = 0;
        car_ctr.speed = test_speed;
    }
    
    PubCarCtr(car_ctr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathtrack");
    TPathTrack pathtrack;

    ros::Rate rate(20);
    while (ros::ok())
    {
        pathtrack.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
