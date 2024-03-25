#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <yaml-cpp/yaml.h>

#include <common/public.h>
#include <common/mydisplay.h>
#include "geometry_msgs/PoseStamped.h"
#include "mqtt_comm/mqtt_controls.h"
#include "mqtt_comm/path_point.h"
#include "mqtt_comm/task.h"
#include "mqtt_comm/mqtt_task.h"

#include "mqtt_comm/resp_agvstate.h"
#include "mqtt_comm/resp_task.h"
#include "mqtt_comm/resp_video.h"
#include "mqtt_comm/resp_ctrl.h"

#include <gps/MyGPS_msg.h>
#include <car_ctr/car_state.h>
#include <car_ctr/car_ctr.h>

using namespace std;

ros::NodeHandle *nh;
ros::Publisher resp_agvstate_pub, resp_task_pub, resp_video_pub, resp_ctrl_pub;
ros::Publisher task_pub;
mqtt_comm::task task_msg;
TNodeCheck *nodecheck;

string agvId = "123";
mqtt_comm::resp_agvstate agvstate_msg;
float remain_path_length=999;

void matchingLocCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(geo_quat, tf_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw); //四元数转欧拉角
    yaw *= (180 / M_PI);

    agvstate_msg.pointHA=yaw;
    agvstate_msg.pointX=msg->pose.pose.position.x;
    agvstate_msg.pointY=msg->pose.pose.position.y;
}

void CarStateCallback(const car_ctr::car_state::ConstPtr &msg) //  运动模式
{
    agvstate_msg.steerControlMode = msg->turnmode;
    agvstate_msg.autoDriveEnable = msg->ctrmode;
    agvstate_msg.vehCtrlMode = msg->ctrmode;
    agvstate_msg.vehSpeed = msg->speed[0];
    // agvstate_msg.cargoloadingStatus = msg->holdcar;
    // agvstate_msg.vehSpeed=msg->speed[0]*3.6;
    // if(msg->turnmode==2)  agvstate_msg.vehSpeed=0.2;   //  在自转中给定虚拟速度
}

void CarCtrCallback(const car_ctr::car_ctr::ConstPtr &msg) // 接收车辆控制指令
{
}

// void BatteryCallback(const data_comm::battery_info::ConstPtr &msg) //  电池信息
// {
//     agvstate_msg.batterySOC = msg->SOC;
//     agvstate_msg.batterySOH = msg->SOH;
//     agvstate_msg.batteryCurrent = msg->current;
//     agvstate_msg.batteryVoltage = msg->voltage;

//     // printf("%.0f %.0f %.1f %.1f\n", agvstate_msg.batterySOC, agvstate_msg.batterySOH, agvstate_msg.batteryCurrent, agvstate_msg.batteryVoltage);
// }

void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // remain_path_length=msg->data;
    // agvstate_msg.remain_path=msg->data;
    // ROS_INFO("%.2f", remain_path_length); 
}

void Pub_AgvState() //  发布车辆状态
{
    agvstate_msg.msgType = "agvinfo";
    // agvstate_msg.agvId = agvId;
    agvstate_msg.timestamp = ros::Time::now().toSec() * 1000;
    
    // float task_runtime=(ros::Time::now()-task_msg.stamp).toSec();  // 任务运行时间
    // if (task_msg.cmd == "pick task")
    // {
    //     if (paw_state_str == "wait_for_paw_drop")
    //         agvstate_msg.taskStatus = 2; // 自动导航中
    //     else if (paw_state_str == "paw_dropping")
    //         agvstate_msg.taskStatus = 5; // 自动取车中
    //     else if (paw_state_str == "baojia_done" && task_runtime>4)
    //     {
    //         agvstate_msg.taskStatus = 15; // 取车完成
    //     }
    // }
    // else if (task_msg.cmd == "release task")
    // {
    //     if (paw_state_str == "baojia_done")
    //         agvstate_msg.taskStatus = 2; // 自动导航中
    //     else if (paw_state_str == "car_dropping")
    //         agvstate_msg.taskStatus = 8; // 自动放车中
    //     else if (paw_state_str == "wait_for_paw_drop" && task_runtime>4)
    //     {
    //         agvstate_msg.taskStatus = 15; // 放车完成 
    //     }
    // }
    // else if (task_msg.cmd == "move task")
    // {
    //     if(remain_path_length<0.01 && task_runtime>4)  agvstate_msg.taskStatus=15; // 完成
    //     else agvstate_msg.taskStatus=2;
    //     // ROS_INFO("%.2f", remain_path_length);
    // }

    // // nh->getParam("/rfid_reader/cargoVin", agvstate_msg.cargoVin);
    // // if(car_state.carhold_state==0x04 && agvstate_msg.cargoVin.find("e200")!=string::npos) 
    // //     agvstate_msg.cargoloadingStatus=1;
    // // else 
    // //     agvstate_msg.cargoloadingStatus=0;
    
    // // ROS_INFO("%d", agvstate_msg.cargoloadingStatus);

    // // int errcode=0, stopcode=0;
    // nh->getParam("/pathtrack/stop_code", agvstate_msg.stopcode);
    // nh->getParam("/err_code", agvstate_msg.errcode);


    resp_agvstate_pub.publish(agvstate_msg);
}

void TurntableCallback(const std_msgs::String::ConstPtr &msg)
{
    vector<string> ss = split(msg->data, ";");
    for (int i = 0; i < ss.size(); i++)
    {
        vector<string> subs = split(ss[i], " ");
        if (subs[0] == "pitch")
        {
            agvstate_msg.pitchAngle = atof(subs[2].c_str());
            // reached_flag = atoi(subs[3].c_str());
        }
        else if (subs[0] == "azimuth")
        {
            agvstate_msg.azimuthAngle = atof(subs[2].c_str());
            // reached_flag = atoi(subs[3].c_str());
        }
    }

    // ROS_INFO("%.2f %.2f", act_pitch_angle, act_azimuth_angle);
}


void mqttTaskCallback(const mqtt_comm::mqtt_task::ConstPtr &msg) //  回应信号
{
    // ROS_INFO("%s", );
}


// void Resp_Controls(mqtt_comm::controls m) //  回应信号
// {
    // printf("%s\n", m.msgType.c_str());

    // if (m.msgType == "task") //  回应任务信号
    // {
    //     mqtt_comm::resp_task msgx;
    //     msgx.agvId = agvId;
    //     msgx.msgType = "recvTask";
    //     msgx.timestamp = ros::Time::now().toSec() * 1000;
    //     msgx.taskId = m.taskId;
    //     msgx.subTaskIndex = m.subtaskIndex;
    //     resp_task_pub.publish(msgx);

    //     printf("任务反馈=%s\n", m.msgType.c_str());
    // }
    // else if (m.msgType == "videoControl") //  回应视频信号
    // {
    //     mqtt_comm::resp_video msgx;
    //     msgx.agvId = agvId;
    //     msgx.msgType = "videoCtrlResp";
    //     msgx.timestamp = ros::Time::now().toSec() * 1000;
    //     msgx.result = 1;
    //     resp_video_pub.publish(msgx);

    //     // printf("%s\n", msgType.c_str());
    // }
    // else if (m.msgType == "ctrl") //  回应控制信号
    // {
    //     mqtt_comm::resp_ctrl msgx;
    //     msgx.agvId = agvId;
    //     msgx.msgType = "respCtrl";
    //     msgx.cmdId = m.cmdId;
    //     msgx.timestamp = ros::Time::now().toSec() * 1000;
    //     msgx.ctrlType = m.ctrlType;
    //     resp_ctrl_pub.publish(msgx);

    //     // printf("%s\n", m.ctrlType.c_str());
    // }
    // else if (m.msgType == "sync")
    // {
    //     nodecheck->Find("node_rate")->Beat();
    //     // ROS_INFO("beat\n");
    // }
// }

// void Controls_Proc(mqtt_comm::controls m) //  处理控制指令
// {
//     if (m.msgType == "task") //  处理任务指令
//     {
//         // mqtt_comm::task task_msg;
//         if (m.taskType == 2)
//             task_msg.cmd = "pick task";
//         else if (m.taskType == 3)
//             task_msg.cmd = "release task";
//         else if (m.taskType == 10)
//             task_msg.cmd = "move task";
//         else if (m.taskType == 8)
//             task_msg.cmd = "charge task";
//         else if (m.taskType == 11)  //  表演运动
//         {
//             task_msg.cmd = "none task";
//             nh->setParam("/show_action_enable", true);
//         }
//         else if (m.taskType==12 || m.taskType==13)  //  表演取车 退出
//         {
//             if(m.taskType==13)  task_msg.cmd = "move task";
//             else if(m.taskType==12)  task_msg.cmd = "pick task";
//             mqtt_comm::path_point p;
//             p.pointHA = m.targetHA;
//             p.pointX = m.targetX;
//             p.pointY = m.targetY;
//             m.path.clear();
//             m.path.push_back(p);
//         }
//         else if (m.taskType==15)   
//         {
//             task_msg.cmd = "move task";
//             m.path.clear();
//         }
//         else
//             task_msg.cmd = "none task",  m.path.clear();

//         remain_path_length=999;    
//         // printf("%s pathlength=%d\n", task_msg.cmd.c_str(), m.path.size());

//         task_msg.stamp = ros::Time::now();
//         task_msg.path = m.path;
//         for (auto &p : task_msg.path)
//         {
//             if (fabs(p.pointX) > 100000 || fabs(p.pointY) > 100000) //  come from diaodu
//             {
//                 p.pointX -= utm_x_zero, p.pointY -= utm_y_zero;
//                 p.pointHA += 90;
//                 if (p.pointHA >= 180)  p.pointHA -= 360;
//                 else if (p.pointHA < -180)   p.pointHA += 360;
//                 // printf("%.2f %.2f %.2f %.2f\n", p.pointX, p.pointY, p.pointHA, p.vehSpeed);

//                 p.vehSpeed/=3.6;  
//             }
//             (task_msg.path.end()-1)->vehSpeed = 0.2;
//             next_task_path_id=0;
//         }

//         if(m.taskType==12)  UpdatePathByShow("pick show", task_msg);
//         else if(m.taskType==13)  UpdatePathByShow("return show", task_msg);
//         else if(m.taskType==15)  UpdatePathByShow("move front", task_msg);
//         // printf("%d\n",m.taskType);

//         task_pub.publish(task_msg);

//         if(m.path.size()>0)
//         {
//             agvstate_msg.targetX = m.path.back().pointX;
//             agvstate_msg.targetY = m.path.back().pointY;
//             agvstate_msg.taskType = m.taskType;
//             agvstate_msg.taskId = m.taskId;
//         }
//     }

//     if (m.msgType == "ctrl") //  处理控制指令
//     {
//         // ROS_INFO("%s", m.msgType.c_str());
//         mqtt_comm::task ctr_msg;
//         ctr_msg.cmd=m.ctrlType+" ctrl";
//         ctr_msg.stamp=ros::Time::now();
//         if(m.ctrlType=="trafficLight")
//         {
//             if(m.lightStatus==1)  ctr_msg.subcmd="green";
//             else ctr_msg.subcmd="red";
//         }
//         else if(m.ctrlType=="speedLimit")
//         {
//             mqtt_comm::path_point p;
//             p.vehSpeed=m.value/3.6;
//             ctr_msg.path.push_back(p);

//             agvstate_msg.speedLimit=m.value;
//         }
//         else if(m.ctrlType=="taskStatus")
//         {
//             char buf[10];
//             sprintf(buf, "%d", m.value);
//             ctr_msg.subcmd=buf;
//         }
//         else if(m.ctrlType=="obstacleDetectionDisable")
//         {
//             char buf[10];
//             sprintf(buf, "%d", m.value);
//             ctr_msg.subcmd=buf;
//         }
//         task_pub.publish(ctr_msg);
//     }
// }

void controlsCallback(const mqtt_comm::mqtt_controls::ConstPtr &msg)
{
    // ROS_INFO("value=%d", msg->value);
    nh->setParam("/pathtrack/run_enable", msg->value==1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mqtt_comm");
    nh = new ros::NodeHandle("~");

    // nh->getParam("/agvId", agvId);
    // ROS_INFO("agvId=%s", agvId.c_str());

    resp_agvstate_pub = nh->advertise<mqtt_comm::resp_agvstate>("/resp_agvstate", 10);
    // resp_task_pub = nh->advertise<mqtt_comm::resp_task>("/resp_task", 10);
    // resp_video_pub = nh->advertise<mqtt_comm::resp_video>("/resp_video", 10);
    // resp_ctrl_pub = nh->advertise<mqtt_comm::resp_ctrl>("/resp_ctrl", 10);
    // task_pub = nh->advertise<mqtt_comm::task>("/task_cmd", 10);

    ros::Subscriber mqtttask_sub = nh->subscribe<mqtt_comm::mqtt_task>("/mqtt_task", 10, mqttTaskCallback);
    ros::Subscriber matching_loc_sub = nh->subscribe<nav_msgs::Odometry>("/laser_localization", 10, matchingLocCallback);
    ros::Subscriber carstate_sub = nh->subscribe<car_ctr::car_state>("/car_state", 10, CarStateCallback);
    ros::Subscriber turntable_sub = nh->subscribe<std_msgs::String>("/turntable/table_state", 10, TurntableCallback);
    // ros::Subscriber battery_sub = nh->subscribe<data_comm::battery_info>("/can_comm/battery_info", 10, BatteryCallback);
    ros::Subscriber controls_sub = nh->subscribe<mqtt_comm::mqtt_controls>("/mqtt_controls", 10, controlsCallback);
    // ros::Subscriber paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, PawStateCallback);

    float check_duration=4;
    nodecheck = new TNodeCheck(nh, "node_rate",check_duration);
    nodecheck->Find("node_rate")->SetLimit(0.4);
    
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        Pub_AgvState(); //  发送状态心跳

        // ROS_INFO("1111");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
