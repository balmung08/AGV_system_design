#include <mqtt_comm/path_point_action.h>
#include <common/public.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <mqtt_comm/task.h>
#include <common/mydisplay.h>
#include <sys/stat.h>

using namespace std;


class TWorkCtr
{
private:
    ros::NodeHandle *nh;
    ros::Subscriber task_sub, turntable_sub;
    ros::Publisher turntablectr_pub, work_speedlimit_pub;

    float remain_path_length=0;
    mqtt_comm::task cur_task;
    TNodeCheck *nodecheck;
    vector<mqtt_comm::path_point_action> actions;
    float act_pitch_angle, act_azimuth_angle;
    string task_id="ABC001", pos_id="";
    char savepath[200];

public:
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void TurntableCallback(const std_msgs::String::ConstPtr &msg);

    void FindPoseAction();
    void Run();

    TWorkCtr()
    {
        nh = new ros::NodeHandle("~");

        nodecheck=new TNodeCheck(nh, "node_rate");
	    nodecheck->Find("node_rate")->SetLimit(15);

        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TWorkCtr::TaskCallback, this);
        turntable_sub = nh->subscribe<std_msgs::String>("/turntable/table_state", 10, &TWorkCtr::TurntableCallback, this);
        
        turntablectr_pub = nh->advertise<std_msgs::String>("/turntable_ctr", 10);
        work_speedlimit_pub = nh->advertise<std_msgs::Float32>("speedlimit", 10);

        nh->setParam("/work_state","work_done");
    }
};


//  接收任务指令
void TWorkCtr::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    cur_task=*msg;
    task_id=cur_task.taskId;
}

void TWorkCtr::TurntableCallback(const std_msgs::String::ConstPtr &msg)
{
    vector<string> ss = split(msg->data, ";");
    for (int i = 0; i < ss.size(); i++)
    {
        vector<string> subs = split(ss[i], " ");
        if (subs[0] == "pitch")
        {
            act_pitch_angle = atof(subs[2].c_str());
            // reached_flag = atoi(subs[3].c_str());
        }
        else if (subs[0] == "azimuth")
        {
            act_azimuth_angle = atof(subs[2].c_str());
            // reached_flag = atoi(subs[3].c_str());
        }
    }

    // ROS_INFO("%.2f %.2f", act_pitch_angle, act_azimuth_angle);
}

// 寻找最近作业点
void TWorkCtr::FindPoseAction()
{
    if(actions.size()>0 || cur_task.path.size()==0)  return;

    for(auto &it:cur_task.path)
        if(it.actions.size()>0)
        {
            geometry_msgs::PointStamped p_map, p_base;
            p_map.header.frame_id="map";
            p_map.header.stamp=ros::Time::now();
            p_map.point.x=it.pointX, p_map.point.y=it.pointY;
            transformPoint("base_link", p_map, p_base, "AAA");

            if(p_base.point.x<0.02 && fabs(p_base.point.x)<0.1 && fabs(p_base.point.y)<0.1)
            // if( fabs(p_base.point.x)<0.5 && fabs(p_base.point.y)<0.5)   //根据跟踪定位精度修改
            {
                actions=it.actions;
                it.actions.clear();
                // ROS_INFO("action size=%d", actions.size());
                pos_id=it.caption;
                break;
            }
        }
}

void TWorkCtr::Run()
{
    nodecheck->Find("node_rate")->Beat();

    FindPoseAction();

    std_msgs::Float32 speedlimit_msg;
    if(actions.size()>0)  
    {
        speedlimit_msg.data=0;
        
        mqtt_comm::path_point_action action=actions.front();
        float pitch_angle=action.params[0];
        float azimuth_angle=action.params[1];
        float duration=action.params[2];
        
        static int action_flag=0;
        static TTimer act_tmr;
        if(action_flag==0)
        {
            char strbuf[500] = {0};
            sprintf(strbuf, "PITCH MOV %.2f 10;AZIMUTH MOV %.2f 10", pitch_angle, azimuth_angle);
            
            std_msgs::String str_msg;
            str_msg.data = strbuf;
            turntablectr_pub.publish(str_msg);

            action_flag++;  act_tmr.Clear();
            // ROS_INFO("%s start", action.caption.c_str());
            sprintf(savepath,"/home/bit/save_data");
            mkdir(savepath,S_IRWXU);
            sprintf(savepath,"%s/%s",savepath,task_id.c_str());
            mkdir(savepath,S_IRWXU);
            sprintf(savepath,"%s/%s",savepath,pos_id.c_str());
            mkdir(savepath,S_IRWXU);
            nh->setParam("/save_file_path",savepath);
            mkdir(savepath,S_IRWXU);
        }
        else if(action_flag==1 && fabs(pitch_angle-act_pitch_angle)<0.2 && fabs(azimuth_angle-act_azimuth_angle)<0.2)
        {
            action_flag++;  act_tmr.Clear();
            if(actions.size()>1)  
            {
               nh->setParam("/work_state","start_work");
               char cmd[400];
               sprintf(cmd,"ffmpeg -y -f alsa -i sysdefault:CARD=Device -t %.1f %s/audio.wav",duration, savepath);
               system(cmd);
            //    ROS_INFO("work start!");
            }
        }
        else if(action_flag==2)
        {
            //  此处加入作业控制代码
            if(act_tmr.GetValue()>duration)  
            {
                // ROS_INFO("%s stop", action.caption.c_str());
                action_flag=0;
                if(actions.size()>1)  
                {
                    nh->setParam("/work_state","work_done");
                    // ROS_INFO("work done!");
                }
                actions.erase(actions.begin());
            }
        }
    }
    else
    {
        speedlimit_msg.data=999;
    }

    work_speedlimit_pub.publish(speedlimit_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "work_ctr");

    TWorkCtr workctr;
    ros::Rate looprate(20);
    while (ros::ok())
    {
        workctr.Run();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};