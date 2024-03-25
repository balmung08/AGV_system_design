#include <ros/ros.h>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <common/public.h>
#include <common/mydisplay.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <mqtt_comm/task.h>
#include <mqtt_comm/path_point.h>
#include <std_msgs/Float32MultiArray.h>
#include <data_comm/car_state.h>

using namespace std;

const float last_path_precision = 0.01;

class TLocalPathPlan
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher taskpath_pub, globalpath_pub, localpath_pub, shotpose_pub, pathmarkers_pub, remainpath_pub, passedpath_pub;
    ros::Subscriber task_sub, rviz_goal_sub, carstate_sub;
    nav_msgs::Path task_path;   //  稀疏任务点
    nav_msgs::Path global_path; //  加密后的全局点(带速度规划,速度为正), 最后一段最密
    nav_msgs::Path local_path;  //  从global_path中抽取的局部路径
    geometry_msgs::PoseStamped shotpose;
    double utm_x_zero = 0, utm_y_zero = 0;

    int Nearest_ID = 0;
    mqtt_comm::task cur_task;
    TNodeCheck *nodecheck;
    float path_remain;
    float path_passed = 0;
    string paw_state;
    data_comm::car_state cur_carstate;

public:
    TLocalPathPlan()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        localpath_pub = nh_local->advertise<nav_msgs::Path>("localpath", 10);
        globalpath_pub = nh_local->advertise<nav_msgs::Path>("globalpath", 10);
        taskpath_pub = nh_local->advertise<nav_msgs::Path>("taskpath", 10);
        shotpose_pub = nh_local->advertise<geometry_msgs::PoseStamped>("target_pose", 10);
        pathmarkers_pub = nh_local->advertise<visualization_msgs::MarkerArray>("path_markers", 10);
        remainpath_pub = nh_local->advertise<std_msgs::Float64>("remainpath", 10);
        passedpath_pub = nh_local->advertise<std_msgs::Float64>("passedpath", 10);

        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TLocalPathPlan::TaskCallback, this);
        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &TLocalPathPlan::CarStateCallback, this);
        
        rviz_goal_sub = nh->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &TLocalPathPlan::RvizGoalCallback, this);

        task_path.header.frame_id = "map";
        global_path.header.frame_id = "map";
        local_path.header.frame_id = "map";

        cur_task.cmd = "none task";
        cur_task.stamp = ros::Time::now();

        nodecheck = new TNodeCheck(nh_local, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(15);
    };

    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void RvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);

    void GetGlobalPathFromTask(mqtt_comm::task task);
    void GlobalPathPlan();
    void LocalPathPlan();
    // nav_msgs::Path LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds);
    nav_msgs::Path LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds, float last_ds=-1);
    void SpeedPlan(nav_msgs::Path &path, float min_vel, float safe_dis = 0.3);
    bool CheckStopAtNextPoint(nav_msgs::Path path, int id, bool action_stop=false);
    void Run();
 
    nav_msgs::Path GenFinalPathByPose(geometry_msgs::PoseStamped pose);
    void PubPathMarkers();
    void UpdateLaneType();
};

//  发布路径速度与方向标识 task_path
void TLocalPathPlan::PubPathMarkers()
{
    if (task_path.poses.size() == 0)  return;

    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;
    marker.header = task_path.header;
    marker.ns = "path_marker";
    marker.lifetime = ros::Duration(0.2);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.pose.position.z = 2;
    marker.scale.z = 1;

    marker.id = 0;
    for (int i = 0; i < task_path.poses.size(); i++)
    {
        marker.id++;
        marker.pose = GetExtendPoseByPose(task_path.poses[i], 1.5).pose;
        char text[100];
        if (i < task_path.poses.size() - 1)
            sprintf(text, "%.1f", task_path.poses[i].pose.position.z);
        else
            sprintf(text, "target");
        marker.text = text;
        markerarray.markers.push_back(marker);
    }

    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 1.5;
    marker.scale.y = 0.5;
    // marker.color.r=marker.color.g=1.0f;  marker.color.b=0.0f;
    for (int i = 0; i < task_path.poses.size() - 1; i++)
    {
        marker.id++;
        geometry_msgs::PoseStamped p1 = task_path.poses[i];
        geometry_msgs::PoseStamped p2 = task_path.poses[i + 1];
        marker.pose.position.x = (p1.pose.position.x + p2.pose.position.x) * 0.5;
        marker.pose.position.y = (p1.pose.position.y + p2.pose.position.y) * 0.5;

        marker.pose.position.z = 1;
        marker.pose.orientation = GetQuaternionMsgByPoints(p1.pose.position, p2.pose.position);
        markerarray.markers.push_back(marker);
    }
    pathmarkers_pub.publish(markerarray);
}

//  在rviz里模拟目标点
void TLocalPathPlan::RvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mqtt_comm::task task;
    mqtt_comm::path_point p;
    task.cmd = "move task";
    task.stamp = ros::Time::now();

    geometry_msgs::PoseStamped p0_base, p0_map;
    p0_base.header.frame_id = "base_link";
    p0_base.pose.orientation.w = 1;
    transformPose("map", p0_base, p0_map);
    p.pointHA = GetYawFromPose(p0_map) * 180 / M_PI;
    p.pointX = p0_map.pose.position.x;
    p.pointY = p0_map.pose.position.y;
    p.vehSpeed = 0.4;
    task.path.push_back(p);

    p.pointHA = GetYawFromPose(*msg) * 180 / M_PI;
    p.pointX = msg->pose.position.x;
    p.pointY = msg->pose.position.y;
    p.vehSpeed = 1;
    task.path.push_back(p);

    GetGlobalPathFromTask(task);
    GlobalPathPlan();
}

//  根据task生成全局路径 task_path 重要函数
void TLocalPathPlan::GetGlobalPathFromTask(mqtt_comm::task task)
{
    Nearest_ID = 0;

    task_path.header.frame_id = "map";
    task_path.header.stamp = ros::Time::now();
    task_path.poses.clear();

    shotpose.header.frame_id = "", global_path.poses.clear();
    if (task.path.empty())  return;

    shotpose.header.frame_id = "map";
    shotpose.header.stamp = ros::Time::now();
    shotpose.pose.position.x = task.path.back().pointX;
    shotpose.pose.position.y = task.path.back().pointY;
    shotpose.pose.position.z = task.path.back().vehSpeed;
    shotpose.pose.orientation = tf::createQuaternionMsgFromYaw(task.path.back().pointHA * M_PI / 180);
    shotpose_pub.publish(shotpose); //  发布出去

    geometry_msgs::PoseStamped p_map;
    p_map.header.frame_id = "map";
    p_map.header.stamp = ros::Time::now();
    for (int i = 0; i < task.path.size(); i++) //  加入设置路径点
    {
        p_map.pose.position.x = task.path[i].pointX;
        p_map.pose.position.y = task.path[i].pointY;
        p_map.pose.position.z = task.path[i].vehSpeed;

        p_map.pose.orientation = tf::createQuaternionMsgFromYaw(task.path[i].pointHA * M_PI / 180);
        task_path.poses.push_back(p_map);
    }
}

//  接收任务指令
void TLocalPathPlan::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    // ROS_ERROR("%s\n", msg->cmd.c_str());
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        cur_task.stamp = ros::Time::now();

        GetGlobalPathFromTask(*msg);
        GlobalPathPlan();
    }
    else if(msg->cmd=="mission delete")
    {
        task_path.poses.clear();
        global_path.poses.clear();
        local_path.poses.clear();
    }
}

void TLocalPathPlan::CarStateCallback(const data_comm::car_state::ConstPtr &msg) //  接收车体状态信息
{
    cur_carstate = *msg;
}

//  根据单个目标点pose生成最后路径,仅在近距离有效
nav_msgs::Path TLocalPathPlan::GenFinalPathByPose(geometry_msgs::PoseStamped pose)
{
    float Lx=8;
    if(pose.pose.position.x>=0)  Lx=-8;
    else if (pose.pose.position.x<0) Lx=8;
    // printf("pose.pose.position.x=%.2f\n",pose.pose.position.x);

    transformPose("map", pose, shotpose, "CCC");
    geometry_msgs::PoseStamped px_map;

    nav_msgs::Path path;
    px_map = GetExtendPoseByPose(shotpose, Lx);
    path = LineInterpolation(px_map.pose.position, shotpose.pose.position, 0.03);//last_path_precision);
    
    // float angle_trunk=(cur_task.path.end()-1)->pointHA;
    // for(auto &p:path.poses)  p.pose.orientation=tf::createQuaternionMsgFromYaw(angle_trunk); 

    for(auto &p:path.poses)  p.pose.orientation=shotpose.pose.orientation; 
    // task_path.poses.clear();
    // task_path.poses.push_back(p1_map);
    // task_path.poses.push_back(p2_map);

    return path;
}

//  两点之间插值生成路径
nav_msgs::Path TLocalPathPlan::LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds, float last_ds)
{
    //  曲线插值加密, ds为间隔距离, 最后一段特别密
    float dd = GetDistanceXY(p1, p2);
    float vel = p1.z;
    int n = dd / ds;
    if (n < 1)  n = 1;

    float dx = (p2.x - p1.x) / n, dy = (p2.y - p1.y) / n;
    geometry_msgs::PoseStamped p;
    float angle = atan2(p2.y - p1.y, p2.x - p1.x);
    p.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    p.header = global_path.header;
    p.pose.position = p1;
    // printf("i=%d %.2f\n", i, angle*180/M_PI);

    nav_msgs::Path path;
    path.header = global_path.header;
    for (int i = 0; i < n - 1; i++)
    {
        p.pose.position.x = p1.x + dx * i;
        p.pose.position.y = p1.y + dy * i;
        p.pose.position.z = vel; //  z 表示运动速度
        path.poses.push_back(p);
    }

    if (last_ds > 0)
    {
        n = ds / last_ds;
        if (n > 1)
        {
            p1 = p.pose.position;
            dx = (p2.x - p1.x) / n, dy = (p2.y - p1.y) / n;
            for (int i = 1; i < n; i++)
            {
                p.pose.position.x = p1.x + dx * i;
                p.pose.position.y = p1.y + dy * i;
                p.pose.position.z = vel; //  z 表示运动速度
                path.poses.push_back(p);
            }
        }
    }
    return path;
}

//  检测是否停止
bool TLocalPathPlan::CheckStopAtNextPoint(nav_msgs::Path path, int id, bool action_stop)
{
    if(id>=path.poses.size()-2) return true;

    if(action_stop && cur_task.path[id+1].actions.size()>0) return true;
    if(cur_task.only_akm)  return false;

    //  检查运动模式是否改变
    if(CheckMoveDir(path, id)!=CheckMoveDir(path, id+1))
    {
        // ROS_INFO("STOP 1 at %d",id);
        return true;
    }

    float line_varangle_max = 40; //  直线允许最大变化角度

    //  检查车头朝向是否改变
    float heading1 = GetYawFromPose(path.poses[id]) * 180 / M_PI;
    float heading2 = GetYawFromPose(path.poses[id + 1]) * 180 / M_PI;
    float dangle=fabs(heading1 - heading2);
    if(dangle>=270)  dangle-=360;
    // ROS_INFO("%d--%d heading1=%.2f heading2=%.2f dangle=%.2f",id,id+1,heading1,heading2,dangle);
    if (fabs(dangle) > line_varangle_max)
    {
        // ROS_INFO("STOP 2 at %d heading1=%.2f heading2=%.2f",id,heading1,heading2);
        return true;
    }

    //  检查路径方向是否改变
    float line_angle1 = GetAngleByPoints(path.poses[id].pose.position, path.poses[id + 1].pose.position) * 180 / M_PI;
    float line_angle2 = GetAngleByPoints(path.poses[id + 1].pose.position, path.poses[id + 2].pose.position) * 180 / M_PI;
    dangle=fabs(line_angle1 - line_angle2);
    if(dangle>=270)  dangle-=360;
    // ROS_INFO("%d--%d line_angle1=%.2f line_angle2=%.2f dangle=%.2f",id,id+1,line_angle1,line_angle2,dangle);
    if (fabs(dangle) > line_varangle_max)
    {
        // ROS_INFO("STOP 3 at %d line_angle1=%.2f line_angle2=%.2f",id,line_angle1,line_angle2);
        return true;
    }

    return false;
}

//  速度规划
void TLocalPathPlan::SpeedPlan(nav_msgs::Path &path, float min_vel, float safe_dis)
{
    float acc_per_len=6.0/20, dec_per_len=-6.0/20;   //  速度/距离系数

    // 加速约束,从前往后
    if(fabs(cur_carstate.speed[0])>0.1)  path.poses.begin()->pose.position.z=fabs(cur_carstate.speed[0]);
    else  path.poses.begin()->pose.position.z = min_vel;
    
    for (int i = 1; i < path.poses.size(); i++)
    {
        float ref_vel = path.poses[i].pose.position.z;
        float cur_vel = path.poses[i - 1].pose.position.z;
        float ds = GetDistanceXY(path.poses[i].pose.position, path.poses[i - 1].pose.position);
        float vk = (ref_vel - cur_vel) / ds;

        if (vk > acc_per_len)
            vk = acc_per_len;
        else if (vk < dec_per_len)
            vk = dec_per_len;
        path.poses[i].pose.position.z = cur_vel + vk * ds;
    }

    // 减速约束,从后往前
    float len = 0;
    (path.poses.end() - 1)->pose.position.z = min_vel;
    for (int i = path.poses.size() - 2; i > 0; i--)
    {
        float ref_vel = path.poses[i].pose.position.z;
        float cur_vel = path.poses[i + 1].pose.position.z;
        float ds = GetDistanceXY(path.poses[i].pose.position, path.poses[i + 1].pose.position);
        float vk = (ref_vel - cur_vel) / ds;
        len += ds;

        if (fabs(vk) < 0.001)  break;

        if (vk > acc_per_len)  vk = acc_per_len;
        else if (vk < dec_per_len)  vk = dec_per_len;
        if (len <= safe_dis)
            path.poses[i].pose.position.z = min_vel;
        else
            path.poses[i].pose.position.z = cur_vel + vk * ds;
    }

    // for(auto p : path.poses)  printf("%.2f ", p.pose.position.z);
    // printf("\n\n");
}

//  根据task_path生成全局路径golbal_path 1 路径点插值 2 加减速规划
void TLocalPathPlan::GlobalPathPlan()
{
    Nearest_ID = 0;
    global_path.header.stamp = ros::Time::now();
    if (task_path.poses.size() < 2)
    {
        global_path.poses.clear();
        return;
    }

    global_path.poses.clear();
    nav_msgs::Path plan_path; //  速度规划的路径
    // ROS_ERROR("%d %s\n",task_path.poses.size(), cur_task.cmd.c_str());
    
    for (int i = 0; i < task_path.poses.size() - 1; i++)
    {
        geometry_msgs::Point p1, p2;
        p1.x = task_path.poses[i].pose.position.x;
        p1.y = task_path.poses[i].pose.position.y;
        p2.x = task_path.poses[i+1].pose.position.x;
        p2.y = task_path.poses[i+1].pose.position.y;
        p1.z = p2.z = task_path.poses[i].pose.position.z;

        bool stop_flag=CheckStopAtNextPoint(task_path, i, true);
        
        //  路径加密处理
        float ds = 0.1;
        nav_msgs::Path path;
        if(stop_flag)  path = LineInterpolation(p1, p2, ds, last_path_precision);
        else  path = LineInterpolation(p1, p2, ds);
        // if (i == task_path.poses.size() - 2)  ds = last_path_precision;
        
        for (auto &p : path.poses)
            p.pose.orientation = task_path.poses[i].pose.orientation;

        plan_path.header = path.header;
        for (auto p : path.poses)
            plan_path.poses.push_back(p);

        //  判断停止点
        if (stop_flag) // 有折点要停止,整体进行速度规划, 加入全局路径
        {
            float safe_dis = 0.3;
            SpeedPlan(plan_path, 0.1, safe_dis);
            for (auto p : plan_path.poses)  global_path.poses.push_back(p);
            plan_path.poses.clear();
        }
    }

    if (global_path.poses.size()>0)
        (global_path.poses.end()-1)->pose.position.z = 0;

    // printf("globalpath size=%d\n", global_path.poses.size());
}



void TLocalPathPlan::UpdateLaneType()
{
    // 寻找任务路径上最近点，如果加入引导点，则存在问题
    int id=0;
    FindNearestPointInPath(task_path, id, 2);
    // printf("id=%d\n",id);
    if(id>=0 && id<cur_task.path.size())
    {
        static int lane_type=-1;
        if(lane_type!=cur_task.path[id].lane_type)
        {
            lane_type=cur_task.path[id].lane_type;
            nh_local->setParam("lane_type", lane_type);
        }
    }
}

//  局部路径规划
void TLocalPathPlan::LocalPathPlan()
{
    local_path.header.stamp = ros::Time::now();
    local_path.poses.clear();
    

    UpdateLaneType();
    FindNearestPointInPath(global_path, Nearest_ID, 2);
    // printf("near=%d\n",Nearest_ID);

    // 获得已经走过的路程和剩余路程
    GetPathLength(global_path, Nearest_ID, path_passed, path_remain);

    std_msgs::Float64 data_msg;
    data_msg.data = path_remain;
    remainpath_pub.publish(data_msg);
    data_msg.data = path_passed;
    passedpath_pub.publish(data_msg);

    // printf("task=%s path_remain=%.2f paw_state=%s\n", cur_task.cmd.c_str(), path_remain, paw_state.c_str());
    float ds = 0;
    for (int i = Nearest_ID; i < global_path.poses.size(); i++)
    {
        local_path.poses.push_back(global_path.poses[i]);
        
        if (i > 0)
        {
            geometry_msgs::Point p1 = global_path.poses[i].pose.position;
            geometry_msgs::Point p2 = global_path.poses[i - 1].pose.position;
            ds += GetDistanceXY(p1, p2);
        }
    
        if (ds>10 || CheckStopAtNextPoint(global_path, i, false))
        {
            if (i < global_path.poses.size() - 1)
            
                local_path.poses.push_back(global_path.poses[i + 1]);
            break;
        }
    }
    // printf("local_path_size=%d\n",local_path.poses.size());
}


//  主运行函数
void TLocalPathPlan::Run()
{
    // nh_local->setParam("lane_type", 0);
    nh_local->getParam("/gps_base/utmx_zero", utm_x_zero);
    nh_local->getParam("/gps_base/utmy_zero", utm_y_zero);
    static TTimer tmr;
    if (tmr.GetValue() > 1)
    {
        tmr.Clear();
        taskpath_pub.publish(task_path);
        globalpath_pub.publish(global_path);
    }
    
    LocalPathPlan();     //局部路径规划
    localpath_pub.publish(local_path);
    PubPathMarkers();

    // printf("%d\n", local_path.poses.size());
    nodecheck->Find("node_rate")->Beat();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local path plan");
    TLocalPathPlan localpathplan;
    ROS_INFO("0");
    ros::Rate rate(20);
    while (ros::ok())
    {
        localpathplan.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
