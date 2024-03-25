// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

// C++ includes
#include <vector>
#include <string>

using namespace std;

bool transformPoint(string target_frame, geometry_msgs::PointStamped &src_p, geometry_msgs::PointStamped &dst_p, string tag="")
{
    // printf("%s %s\n",target_frame.c_str(), src_p.header.frame_id.c_str());
    
    if(target_frame==src_p.header.frame_id) 
    {
        dst_p=src_p;
        return true;
    }

    static tf::TransformListener listener;
    bool res;
    try
    {
        src_p.header.stamp = ros::Time(0);
        listener.waitForTransform(target_frame, src_p.header.frame_id, src_p.header.stamp, ros::Duration(1));
        listener.transformPoint(target_frame, src_p, dst_p);
        res = true;
        // printf("AAA=%.2f %.2f\n", dst_p.point.x, dst_p.point.y);
    }
    catch (exception e)
    {
        res = false;
        if(tag!="")  ROS_INFO("tag=%s  %s", tag.c_str(), e.what());
    }

    return res;
}

bool transformPose(string target_frame, geometry_msgs::PoseStamped src_p, geometry_msgs::PoseStamped &dst_p, string tag="")
{
    // printf("%s %s\n",target_frame.c_str(), src_p.header.frame_id.c_str());
    
    if(target_frame==src_p.header.frame_id) 
    {
        dst_p=src_p;
        return true;
    }

    static tf::TransformListener listener;
    bool res;
    string *str = new string;
    try
    {
        src_p.header.stamp = ros::Time(0);
        listener.waitForTransform(target_frame, src_p.header.frame_id, src_p.header.stamp, ros::Duration(2), ros::Duration(0.01), str);
        listener.transformPose(target_frame, src_p, dst_p);
        // dst_p.header.stamp=src_p.header.stamp;
        // dst_p.header.frame_id=target_frame;
        res = true;
        // printf("AAA=%.2f %.2f\n", dst_p.point.x, dst_p.point.y);
    }
    catch (exception e)
    {
        res = false;
        if(tag!="")  ROS_INFO("tag=%s  %s %s", tag.c_str(), e.what(), str->c_str());
    }

    return res;
}

float GetYawFromPose(geometry_msgs::PoseStamped pose)
{
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::PoseStamped GetExtendPoseByPose(geometry_msgs::PoseStamped targetpose, float l)
{
    float yaw = GetYawFromPose(targetpose);
    geometry_msgs::PoseStamped pose = targetpose;
    pose.pose.position.x += l * cos(yaw);
    pose.pose.position.y += l * sin(yaw);
    return pose;
}

float GetAngleByPoses(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
    float angle1=GetYawFromPose(p1);
    float angle2=GetYawFromPose(p2);
    float angle = angle2-angle1;
    return angle;
}

float GetAngleByPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float angle = atan2(y2 - y1, x2 - x1);
    return angle;
}

geometry_msgs::Quaternion GetQuaternionMsgByPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    float angle = GetAngleByPoints(p1, p2);
    return tf::createQuaternionMsgFromYaw(angle);
}

float GetDistanceXY(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

float GetDistance(geometry_msgs::PoseStamped p)
{
    geometry_msgs::PointStamped p1, p2;
    p1.header = p.header;
    p1.point = p.pose.position;
    transformPoint("base_link", p1, p2, "XXX");

    float ds = sqrt(pow(p2.point.x, 2) + pow(p2.point.y, 2));
    return ds;
}

float GetPathLength(nav_msgs::Path path, int start_id = 0, int stop_id = -1)
{
    if (stop_id < 0 || stop_id >= path.poses.size()-1)  stop_id = path.poses.size() - 1;
    if(path.poses.size()==0)  return 0;

    float res = 0;
    for (int i = start_id; i <= stop_id - 1; i++)
    {
        geometry_msgs::Point p1 = path.poses[i].pose.position;
        geometry_msgs::Point p2 = path.poses[i + 1].pose.position;
        res += GetDistanceXY(p1, p2);
    }
    return res;
}

void GetPathLength(nav_msgs::Path path, int start_id, float &passed_path, float &remain_path)
{
    passed_path = remain_path = 0;
    if (path.poses.size()==0)  return;

    for (int i = 0; i<start_id; i++)
    {
        geometry_msgs::Point p1 = path.poses[i].pose.position;
        geometry_msgs::Point p2 = path.poses[i + 1].pose.position;
        passed_path += GetDistanceXY(p1, p2);
    }

    for (int i = start_id; i<path.poses.size()-1; i++)
    {
        geometry_msgs::Point p1 = path.poses[i].pose.position;
        geometry_msgs::Point p2 = path.poses[i + 1].pose.position;
        remain_path += GetDistanceXY(p1, p2);
    }
}

//  获得路径上最近点ID
void FindNearestPointInPath(nav_msgs::Path path, int &start_id, float max_len)
{
    if (path.poses.size() == 0)  return;

    if (start_id<=0)  max_len = 100000;

    float dmin = 9999999;
    float ds = 0;
    for (int i = start_id; i < path.poses.size(); i++)
    {
        float dd = GetDistance(path.poses[i]);
        if (i > 0)
            ds += GetDistanceXY(path.poses[i - 1].pose.position, path.poses[i].pose.position);
        if (ds > max_len)  break; //  按照最高速度在50ms内运动不会超过0.6m
        if (dd <= dmin)   start_id = i, dmin = dd;
    }
}

//  获得路径上最近点ID
void FindNearestPointInPath(nav_msgs::Path path, geometry_msgs::PointStamped point, int &id)
{
    // printf("size=%d\n",path.poses.size());
    if (path.poses.size() == 0)  return;

    geometry_msgs::PointStamped point_path;
    transformPoint(path.header.frame_id, point, point_path);

    float dmin = 9999999;
    for (int i = id; i < path.poses.size(); i++)
    {
        float dd = GetDistanceXY(path.poses[i].pose.position, point_path.point);
        if (dd <= dmin)  id=i, dmin=dd;
    }
}

string CheckMoveDir(nav_msgs::Path path, int id)
{
    string move_dir="";
    if (path.poses.size()<2 || id+1>=path.poses.size())  return move_dir;

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


visualization_msgs::Marker displayCarPosition(geometry_msgs::PoseStamped pose_stamped, int Quality)
{

  //marker.type = visualization_msgs::Marker::SPHERE;
  //marker.action = visualization_msgs::Marker::ADD;
    visualization_msgs::Marker marker;
    marker.header=pose_stamped.header;
    //marker.header.stamp = ros::Time();   //用哪个？
    marker.ns = "car_position_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.9;//+pose_stamped.pose.position.z;
    marker.scale.y = 0.4;
    marker.scale.z = 0.3;
    marker.color.a = 1.0f;

    marker.frame_locked = true;
    marker.lifetime = ros::Duration();
    // marker.pose.position = pose_stamped.pose.position;
    marker.pose.position.x = pose_stamped.pose.position.x;
    marker.pose.position.y = pose_stamped.pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation= pose_stamped.pose.orientation;

    if(Quality == 4)
    {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }
    else if(Quality == -1)
    {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    }
    else
    {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    }
    return marker;
}


visualization_msgs::Marker displayPoint(geometry_msgs::PointStamped p, std_msgs::ColorRGBA c, geometry_msgs::Vector3 scale)
{
    visualization_msgs::Marker marker;
    marker.header = p.header;
    marker.ns = "point_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale=scale;
    marker.lifetime = ros::Duration(0.1);
    marker.color=c;
    marker.pose.position = p.point; //斟酌一下
    return marker;
}

visualization_msgs::Marker GetVisualMarker(geometry_msgs::Point p, std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.lifetime = ros::Duration();
    marker.pose.position = p;
    marker.color=color;
    marker.color.a = 1.0f;
    // marker.color.r = 0.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 0.0f;
    return marker;
}

nav_msgs::Path GetVisualPath(vector<geometry_msgs::Point> path_buf)
{
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";
    for(int i=0;i<path_buf.size();i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position=path_buf[i];
        pose.header.stamp=ros::Time::now();
        pose.header.frame_id="map";
        path.poses.push_back(pose);
    }
    return path;
}

std_msgs::ColorRGBA GetColor(float r,float g,float b)
{
    std_msgs::ColorRGBA color;
    color.r=r, color.g=g, color.b=b;
    return color;
}




