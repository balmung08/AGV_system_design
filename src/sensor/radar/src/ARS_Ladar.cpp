#include "radar/ARS_Ladar.h"
#include <common/can.h>

ARS_Ladar::ARS_Ladar()
{
    pn=new ros::NodeHandle("~");

    radarstate = false;
    // frame_id=0x600;

    int id=0;
    pn->getParam("can_ch", id);
    pn->getParam("ROI_longth", ROI_longth);
    pn->getParam("ROI_width", ROI_width);
    //pn.getParam("buffer_time", buffer_time);

    can = InitCan(id);

    radar_pub = pn->advertise<std_msgs::Int32>("Radar", 1);
    radarstate_pub = pn->advertise<std_msgs::Int32MultiArray>("radarstate", 3);
    string pub_topic="Radar_points";
    pn->getParam("pub_topic", pub_topic);
    Radarpoints_pub = pn->advertise<visualization_msgs::MarkerArray>(pub_topic, 1);
    pn->getParam("frame_id", frame_id);

    for (int i = 0; i < MAX_SIZE; ++i)
    {
        objects[i].isExit = false;
        objects[i].old_pre_Distx = 0;
        objects[i].old_pre_Disty = 0;
        objects[i].pre_Distx = 0;
        objects[i].pre_Disty = 0;
        objects[i].m_Distx = 0;
        objects[i].m_Disty = 0;
    }

    gettimeofday(&now_time, NULL);
    gettimeofday(&obstacle_time, NULL);
    isRadarObsExit = false;
    speed = 1;

    bool init_flag = false;
    pn->getParam("init_flag", init_flag);
    if(init_flag)  Init();

    nodecheck=new TNodeCheck(pn,"node_rate");
    nodecheck->Find("node_rate")->SetLimit(20);

    start();    
}

inline bool ARS_Ladar::Check(TObsInfo p)
{
    float sx = 1, sy = 50, angle = 80;

    bool r = false;
    if (p.exitpro <= 3 || fabs(sqrt(p.m_Distx * p.m_Distx + p.m_Disty * p.m_Disty) - sqrt(p.pre_Distx * p.pre_Distx + p.pre_Disty * p.pre_Disty)) > 2)
    {
        return false;
    }

    return true;
}

void ARS_Ladar::ProcessRadarData(const can_frame can_data) //解析CAN帧
{
    TObsInfo p;
    p.isExit = false;

    uint8_t *radardata = new uint8_t[can_data.can_dlc];
    memset(radardata, 0, sizeof(radardata));
    for (int i = 0; i < can_data.can_dlc; ++i)
    {
        *(radardata + i) = can_data.data[i];
    }

    if (can_data.can_id == 0x060a)
    {

        NUMBER = objectListStatus60A.num_of_objects(radardata, 0);
        radarstate = true;

        for (int i = 0; i < MAX_SIZE; ++i)
        {
            objects[i].isExit = false;
        }
    }
    else if (can_data.can_id == 0x060b) //判断报文头范围，总共64个ID范围
    {

        int index = objectGeneralInfo60B.object_id(radardata, 0);

        objects[index].old_pre_Distx = objects[index].pre_Distx;
        objects[index].old_pre_Disty = objects[index].pre_Disty;
        objects[index].pre_Distx = objects[index].m_Distx;
        objects[index].pre_Disty = objects[index].m_Disty;

        objects[index].m_Disty = objectGeneralInfo60B.longitude_dist(radardata, 0);
        objects[index].m_Distx = objectGeneralInfo60B.lateral_dist(radardata, 0);
        objects[index].m_Vely = objectGeneralInfo60B.longitude_vel(radardata, 0);
        objects[index].m_Velx = objectGeneralInfo60B.lateral_vel(radardata, 0);
        objects[index].m_DynProp = objectGeneralInfo60B.dynprop(radardata, 0);
        objects[index].RCS = objectGeneralInfo60B.rcs(radardata, 0);

        if (Check(objects[index]))
        {
            objects[index].isExit = true;
        }
    }
    else if (can_data.can_id == 0x060c)
    {
        int index = objectQualityInfo60C.object_id(radardata, 0);
        objects[index].obj_DistLong_rms = objectQualityInfo60C.longitude_dist_rms(radardata, 0);
        objects[index].obj_DistLat_rms = objectQualityInfo60C.lateral_dist_rms(radardata, 0);
        objects[index].exitpro = objectQualityInfo60C.probexist(radardata, 0);
        // printf("%d  %d   %d\n",objects[index].exitpro,objects[index].obj_DistLong_rms,objects[index].obj_DistLat_rms);
    }
    else if (can_data.can_id == 0x060d) //这一帧是输出目标的属性，如是行人还是车辆等，还会输出物体的高度与长度
    {
        int index = objectExtendedInfo60D.object_id(radardata, 0);
        objects[index].length = objectExtendedInfo60D.object_length(radardata, 0);
        objects[index].width = objectExtendedInfo60D.object_width(radardata, 0);
        objects[index].object_Class = objectExtendedInfo60D.obstacle_class(radardata, 0);
        // printf("%d\n",objects[index].object_Class);
    }
    else if (can_data.can_id == 0x0003)
    {
        distance = can->rec_frame.data[0] + can->rec_frame.data[1] * 256;
        distance = distance * 0.01;
    }
    else
    {
        int A = 0;
    }

    delete[] radardata;
    radardata = nullptr;
}

void ARS_Ladar::PubRadarObject()
{
    // printf("%.1f\n",myhb->value);

    // printf("hello\n");
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;
    markerarray.markers.clear();
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.lifetime = ros::Duration(0.2);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    int marker_id = 0;

    for (int i = 0; i < MAX_SIZE; ++i)
    {
        marker_id = i;
        if (objects[i].isExit && objects[i].exitpro > 2 && fabs(objects[i].m_Disty) < ROI_longth && fabs(objects[i].m_Distx) < ROI_width)
        {
            gettimeofday(&obstacle_time, NULL);
            // printf("hello\n");
        }
        else continue;

        if (objects[i].isExit && objects[i].exitpro > 2)
        {
            marker.color.a = 1.0f;
        }
        else
        {
            marker.color.a = 0.0f;
        }
        if ((fabs(objects[i].m_Velx) > 0.1 || fabs(objects[i].m_Vely) > 0.1) && objects[i].exitpro >= 4)
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        }
        else
        {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        }
        marker.id = i;
        marker.pose.position.y = objects[i].m_Distx;
        marker.pose.position.x = objects[i].m_Disty;
        marker.pose.position.z = 0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        markerarray.markers.push_back(marker);
    }

    Radarpoints_pub.publish(markerarray);
}

void ARS_Ladar::run()
{
    int COUNT=0;
    while(1)
    {
        usleep(1000);
        COUNT++;
        gettimeofday(&now_time, NULL);
        NoObsTime = fabs((1000000 * (now_time.tv_sec - obstacle_time.tv_sec) + (now_time.tv_usec - obstacle_time.tv_usec)) / 1000000);

        if (radartimer1.GetValue() > 0.5)
        {
            radartimer1.Clear();
            radarstate = false;
        }

        if (COUNT % 20 == 0)
        {
            COUNT = 0;
            if (NoObsTime > buffer_time)
            {
                isRadarObsExit = false;
                std_msgs::Int32 msg;
                msg.data = 0;
                radar_pub.publish(msg);
            }
            else
            {
                isRadarObsExit = true;
                std_msgs::Int32 msg;
                msg.data = 1;
                radar_pub.publish(msg);
            }

            //  ROS_INFO("AAA\n");
            PubRadarObject();
            nodecheck->Find("node_rate")->Beat();
        }

        for (int i = 0; i < can->rec_frames.size(); i++)
        {
            can_frame *frame = &can->rec_frames[i];
            
            int flag=frame->can_id & 0xFF0;
            if (flag!=0x600) continue;

            ProcessRadarData(can->rec_frame);
            // printf("frame_id=%04x\n", frame->can_id);
            frame->can_id = 0;
        }
    }
}

void ARS_Ladar::Init()
{
    RadarConfig200 radarConfig200;
    uint8_t *radar200 = new uint8_t[8];
    memset(radar200, 0, sizeof(radar200));
    radarConfig200.UpdateData(radar200);
    can->send_frame.can_id = 0x200;
    can->send_frame.can_dlc = 0x08;
    for (int i = 0; i < can->send_frame.can_dlc; ++i)
    {
        can->send_frame.data[i] = *(radar200 + i);
    }
    delete[] radar200;
    radar200 = nullptr;
    can->Send();

    usleep(1000);
    RadarConfig202 radarConfig202;
    uint8_t *radar202 = new uint8_t[8];
    memset(radar202, 0, sizeof(radar202));
    radarConfig202.UpdateData(radar202);
    can->send_frame.can_id = 0x202;
    can->send_frame.can_dlc = 0x08;
    for (int i = 0; i < can->send_frame.can_dlc; ++i)
    {
        can->send_frame.data[i] = *(radar202 + i);
    }
    delete[] radar202;
    radar202 = nullptr;
    can->Send();

    usleep(1000);
    can->send_frame.can_id = 0x605;
    can->send_frame.can_dlc = 0x08;
    can->send_frame.data[0] = 0x40;
    can->send_frame.data[1] = 0x10;
    can->send_frame.data[2] = 0x10;
    can->send_frame.data[3] = 0x00;
    can->send_frame.data[4] = 0x01;
    can->send_frame.data[5] = 0x00;
    can->send_frame.data[6] = 0x00;
    can->send_frame.data[7] = 0x00;

    can->Send();
}

void ARS_Ladar::speedsend()
{
    motioninputspeed300.SetSpeed(speed);

    uint8_t *speed300 = new uint8_t[8];
    memset(speed300, 0, sizeof(speed300));
    motioninputspeed300.UpdateData(speed300);
    can->send_frame.can_id = 0x300;
    can->send_frame.can_dlc = 0x08;
    for (int i = 0; i < can->send_frame.can_dlc; ++i)
    {
        can->send_frame.data[i] = *(speed300 + i);
    }
    delete[] speed300;
    speed300 = nullptr;
    can->Send();

    can->send_frame.can_id = 0x605;
    can->send_frame.can_dlc = 0x08;
    can->send_frame.data[0] = 0x40;
    can->send_frame.data[1] = 0x10;
    can->send_frame.data[2] = 0x10;
    can->send_frame.data[3] = 0x00;
    can->send_frame.data[4] = 0x01;
    can->send_frame.data[5] = 0x00;
    can->send_frame.data[6] = 0x00;
    can->send_frame.data[7] = 0x00;

    can->Send();
}

int main(int argc, char *argv[])
{
   int falsecount = 0;
   int NUM = 0;

    ros::init(argc, argv, "ARS_Ladar");
    ARS_Ladar radar;
    
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        if (radar.radarstate == false)
        {
            falsecount++;
            // printf("%d\n", falsecount);
        }
        
        if (radar.radartimer.GetValue() > 1)
        {
            radar.radartimer.Clear();
            NUM++;
            std_msgs::Int32MultiArray msg;
            if (radar.radarstate)
            {
                msg.data.push_back(1);
            }
            else if (falsecount > 15)
            {
                msg.data.push_back(0);
            }

            radar.radarstate_pub.publish(msg);

            // if (falsecount > 15 && NUM > 5)
            // {
            //     //NUM = 0;
            //     //ros::shutdown();
            // }
            falsecount = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
