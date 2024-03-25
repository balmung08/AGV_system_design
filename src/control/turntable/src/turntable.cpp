#include "ros/ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <common/public.h>
#include <common/myudp.h>
#include "motordrv_sdo.h"


TMotorDrv *motor_pitch, *motor_azimuth;

float azimuth_check_pos1, azimuth_check_pos2, azimuth_check_vel;
float pitch_check_pos1, pitch_check_pos2, pitch_check_vel;
int azimuth_check_ctr=0, pitch_check_ctr=0;

int azimuth_zero_cnt=0, pitch_zero_cnt=0;
float base_pitch=0, base_roll=0;
bool enable_flag=false;

TUDP *udp=NULL;
TTimer udp_tmr;

void SingleCtr(vector<string> strs)
{
    if(strs.size()<2) return;

	TMotorDrv *drv=NULL;
    if(strs[0]=="AZIMUTH") {drv=motor_azimuth;   enable_flag=(motor_azimuth->servo_on);}
	else if(strs[0]=="PITCH") {drv=motor_pitch;    enable_flag=(motor_pitch->servo_on);}

    if(drv!=NULL)
	{
		// PITCH/AZIMUTH MOV pos vel 
		// ROS_INFO("%s",strs);
		if(strs[1]=="MOV" && strs.size()==4)  
		{
			drv->Stop(0);
	        float pos=atof(strs[2].c_str());
			float vel=atof(strs[3].c_str());
			if(drv==motor_azimuth) 
			{
				azimuth_check_ctr=0;
				motor_azimuth->SetSpd_Deg(vel);
				motor_azimuth->SetTargetPosDeg(pos);
			}
			else 
			{
				pitch_check_ctr=0;
				motor_pitch->SetSpd_Deg(vel);
				motor_pitch->SetTargetPosDeg(pos);
			}	
    	}
		else if(strs[1]=="ENABLE")
		{
			int v=atoi(strs[2].c_str());
			
			if(enable_flag != bool(v)) drv->Enable(v);
			
		}
		else if(strs[1]=="STOP")
		{
			drv->Stop(1);
			// ROS_INFO("%s",strs[1].c_str());
		}
		else if(strs[1]=="CHECK" && strs.size()==5)  
		{
			drv->Stop(0);
			if(drv==motor_azimuth) 
			{
                azimuth_check_pos1=atof(strs[2].c_str());
                azimuth_check_pos2=atof(strs[3].c_str());
				azimuth_check_vel=atof(strs[4].c_str());
				azimuth_check_ctr=1;
				motor_azimuth->SetSpd_Deg(azimuth_check_vel);
   	            motor_azimuth->SetTargetPosDeg(azimuth_check_pos1);
			}
			else if(drv==motor_pitch) 
			{
                pitch_check_pos1=atof(strs[2].c_str());
                pitch_check_pos2=atof(strs[3].c_str());
				pitch_check_vel=atof(strs[4].c_str());
				pitch_check_ctr=1;
				motor_pitch->SetSpd_Deg(pitch_check_vel);
   	            motor_pitch->SetTargetPosDeg(pitch_check_pos1);
			}
		}
	}
}


void BroadTf()
{
	static tf::TransformBroadcaster br;
    tf::Transform transform;
	tf::Quaternion quaternion;
    transform.setOrigin(tf::Vector3(-0.01, 0, 0.122)); //base_link在map中的位置
	quaternion.setRPY(0,0,0);
    transform.setRotation(quaternion);  //base_link在map中的旋转四元数
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/turntable_pitch", "/rslidar"));
        
	transform.setOrigin(tf::Vector3(0, 0, 0)); //base_link在map中的位置
	quaternion.setRPY(0,-motor_pitch->pos_deg/180.0*M_PI,motor_azimuth->pos_deg/180*M_PI);
    transform.setRotation(quaternion);  //base_link在map中的旋转四元数
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/turntable_azimuth", "/turntable_pitch"));

	transform.setOrigin(tf::Vector3(0, 0, 0.169));   
	quaternion.setRPY(0,0,0);  
    transform.setRotation(quaternion);  //base_link在map中的旋转四元数
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/turntable_base", "/turntable_azimuth"));

    // transform.setOrigin(tf::Vector3(0, 0, 0)); //base_link在map中的位置
	// quaternion.setRPY(base_roll,base_pitch,0);  
    // transform.setRotation(quaternion);  //base_link在map中的旋转四元数
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/turntable_base"));
}

void TurntableCtrCallback(const std_msgs::String::ConstPtr &msg)
{
	vector<string> strs=split(msg->data,";");
	// ROS_INFO("%s",msg->data.c_str());
	if(strs.size()<1)  return;

    for(int i=0;i<strs.size();i++)
	{
		vector<string> substrs=split(strs[i]," ");
        SingleCtr(substrs);
	}
}

// void AngleCanCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
// {
//     base_pitch=msg->point.x*3.1416/180;   
// 	base_roll=msg->point.y*3.1416/180;
// }

void udp_Run()
{
    if(udp!=NULL && udp->rec_flag)
	{
		udp->rec_flag=0;
        string ss=udp->rec_buf;
        vector<string> strs=split(ss,";");
		for(int i=0;i<strs.size();i++)
	    {
		    vector<string> substrs=split(strs[i]," ");
            SingleCtr(substrs);
	    }
		// printf("%s\n",udp->rec_buf);
	}
}

void Motor_CheckRun()
{
    if(azimuth_check_ctr==1 && fabs(motor_azimuth->pos_deg-azimuth_check_pos1)<0.2)
	{		    
        motor_azimuth->SetSpd_Deg(azimuth_check_vel);
   	    motor_azimuth->SetTargetPosDeg(azimuth_check_pos2);
		azimuth_check_ctr++;
	}
	else if(azimuth_check_ctr==2 && fabs(motor_azimuth->pos_deg-azimuth_check_pos2)<0.2)
	{		    
        motor_azimuth->SetSpd_Deg(azimuth_check_vel);
	    motor_azimuth->SetTargetPosDeg(azimuth_check_pos1);
		azimuth_check_ctr=1;
	}

    if(pitch_check_ctr==1 && fabs(motor_pitch->pos_deg-pitch_check_pos1)<0.2)
	{		    
        motor_pitch->SetSpd_Deg(pitch_check_vel);
   	    motor_pitch->SetTargetPosDeg(pitch_check_pos2);
		pitch_check_ctr++;
	}
    else if(pitch_check_ctr==2 && fabs(motor_pitch->pos_deg-pitch_check_pos2)<0.2)
	{		    
        motor_pitch->SetSpd_Deg(pitch_check_vel);
   	    motor_pitch->SetTargetPosDeg(pitch_check_pos1);
		pitch_check_ctr=1;
	}
}

TNodeCheck *nodecheck;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "turntable");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	nodecheck=new TNodeCheck(&nh, "node_rate");
	nodecheck->Find("node_rate")->SetLimit(90);

	// int local_port=1234;
    // nh.getParam("udp_local_port",local_port);
    // int remote_port=1234;
    // nh.getParam("udp_remote_port",remote_port);
    // string remote_ip="192.168.1.102";
    // nh.getParam("udp_remote_ip",remote_ip);
    // if(local_port>0)
	// {
	//    udp=new TUDP(local_port);
	//    udp->AddRemoteIP(remote_ip,remote_port);
	//    // printf("%d %d %s\n",local_port,remote_port,remote_ip.c_str());    
	// }  

    int can_ch=0, azimuth_canid=5, pitch_canid=6;
	nh.getParam("can_ch",can_ch);
	nh.getParam("azimuth_canid",azimuth_canid);
	nh.getParam("pitch_canid",pitch_canid);
    nh.getParam("pitch_zero",pitch_zero_cnt);
	nh.getParam("azimuth_zero",azimuth_zero_cnt);

    bool pitch_enable, azimuth_enable, show_cnt;
    nh.getParam("pitch_enable", pitch_enable);
	nh.getParam("azimuth_enable", azimuth_enable);
	nh.getParam("show_cnt", show_cnt);

	motor_azimuth=new TMotorDrv(can_ch, azimuth_canid, azimuth_zero_cnt);
	motor_azimuth->Enable(azimuth_enable);
	motor_pitch=new TMotorDrv(can_ch, pitch_canid, pitch_zero_cnt);
	motor_pitch->Enable(pitch_enable);

    motor_azimuth->SetSpd_Deg(40);
	motor_pitch->SetSpd_Deg(40);
	
    string sub_topic="/turntable_ctr", pub_topic="table_state";
	nh.getParam("sub_topic",sub_topic);
	ros::Subscriber ctr_sub = n.subscribe<std_msgs::String>(sub_topic, 10, TurntableCtrCallback);
	// ros::Subscriber baseangle_sub = n.subscribe<geometry_msgs::PointStamped>("/msensor_angle/angle", 10, AngleCanCallback);
	
	nh.getParam("pub_topic",pub_topic);
	ros::Publisher table_pub = nh.advertise<std_msgs::String>(pub_topic, 10);
	ros::Rate loop_rate(100);  

    bool first_check=true;
	while (ros::ok())
	{
		Motor_CheckRun();
		motor_azimuth->CheckStatus();
		motor_pitch->CheckStatus();

		if(show_cnt) ROS_INFO("azimuth_cnt=%d pitch_cnt=%d", motor_azimuth->pos_cnt, motor_pitch->pos_cnt);

		if(first_check)
		{
			first_check=false;
			motor_pitch->SetTargetPosDeg(0);
	        motor_azimuth->SetTargetPosDeg(0);
		}

		if(motor_pitch->heartbeat.value>30 && motor_azimuth->heartbeat.value>30)
		{
			nodecheck->Find("node_rate")->Beat();
		}
		else 
		{
		    if(motor_pitch->heartbeat.value<10)  motor_pitch->pos_deg=0;
			if(motor_azimuth->heartbeat.value<10)  motor_azimuth->pos_deg=0;
		}
		
		// motor_pitch->pos_deg=motor_azimuth->pos_deg=0;  // 用于零时调试
		BroadTf();

        char buf[500];
		sprintf(buf,"azimuth %d %.2f %d;pitch %d %.2f %d", motor_azimuth->servo_on, motor_azimuth->pos_deg, motor_azimuth->reach_flag, 
		                                                   motor_pitch->servo_on, motor_pitch->pos_deg, motor_pitch->reach_flag);
	    // if(udp!=NULL && udp_tmr.GetValue()>0.02) 
		// {
		//    udp_tmr.Clear();
		//    udp->Send(buf);
		// }	
		// udp_Run();

		// printf("%s\n",buf);
		std_msgs::String msg;
		msg.data=buf;
		table_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
