//添加twice轨迹跟踪
/*
		SetMode(car_state2);
		SetRunPara(car_orrentation_state2,car_speed2,car_angle2);
		SetPawMode(paw_distance_control2, paw_lift_control2, vehicle_put2, paw_distance2);
输入：
pawctr:

int32 paw_distance_control: pawdistance_mode=0x00 无动作 pawdistance_mode=0x01 轴距增大 pawdistance_mode=0x02 轴距减小
float32 paw_distance:[0,1000]
int32 paw_lift_control: pawlifting_mode=0x00 无动作  pawlifting_mode=0x01 夹爪上升  pawlifting_mode=0x02 夹爪降低
int32 vehicle_put: Vehicleput_mode=0x00 无动作 Vehicleput_mode=0xAA 自动取车 Vehicleput_mode=0xBB 自动放车

zcarctr:

int32 car_state: cmd=0x0; 初始化状态 cmd=0x1; 急停 cmd=0x2; 待机 cmd=0x3; 工作 cmd=0x4; 充电 cmd=0x5; 自检
int32 car_orrentation_state mode=0x0; 阿克曼前后轮模式 mode=0x1; 差速模式 mode=0x2; 自转模式 mode=0x3; 横移模式
float32 car_speed
float32 car_angle
// 1）（阿克曼模式）单位0.01m/s，范围[-8.5,8.5]，指令 -850~850
// 2）（横移模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// 3）（自传模式）角速度值单位rad/s
// 4）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// p2
// 1）（阿克曼模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 2）（横移模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 3）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500
*/
#include "ros/ros.h"
#include <sstream>
#include <common/can.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <common/public.h>
#include <data_comm/paw_state.h>
#include <data_comm/paw_ctr.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <data_comm/battery_info.h>
#include <data_comm/car_angle.h>

TCan *can;
TNodeCheck *nodecheck;

data_comm::car_state car_state;
data_comm::car_angle car_angle;
data_comm::car_ctr car_cmd;
data_comm::paw_state paw_state;
data_comm::paw_ctr paw_ctr;

ros::NodeHandle *nh;
ros::Publisher carstate_pub, pawstate_pub, battery_pub,carangle_pub;

//爪子的回调函数
void PawCtrCallback(const data_comm::paw_ctr::ConstPtr &msg)
{
	paw_ctr = *msg;
}

//转运车控制的回调函数
void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)
{
	bool moving = false;
	for (int i = 0; i < 4; i++)
		if (fabs(car_state.speed[i]) > 0.01)
			moving = true;

	if (car_cmd.turnmode != msg->turnmode && moving)  return;

	car_cmd = *msg;
	// printf("%d\n", car_cmd.workmode);
}

// 0-初始化状态  1-急停  2-待机  3-工作  4-充电  5-自检  cmd_car
// 0-爪子关闭   1-开启     cmd_paw
void SetWorkMode(data_comm::car_ctr cmd) // int cmd_car, int cmd_paw1)
{
	if(car_state.workmode==1)  cmd.workmode=2;
	
	can->send_frame.can_id = 0x8C110A1C; // | (1<<32);

	can->send_frame.can_dlc = 0x08;
	can->send_frame.data[0] = cmd.pawmode * pow(2, 3) + cmd.workmode;

	for (int i = 1; i < 8; i++)
		can->send_frame.data[i] = 0x0;
	can->Send();
	// ROS_ERROR("%d\n", cmd.workmode);
}

// 0-阿克曼前后轮模式  1-差速模式  2-自转模式  3-横移模式
// 1）（阿克曼模式）单位0.01m/s，范围[-8.5,8.5]，指令 -850~850
// 2）（横移模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// 3）（自传模式）角速度值单位rad/s
// 4）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// p2
// 1）（阿克曼模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 2）（横移模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 3）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500

void SetRunPara(data_comm::car_ctr cmd)
{
	can->send_frame.can_id = 0x8C120A1C; // | (1<<32);

	can->send_frame.can_dlc = 0x08;

	for (int i = 0; i < 8; i++)
		can->send_frame.data[i] = 0x0;

	can->send_frame.data[0] = cmd.turnmode;
	// printf("%d\n",turnmode);
	int x = cmd.speed * 100;

	if (x < 0)
		x = x + 0x10000;
	can->send_frame.data[1] = x % 256;
	can->send_frame.data[2] = x / 256;

	x = cmd.angle * 100;
	if (x < 0)  x = x + 0x10000;
	can->send_frame.data[3] = x % 256;
	can->send_frame.data[4] = x / 256;

	if(cmd.workmode==4) can->send_frame.data[5]=1;   //充电口打开
	else can->send_frame.data[5]=0;     //充电口关闭

	static int count = 0;
	can->send_frame.data[7] = count++;

	can->Send();
}

// pawdistance_mode
// pawdistance_mode=0x00 无动作
// pawdistance_mode=0x01 轴距增大
// pawdistance_mode=0x02 轴距减小
// pawlifting_mode
// pawlifting_mode=0x00 无动作
// pawlifting_mode=0x01 夹爪上升
// pawlifting_mode=0x02 夹爪降低
// Vehicleput_mode
// Vehicleput_mode=0x00 无动作
// Vehicleput_mode=0xAA 自动取车
// Vehicleput_mode=0xBB 自动放车
// p1夹抓轴距调整值 【0，1000】
void SetPawMode(data_comm::paw_ctr cmd)
{
	can->send_frame.can_id = 0x8C130A1C;
	can->send_frame.can_dlc = 0x08;

	for (int i = 0; i < 8; i++)
		can->send_frame.data[i] = 0x0;

	int left_pawdistance_mode_big, left_pawdistance_mode_little, right_pawdistance_mode_big, right_pawdistance_mode_little;
	left_pawdistance_mode_big = left_pawdistance_mode_little = right_pawdistance_mode_big = right_pawdistance_mode_little = 0;

	if (cmd.left_paw_distance_control == 1)
	{
		left_pawdistance_mode_big = 1;
		left_pawdistance_mode_little = 0;
	}
	else if (cmd.left_paw_distance_control == 2)
	{
		left_pawdistance_mode_big = 0;
		left_pawdistance_mode_little = 1;
	}

	if (cmd.right_paw_distance_control == 1)
	{
		right_pawdistance_mode_big = 1;
		right_pawdistance_mode_little = 0;
	}
	else if (cmd.right_paw_distance_control == 2)
	{
		right_pawdistance_mode_big = 0;
		right_pawdistance_mode_little = 1;
	}
	else
	{
		right_pawdistance_mode_big = 0;
		right_pawdistance_mode_little = 0;
	}

	can->send_frame.data[0] = right_pawdistance_mode_little * pow(2, 3) + right_pawdistance_mode_big * pow(2, 2) + left_pawdistance_mode_little * pow(2, 1) + left_pawdistance_mode_big;

	int x = cmd.left_paw_speed; //  left_wheel_paw * 1;
	if (x < 0)
		x = x + 0x10000;
	can->send_frame.data[1] = x % 256;
	can->send_frame.data[2] = x / 256;

	x = cmd.right_paw_speed;
	if (x < 0)
		x = x + 0x10000;
	can->send_frame.data[5] = x % 256;
	can->send_frame.data[6] = x / 256;

	static int paw_cmd=0;
	can->send_frame.data[3] = cmd.paw_lift_control;
	can->send_frame.data[4] = cmd.vehicle_put_control;
	// if((paw_cmd==0xAA && cmd.vehicle_put_control==0xBB) || (paw_cmd==0xBB && cmd.vehicle_put_control==0xAA))
	// {
	// 	ROS_INFO("PAW_ctr err!! \n");
	// }cur_carstate.turnmode
	paw_cmd=cmd.vehicle_put_control;

	static int count = 0;
	can->send_frame.data[7] = count++;

	can->Send();
}

void ProcCanMsg()
{
	for (int i = 0; i < can->rec_frames.size(); i++)
	{
		can_frame *frame = &can->rec_frames[i];
		// printf("frame=%x\n",frame->can_id);
		if (frame->can_id == 0x88411B0A) // AGV状态信息
		{
			nodecheck->Find("can_rate")->Beat();

			car_state.workmode = (frame->data[0] & 0x7);
			car_state.turnmode = (frame->data[0] >> 3 & 0x3);
			car_state.ctrmode = !(frame->data[0] >> 5 & 0x1);
			car_state.holdcar = (frame->data[0] >> 6 & 0x1); 
			// printf("workmode=%d\n",car_state.workmode);
			for (int i = 0; i < 4; i++)
				car_state.speed[i] = char(frame->data[1 + i]) * 0.1;
			carstate_pub.publish(car_state);
			// printf("%02x \n",car_state.turnmode);

			// int id = frame->data[5] * 256 + frame->data[6];
 			// int charge = frame->data[6];   //充电
		}
		else if (frame->can_id == 0x88421C0A) //  Paw state
		{
			nodecheck->Find("can_rate")->Beat();
			// printf("six_bite=%d\n",frame->data[6]);

			paw_state.updown_state = frame->data[0];
			paw_state.left_axischange_state = frame->data[1];
			paw_state.carhold_state = frame->data[2];               //0x00无动作 0x01自动取车中 0x02自动取车完成0x03自动放车中 0x04自动放车完成
			paw_state.right_axischange_state = frame->data[3];
			paw_state.paw_state=frame->data[5];                     //liuzhi_1109::0收回 1张开 2动作中
			
			// ROS_INFO("carhold_state=%d",paw_state.carhold_state);
			pawstate_pub.publish(paw_state);
		}
		else if (frame->can_id == 0x88431C0A) //  电池状态
		{
			nodecheck->Find("can_rate")->Beat();

			data_comm::battery_info msg;
			msg.SOC = frame->data[0];
			msg.current = (frame->data[1] * 256 + frame->data[2]) * 0.1;
			msg.voltage = (frame->data[3] * 256 + frame->data[4]) * 0.1;
			msg.SOH = frame->data[5];
			msg.DCDC = frame->data[6];
			battery_pub.publish(msg);
			// printf("AAA=%d\n", msg.SOC);
		}
		else if (frame->can_id == 0x88441C0A)  //  故障码
		{
            nodecheck->Find("can_rate")->Beat();

			car_state.errcode=frame->data[0]*256*256*256;
			car_state.errcode+=frame->data[1]*256*256;
			car_state.errcode+=frame->data[2]*256;
			car_state.errcode+=frame->data[3];
		}
		else if (frame->can_id == 0x88451C0A)
		{
            nodecheck->Find("can_rate")->Beat();

			car_state.odom = (frame->data[0] + frame->data[1] * 256)*0.1;		   // 工作里程  单位：0.1公里
			car_state.runningtime = (frame->data[3] + frame->data[4] * 256)*0.1;   // 工作时间  单位：小时
			// ROS_INFO("%.1f %.1f", car_state.odom, car_state.runningtime);    
			// paw_state.paw_state=frame->data[5];
		}
		else if (frame->can_id ==0x88461C0A) //车轮转角
		{
			nodecheck->Find("can_rate")->Beat();
			if(frame->data[1]>127) car_angle.angle[0] = -((256-frame->data[0]) + (255-frame->data[1]) * 256)*0.1;
			else car_angle.angle[0] = (frame->data[0] + frame->data[1] * 256)*0.1; 
			if(frame->data[3]>127) car_angle.angle[1] = -((256-frame->data[2]) + (255-frame->data[3]) * 256)*0.1;
			else car_angle.angle[1] = (frame->data[2] + frame->data[3] * 256)*0.1; 
			if(frame->data[5]>127) car_angle.angle[2] = -((256-frame->data[4]) + (255-frame->data[5]) * 256)*0.1;
			else car_angle.angle[2] = (frame->data[4] + frame->data[5] * 256)*0.1; 
			if(frame->data[7]>127) car_angle.angle[3] = -((256-frame->data[6]) + (255-frame->data[7]) * 256)*0.1;
			else car_angle.angle[3] = (frame->data[6] + frame->data[7] * 256)*0.1; 
			
			float x0=5438.0/2;
			float s1=x0/(tan(car_angle.angle[0]*M_PI/180));
			float s2=x0/(tan(car_angle.angle[2]*M_PI/180));
			float s3=2*x0/(s1+s2);
			car_angle.angle[4] = 180/M_PI*atan(s3);
			// ROS_INFO("angle=%.2f %.2f %.2f %.2f %.2f",car_angle.angle[0],car_angle.angle[1],car_angle.angle[2],car_angle.angle[3],car_angle.angle[4]);
			carangle_pub.publish(car_angle);
		}

		frame->can_id = 0;
	}
}

void can0_restart()
{
    system("sudo ifconfig can0 down");
    system("sudo tc qdisc del dev can0 root");
    system("sudo ip link set can0 type can restart-ms 100");
    system("sudo ip link set can0 type can bitrate 250000 sample-point 0.75");
    system("sudo ip link set can0 type can berr-reporting on");
    system("sudo ifconfig can0 txqueuelen 1000");
    system("sudo tc qdisc add dev can0 root handle 1: pfifo");
    system("sudo ifconfig can0 up");
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "plus1");

	nh = new ros::NodeHandle("~");
	nodecheck = new TNodeCheck(nh, "node_rate can_rate");
	nodecheck->Find("node_rate")->SetLimit(10);

	int can_ch = 0;
	can = InitCan(can_ch, 500000);

	ros::Subscriber pawctr_sub = nh->subscribe("/pawcontrol/paw_ctr", 10, &PawCtrCallback);
	ros::Subscriber carctr_sub = nh->subscribe("/pathtrack/ctr_cmd", 1, &CarCtrCallback);

	carstate_pub = nh->advertise<data_comm::car_state>("car_state", 10);
	carangle_pub = nh->advertise<data_comm::car_angle>("car_angle", 10);
	pawstate_pub = nh->advertise<data_comm::paw_state>("paw_state", 10);
	battery_pub = nh->advertise<data_comm::battery_info>("battery_info", 10);

	TTimer can_tmr;

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		nodecheck->Find("node_rate")->Beat();
		if (car_state.workmode!=3 || car_state.turnmode!=car_cmd.turnmode) // 转运车不在工作状态 或者在转向切换过程中
		{
			car_cmd.speed=car_cmd.angle=0;
		}

		car_cmd.pawmode = 1;
		// car_cmd.workmode = 3;
		// car_cmd.turnmode = 0;
		// car_cmd.speed = 0.6;
		// car_cmd.angle = 0;

		SetWorkMode(car_cmd);
		SetRunPara(car_cmd);
		SetPawMode(paw_ctr);

		ProcCanMsg();

		if(nodecheck->Find("can_rate")->value>20)  can_tmr.Clear();
		else if(can_tmr.GetValue()>4)  can0_restart(), can_tmr.Clear();

        // if(nodecheck->Find("node_rate")->value<5)  ROS_ERROR_STREAM_THROTTLE(1, "CAN comm error!");

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}