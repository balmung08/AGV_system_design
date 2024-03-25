#include <common/public.h>
#include <common/myudp.h>
#include <common/can.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <car_ctr/car_state.h>
#include <car_ctr/car_ctr.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>


const float RPM2SPD = 3.1416 * 0.24 / 60;
const float SPD2RPM = 1 / RPM2SPD;

class TCarCtr 
{
private:
    TUDP *udp;
    TTimer delay_time;
    ros::NodeHandle *nh;
	
    ros::Subscriber carctr_sub;
    ros::Publisher carstate_pub;

    car_ctr::car_ctr car_ctr;
    car_ctr::car_state car_state;
    bool can_enable=false;
    TCan *can=NULL;
    float mode_change_time=2.5;

    void CarCmdCallback(const car_ctr::car_ctr::ConstPtr &msg)
    {
        car_ctr=*msg;
        char str[50]={0};

        if(car_state.turnmode!=car_ctr.turnmode)
        {
            sprintf(str, "Car SteerModeCtrl %d",car_ctr.turnmode);
            udp->Send(str);
            delay_time.Clear();
        }
        else if(delay_time.GetValue() > mode_change_time && (car_ctr.turnmode==0 || car_ctr.turnmode==1)) 
        {
            sprintf(str, "Car Run %.2f %.2f", msg->speed, msg->angle);
            udp->Send(str);
        }
        else if(delay_time.GetValue() > mode_change_time && (car_ctr.turnmode==2 || car_ctr.turnmode==4)) 
        {   
            sprintf(str, "Car Run %.2f 0.0", msg->speed);  
            udp->Send(str);
        }
    }

public:
    TCarCtr()
    {
        nh=new ros::NodeHandle("~");

        nh->getParam("can_enable",can_enable);
        if(can_enable)
        {  
            can = InitCan(0, 500000);
        }
        else
        {
            string remote_ip=""; 
            nh->getParam("remote_ip",remote_ip);
            udp = new TUDP(8090);
            udp->AddRemoteIP(remote_ip, 8080);

            udp->Send("OK");
            usleep(10000);
            carstate_pub = nh->advertise<car_ctr::car_state>("/car_state", 10);
            carctr_sub = nh->subscribe<car_ctr::car_ctr>("/car_cmd", 10, &TCarCtr::CarCmdCallback, this);
            Enable(true);
        }
    }

    
    void Stop(int flag)
    {
        char str[50];
        sprintf(str, "Car Stop %d", flag);
        udp->Send(str);
    }

    void Enable(bool e)
    {
        char str[50];
        if (e)  sprintf(str, "Car Enable");
        else  sprintf(str, "Car Disable");
        udp->Send(str);
        usleep(2000);
    }

    void UDP_Proc() //  用于处理小车上传的状态信息
    {
        if (udp->rec_count == 0)  return;
        udp->rec_count = 0;

        // car_ctr::car_state car_state;
    
        char recbuf[1000];
        strcpy(recbuf, udp->rec_buf);

        vector<string> strs = split(recbuf, ";");
        // ROS_INFO("%s %d", recbuf, strs.size());
        // return;
        
        for (int i = 0; i < strs.size(); i++)
        {
            vector<string> substrs = split(strs.at(i), " ");
            if (substrs.size()>=4 && substrs[0] == "Car")
            {
                car_state.enable = atoi(substrs.at(1).c_str())==8;
                car_state.ctrmode = atoi(substrs.at(2).c_str());
                car_state.turnmode = atoi(substrs.at(3).c_str());
            }
            if (substrs.size()>0 && substrs[0].find("Wheel") != -1) // || substrs[0].substr(0,4)=="Turn" )
            {
                int len = substrs[0].length();
                int id = atoi(substrs[0].substr(len - 1, len).c_str());
                // car_state_msg.WheelMotor_Enable[id] = atoi(substrs.at(1).c_str());
                car_state.speed[id] = atof(substrs.at(2).c_str()) * RPM2SPD;
            }
        }
            
        carstate_pub.publish(car_state);
    }

    void Can_Proc()
    {

    }

    void run()
    {
        if(can_enable)  Can_Proc();
        else  UDP_Proc();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Car_Ctr");
    TCarCtr myCar;

    ros::Rate rate(20);
    while (ros::ok())
    {
        myCar.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
