#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QDebug>
#include <std_msgs/String.h>
#include <common/public.h>

#include "turntable_panel.h"

using namespace std;

namespace rviz_gui
{

// 构造函数，初始化变量
Panel_Turntable::Panel_Turntable(QWidget *parent)
    : rviz::Panel(parent), ui(new Ui::Panel_TurnTable)
{
    ui->setupUi(this);

    nh = new ros::NodeHandle();
    nh_local = new ros::NodeHandle("~");

    sub = nh->subscribe("/turntable/table_state", 10, &Panel_Turntable::TTCallback, this);
    pub = nh->advertise<std_msgs::String>("/turntable_ctr", 10);

    qtmr.start(20);
    connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));

    connect(ui->Btn_Azimuth_Check, SIGNAL(clicked()), this, SLOT(btn_azimuth_check_onclick()));
    connect(ui->Btn_Azimuth_Zero, SIGNAL(clicked()), this, SLOT(btn_azimuth_zero_onclick()));
    connect(ui->Btn_Azimuth_Stop, SIGNAL(clicked()), this, SLOT(btn_azimuth_stop_onclick()));
    connect(ui->Btn_Azimuth_Enable, SIGNAL(clicked()), this, SLOT(btn_azimuth_enable_onclick()));
    connect(ui->Btn_Azimuth_Disable, SIGNAL(clicked()), this, SLOT(btn_azimuth_disable_onclick()));
    connect(ui->Btn_Azimuth_Move, SIGNAL(clicked()), this, SLOT(btn_azimuth_move_onclick()));
  
    connect(ui->Btn_Pitch_Check, SIGNAL(clicked()), this, SLOT(btn_pitch_check_onclick()));
    connect(ui->Btn_Pitch_Zero, SIGNAL(clicked()), this, SLOT(btn_pitch_zero_onclick()));
    connect(ui->Btn_Pitch_Stop, SIGNAL(clicked()), this, SLOT(btn_pitch_stop_onclick()));
    connect(ui->Btn_Pitch_Enable, SIGNAL(clicked()), this, SLOT(btn_pitch_enable_onclick()));
    connect(ui->Btn_Pitch_Disable, SIGNAL(clicked()), this, SLOT(btn_pitch_disable_onclick()));
    connect(ui->Btn_Pitch_Move, SIGNAL(clicked()), this, SLOT(btn_pitch_move_onclick()));

}

void Panel_Turntable::qtmrfunc()
{
    if(turntable_cmd=="")  return;
    //ROS_INFO("%s",turntable_cmd.c_str());


    std_msgs::String msg;
    msg.data = turntable_cmd;
    pub.publish(msg);
    turntable_cmd="";
}

void Panel_Turntable::TTCallback(const std_msgs::StringConstPtr &msg)
{
    vector<string> ss=split(msg->data,";");
    char buf[500];
    
    for(int i=0;i<ss.size();i++)
    {
        // printf("%s\n",ss[i].c_str());
        vector<string> subs=split(ss[i]," ");
        float angle=atof(subs[2].c_str());
        int r=atof(subs[3].c_str());
        int e=atoi(subs[1].c_str());
        if(subs[0]=="azimuth") 
        {
            sprintf(buf,"Azimuth: %5.1f  reach: %d  Enabled: %d ",angle, r, e);
            ui->Label_Azimuth_Angle->setText(QString::fromUtf8(buf));
        }
        else if(subs[0]=="pitch") 
        {
            sprintf(buf,"Pitch:  %5.1f  reach: %d  Enabled: %d ",angle, r, e);
            ui->Label_Pitch_Angle->setText(QString::fromUtf8(buf));
        }
        
    }
}

void Panel_Turntable::btn_azimuth_check_onclick()
{
    turntable_cmd = "AZIMUTH CHECK 30 -30 20";
}

void Panel_Turntable::btn_azimuth_zero_onclick()
{
    turntable_cmd = "AZIMUTH MOV 0 20";
}

void Panel_Turntable::btn_azimuth_stop_onclick()
{
    turntable_cmd = "AZIMUTH STOP";
}

void Panel_Turntable::btn_azimuth_enable_onclick()
{
    turntable_cmd = "AZIMUTH ENABLE 1";
}

void Panel_Turntable::btn_azimuth_disable_onclick()
{
    turntable_cmd = "AZIMUTH ENABLE 0";
}

void Panel_Turntable::btn_azimuth_move_onclick()
{
    float angle=atof(ui->edt_azimuth_pos->text().toStdString().c_str());
    char buf[200];
    sprintf(buf,"AZIMUTH MOV %.2f 20", angle);
    turntable_cmd = buf;
}

void Panel_Turntable::btn_pitch_check_onclick()
{
    turntable_cmd = "PITCH CHECK 10 -30 30";
}

void Panel_Turntable::btn_pitch_zero_onclick()
{
    turntable_cmd = "PITCH MOV 0 10";    
}

void Panel_Turntable::btn_pitch_stop_onclick()
{
    turntable_cmd = "PITCH STOP";
}

void Panel_Turntable::btn_pitch_enable_onclick()
{
    turntable_cmd = "PITCH ENABLE 1";
}

void Panel_Turntable::btn_pitch_disable_onclick()
{
    turntable_cmd = "PITCH ENABLE 0";
}

void Panel_Turntable::btn_pitch_move_onclick()
{
    float angle=atof(ui->edt_pitch_pos->text().toStdString().c_str());
    char buf[200];
    sprintf(buf,"PITCH MOV %.2f 10", angle);
    turntable_cmd = buf;
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gui::Panel_Turntable, rviz::Panel)
// END_TUTORIAL
