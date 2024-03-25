#ifndef ABC_H
#define ABC_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <std_msgs/String.h>
#include <QTimer>
#endif

#include "ui_turntable.h"

using namespace std;


namespace rviz_gui
{
// 所有的plugin都必须是rviz::Panel的子类
class Panel_Turntable: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    Panel_Turntable(QWidget *parent = 0);

// 公共槽.
public Q_SLOTS:


// 内部槽.
protected Q_SLOTS:
    void qtmrfunc();

    void btn_azimuth_check_onclick();
    void btn_azimuth_zero_onclick();
    void btn_azimuth_stop_onclick();
    void btn_azimuth_enable_onclick();
    void btn_azimuth_disable_onclick();
    void btn_azimuth_move_onclick();

    void btn_pitch_check_onclick();
    void btn_pitch_zero_onclick();
    void btn_pitch_stop_onclick();
    void btn_pitch_enable_onclick();
    void btn_pitch_disable_onclick();
    void btn_pitch_move_onclick();

    // 内部变量.
protected:
    ros::NodeHandle *nh, *nh_local;
    ros::Subscriber sub;
    ros::Publisher pub;

    void TTCallback(const std_msgs::StringConstPtr &msg);

private:
    Ui::Panel_TurnTable *ui;
    QTimer qtmr;
    string turntable_cmd="";
};

} 

#endif 
