#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QListWidgetItem>
#include "utils/config.h"

#include <unistd.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


    QTimer *mTime;


    void closeEvent(QCloseEvent *event);

private slots:

    void OnPointTempInfo(int x,int y,float temp);

    void OnConnectStatusChange(int status,int deviceId);

    void OnSearchCameraIP(QString ip);

    void OnCameraLogInfo(QString msg);

    void OnRS485Receive(QString msg);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_comboBox_currentIndexChanged(const QString &arg1);

    void on_comboBox_currentIndexChanged(int index);

    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_12_clicked();



    void on_ems_pushbutton_clicked();

    void on_distance_pushbutton_clicked();

    void on_transmit_pushbutton_clicked();

    void on_envtemp_pushbutton_clicked();

    void on_relhum_pushbutton_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_14_clicked();

    void on_pushButton_16_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_18_clicked();

    void on_pushButton_19_clicked();

    void on_pushButton_20_clicked();

    void on_pushButton_21_clicked();

    void on_relhum_pushbutton_2_clicked();

    void on_pushButton_22_clicked();

    void on_framerate_pushbutton_clicked();

    void on_pushButton_23_clicked();

    void on_pushButton_24_clicked();

    void on_pushButton_25_clicked();

    void on_pushButton_26_clicked();

    void on_listWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

    void on_pushButton_get_h264_param_clicked();

    void on_pushButton_set_h264_param_clicked();

    void on_pushButton_reboot_clicked();

    void on_pushButton_27_clicked();

    void on_pushButton_29_clicked();

    void on_pushButton_30_clicked();

    void on_pushButton_28_clicked();

    void on_pushButton_RTC_clicked();

    void on_pushButton_sd_clicked();

    void on_pushButton_set_record_format_clicked();

    void on_pushButton_31_clicked();

    void on_pushButton_32_clicked();

    void on_pushButton_33_clicked();

    void on_pushButton_get_range_clicked();

    void on_pushButton_set_range_clicked();

    void on_pushButton_34_clicked();

    void on_pushButton_set_focus_moden_clicked();

    void on_pushButton_focus_in_clicked();

    void on_pushButton_focus_out_clicked();

    void on_pushButton_focus_auto_clicked();

    void on_pushButton_region_clicked();



    void on_pushButton_ls_set_clicked();

    void on_pushButton_ls_get_clicked();


    void on_checkBox_ls_stateChanged(int arg1);

    void on_pushButton_35_clicked();

    void on_pushButton_get_serialport_clicked();

    void on_pushButton_set_serialport_clicked();

    void on_pushButton_set_rs485_clicked();

    void on_pushButton_up_clicked();

    void on_pushButton_up_pressed();

    void on_pushButton_up_released();

    void on_pushButton_right_pressed();

    void on_pushButton_right_released();

    void on_pushButton_left_pressed();

    void on_pushButton_left_released();

    void on_pushButton_down_pressed();

    void on_pushButton_down_released();

    void on_pushButton_pitch_get_clicked();

    void on_pushButton_hor_get_clicked();

    void on_pushButton_preset_set_clicked();

    void on_pushButton_preset_get_clicked();

    void on_pushButton_live_stop_clicked();

    void on_pushButton_transport_stop_clicked();

    void on_pushButton_disconnect_clicked();

    void on_pushButton_preset_del_clicked();

    void on_pushButton_leftup_clicked();

    void on_pushButton_leftdown_clicked();

    void on_pushButton_rightup_clicked();

    void on_pushButton_rightdown_clicked();

    void on_pushButton_485logclear_clicked();

    void on_pushButton_left_clicked();

    void on_pushButton_leftup_pressed();

    void on_pushButton_leftup_released();

    void on_pushButton_rightup_pressed();

    void on_pushButton_rightup_released();

    void on_pushButton_leftdown_pressed();

    void on_pushButton_leftdown_released();

    void on_pushButton_rightdown_pressed();

    void on_pushButton_rightdown_released();

    void on_comboBox_adtransfer_currentIndexChanged(int index);

    void on_pushButton_jpgtransfermodeget_clicked();

    void on_pushButton_jpgtransfermodeset_clicked();

    void on_pushButton_jpgtransferparamget_clicked();

    void on_pushButton_pitch_jpgtransferparamset_clicked();

    void on_pushButton_jpgtransfershow_clicked();

    void on_pushButton_jpgtransfernotshow_clicked();

private:
    Ui::MainWindow *ui;

    Config *config;

    bool autoQueryJPG = false;

private:
    void addLogInfo(QString msg);

    int hexStringToBytes(QString hex,unsigned char *buf);

    qint64 getMillTimeStamp();

    void ptzControl(int type,unsigned char mode,quint8 speed);

    void ptzControl(int type,quint8 speed);

    void ptzControl(int type);

signals:
    void sgnVideoData(uchar* rgbData, int w, int h, int macIndex,int type);

    void sgnConnectStatusChange(int status,int deviceId);


};


















#endif // MAINWINDOW_H
