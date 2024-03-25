#include "node_check.h"

dialog_node_check::dialog_node_check(QWidget *parent) :
    QDialog(parent),
    ui(new Ui_Dialog_NodeCheck)
{
    ui->setupUi(this);
    nh=new ros::NodeHandle("~");

    qtmr.start(100);
    connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));
}

void dialog_node_check::update_rate(string param, QLabel *lab)
{
    char buf[200];
    float value = 0;
    nh->getParam(param, value);
    sprintf(buf, "%.1f", value);
    lab->setText(buf);
}

void dialog_node_check::qtmrfunc()
{
    update_rate("/gps_pro/check/node_rate", ui->label_gps_hb);
    update_rate("/cloud_tf1/check/rslidar_rate", ui->label_lidar1_hb);
    update_rate("/cloud_tf2/check/rslidar_rate", ui->label_lidar2_hb);
    update_rate("/cloud_tf3/check/rslidar_rate", ui->label_lidar3_hb);
    update_rate("/cloud_tf4/check/rslidar_rate", ui->label_lidar4_hb);

    update_rate("/laserscan_check_angle/check/scan1_rate", ui->label_scan1_hb);
    update_rate("/laserscan_check_angle/check/scan2_rate", ui->label_scan2_hb);
    update_rate("/laserscan_check_angle/check/scan3_rate", ui->label_scan3_hb);
    update_rate("/laserscan_check_angle/check/scan4_rate", ui->label_scan4_hb);

    update_rate("/rfid_reader_tcp/check/rfid_rate", ui->label_RFID_hb);
 
    update_rate("/local_path_plan/check/node_rate", ui->label_pathplan_hb);
    update_rate("/pathtrack/check/node_rate", ui->label_pathtrack_hb);
    update_rate("/pawcontrol/check/node_rate", ui->label_work_hb);

    update_rate("/laserscan_check/check/node_rate", ui->label_scancheck_hb);
    update_rate("/laserscan_check_angle/check/node_rate", ui->label_scancheckangle_hb);
    update_rate("/laserscan_check_angle/check/node_rate", ui->label_scancheckangle_hb);
    
    update_rate("/cloud_segmentation/check/node_rate", ui->label_cloudsegmentation_hb);
    update_rate("/cloud_calculation_inside/check/node_rate", ui->label_cloudinside_hb);
    update_rate("/cloud_calculation_front/check/node_rate", ui->label_cloudfront_hb);
    update_rate("/cloud_calculation_back/check/node_rate", ui->label_cloudback_hb);
    update_rate("/cloud_calculation_left/check/node_rate", ui->label_cloudleft_hb);
    update_rate("/cloud_calculation_right/check/node_rate", ui->label_cloudright_hb);
    update_rate("/cloud_calculation_marker/check/node_rate", ui->label_cloudmarker_hb);
    update_rate("/mi_stop/check/node_rate", ui->label_mistop_hb);

    update_rate("/can_comm/check/can_rate", ui->label_plc_hb);
    update_rate("/iot_comm/check/iotheart_rate", ui->label_mqtt_hb);
}

dialog_node_check::~dialog_node_check()
{
    delete ui;
}
