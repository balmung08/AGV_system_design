#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <IRNET.H>
#include <QMessageBox>
#include <QCloseEvent>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include "irnetsdkmanager.h"
#include "playerwidget.h"
#include <thread>
#include <QVariant>

#define Test_ID  1

using namespace IRNet;

#define DEVIDE_ID1   1

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //ui->listWidget_2->setAutoScroll(true);


    config = new Config();


    IRNetSDKManager::GetInstance()->Init();

    ui->player_widget_1->SetDeviceId(DEVIDE_ID1);

    QStringList strList;
    strList<<"1"<<"2"<<"3"<<"4"<<"5"<<"6"<<"7"<<"8"<<"9"<<"10"<<"11"<<"12"<<"13"<<"14"<<"15";
    ui->comboBox->addItems(strList);

    QStringList recordFormatList;
    recordFormatList<<"H264"<<"mp4";
    ui->comboBox_record_fotmat->addItems(recordFormatList);

    QStringList serialportList;
    serialportList<<""<<"RS485";
    ui->comboBox_serialport->addItems(serialportList);


    QStringList strRange;
    strRange<<""<<"1"<<"2";
    ui->comboBox_range->addItems(strRange);

    QStringList listFocusModen;
    listFocusModen<<""<<"手动调焦"<<"自动调焦";
    ui->comboBox_focus_moden->addItems(listFocusModen);


    QStringList rtpModen;
    rtpModen<<"UDP"<<"TCP";
    ui->comboBox_rtp->addItems(rtpModen);

    QStringList adtransferModen;
    adtransferModen<<"UDP"<<"TCP";
    ui->comboBox_adtransfer->addItems(adtransferModen);

    QStringList presets;
    for(int i=1;i<=255;i++){
        presets.append(QString::number(i));
    }
    ui->comboBox_preset->addItems(presets);

    connect(IRNetSDKManager::GetInstance(),SIGNAL(sgnConnectStatusChange(int,int)),this,SLOT(OnConnectStatusChange(int,int)));
    connect(IRNetSDKManager::GetInstance(),SIGNAL(sgnSearchCameraIP(QString)),this,SLOT(OnSearchCameraIP(QString)));
    connect(IRNetSDKManager::GetInstance(),SIGNAL(sgnCameraLogInfo(QString)),this,SLOT(OnCameraLogInfo(QString)));
    connect(IRNetSDKManager::GetInstance(),SIGNAL(sgnRS485Receive(QString)),this,SLOT(OnRS485Receive(QString)));

    connect(ui->player_widget_1,SIGNAL(sgnPointTempInfo(int,int,float)),this,SLOT(OnPointTempInfo(int,int,float)));

    ui->version_label->setText(IRNetSDKManager::GetInstance()->GetSDKVersion());

    QString ip = config->Get(Config::NODE_DEVICE,Config::IP_KEY).toString();
    int model = config->Get(Config::NODE_DEVICE,Config::MODEL_KEY).toInt();
    ui->lineEdit_ip->setText(ip);
    ui->comboBox_model->setCurrentIndex(model);

    QStringList jpgtransferModen;
    jpgtransferModen<<"UDP"<<"TCP";
    ui->comboBox_jpgtransfermode->addItems(jpgtransferModen);

    QStringList jpgtransferEnable;
    jpgtransferEnable<<"禁用"<<"启用";
    ui->comboBox_jpgtransferenable->addItems(jpgtransferEnable);

    ip = ui->lineEdit_ip->text();
    // std::cout<<"ip="<<ip<<std::endl;
    model = ui->comboBox_model->currentIndex();
    std::cout<<"model="<<model<<std::endl;
    ui->player_widget_1->Connect(ip,2100,model,5000);

    usleep(1000);

    ui->player_widget_1->StartTemperatureMeasurement();
}

void MainWindow::OnSearchCameraIP(QString ip){

    ui->listWidget->addItem(ip);

}

void MainWindow::OnCameraLogInfo(QString msg){

    addLogInfo(msg);

}

void MainWindow::OnRS485Receive(QString msg)
{
    ui->listWidget_485log->addItem(msg);
    ui->listWidget_485log->scrollToBottom();

}


void MainWindow::OnConnectStatusChange(int status,int deviceId){

    QString text;
    switch (status) {
    case 0:
    {

        text = "CONNECTED";
        double ems = ui->player_widget_1->GetEmissivity();
        double distance = ui->player_widget_1->GetDistance();
        double transmit = ui->player_widget_1->GetTransmit();
        double env = ui->player_widget_1->GetEnvironmentTemperature();
        double relHum = ui->player_widget_1->GetRelativeHumidity();

        ui->ems_lineEdit->setText(QString::number(ems,'f',2));
        ui->distance_lineEdit->setText(QString::number(distance));
        ui->transmit_lineEdit->setText(QString::number(transmit,'f',2));
        ui->envTemp_lineEdit->setText(QString::number(env));
        ui->relHumidity_lineEdit->setText(QString::number(relHum));

        QString ip = ui->lineEdit_ip->text();
        int model = ui->comboBox_model->currentIndex();

        config->Set(Config::NODE_DEVICE,Config::IP_KEY,QVariant(ip));
        config->Set(Config::NODE_DEVICE,Config::MODEL_KEY,QVariant(model));


//        ui->player_widget_1->StartTemperatureMeasurement();

//        mTime = new QTimer();
//        connect(mTime,&QTimer::timeout,this,[=]()
//        {
//            on_pushButton_23_clicked();
//        });
//        mTime->start(10);






    }

        break;
    case 1:
        text = "CONNECTING";
        break;
    case 2:
        text = "NOT_CONNECTED";
        break;
    case 3:
        text = "FAIL_CONNECTED";
        break;
    case 4:
        text = "DISCONNECR";

        break;
    default:
        break;
    }



    ui->label->setText(text);

}


void MainWindow::closeEvent(QCloseEvent *event)
{
    delete config;
}

void MainWindow::OnPointTempInfo(int x, int y, float temp)
{

    QString info = QString("x=%0 y=%1 温度=%2").arg(x).arg(y).arg(temp);
    ui->label_tempinfo->setText(info);
}


MainWindow::~MainWindow()
{
    IRNetSDKManager::GetInstance()->destroy();
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QString ip = ui->lineEdit_ip->text();
    int model = ui->comboBox_model->currentIndex();
    std::cout<<"model1="<<model<<std::endl;
    ui->player_widget_1->Connect(ip,2100,model,5000);
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->player_widget_1->setRtpOverMode(ui->comboBox_rtp->currentIndex());

    ui->player_widget_1->RealPlay();
}

void MainWindow::on_pushButton_3_clicked()
{
    ui->player_widget_1->StartTemperatureMeasurement();

}

void MainWindow::on_pushButton_4_clicked()
{
    QString path = QCoreApplication::applicationDirPath()+"/test.jpg";
    ui->player_widget_1->SnapJpegTemperature(path.toUtf8().data());
}

void MainWindow::on_pushButton_5_clicked()
{
    QList<QPoint> points;
    points.append(QPoint(100,100));
    points.append(QPoint(200,200));
    ui->player_widget_1->AddRectAnalysis(points);
}

struct Test{
    int a = 0;
    int b = 0;
};

void MainWindow::on_pushButton_6_clicked()
{

    //mTime->stop();

    //IRNet::test(1,1);


    //  ptzControl(PTZControl::PTZ_RIGHT_DOWN,20);


    //    on_pushButton_7_clicked();

    //    std::thread th([=](){

    //        while (1) {

    //            //qDebug()<<"Machine SN: "<< QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss:zzz") <<ui->player_widget_1->GetMachineSN();

    //            on_pushButton_clicked();

    //            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    //            on_pushButton_3_clicked();




    //            std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    //            on_pushButton_disconnect_clicked();

    //            std::this_thread::sleep_for(std::chrono::milliseconds(1000));



    //        }

    //    });
    //    th.detach();

}

void MainWindow::on_pushButton_7_clicked()
{
    ui->player_widget_1->SetFollowMaxTemp(true);
}

void MainWindow::on_pushButton_8_clicked()
{
    ui->player_widget_1->SetFollowMinTemp(true);
}

void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{

}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    ui->player_widget_1->SetPalette(index);
}

void MainWindow::on_pushButton_9_clicked()
{
    if(ui->pushButton_9->text() == "视频录制"){
        QString path = QCoreApplication::applicationDirPath()+"/test.mp4";
        int ret = ui->player_widget_1->Record264video(path.toUtf8().data(),-1);
        if(ret ==0 ){
            ui->pushButton_9->setText("停止");
        }
    }else{
        ui->player_widget_1->StopRecord264Video();
        ui->pushButton_9->setText("视频录制");
    }

}

void MainWindow::on_pushButton_10_clicked()
{
    QList<QPoint> list;
    list.append(QPoint(300,300));
    ui->player_widget_1->AddPointAnalysis(list);
}

void MainWindow::on_pushButton_11_clicked()
{
    QList<QPoint> list;
    list.append(QPoint(200,300));
    list.append(QPoint(260,400));
    ui->player_widget_1->AddLineAnalysis(list);
}


void MainWindow::on_pushButton_12_clicked()
{
    QList<QPoint> list;
    list.append(QPoint(300,200));
    list.append(QPoint(400,200));
    ui->player_widget_1->AddCireAnalysis(list);
}

void MainWindow::on_pushButton_31_clicked()
{
    QList<QPoint> list;
    list.append(QPoint(300,200));
    list.append(QPoint(300,300));
    list.append(QPoint(400,200));
    ui->player_widget_1->AddPolyAnalysis(list);

}

void MainWindow::on_ems_pushbutton_clicked()
{
    double ems = ui->ems_lineEdit->text().toDouble();
    //IRNet::setEmissivity(Test_ID,ems);
    ui->player_widget_1->SetEmissivity(ems);
}

void MainWindow::on_distance_pushbutton_clicked()
{
    double distance = ui->distance_lineEdit->text().toDouble();
    //IRNet::setDistance(Test_ID,distance);
    ui->player_widget_1->SetDistance(distance);
}

void MainWindow::on_transmit_pushbutton_clicked()
{
    double transmit = ui->transmit_lineEdit->text().toDouble();
    //IRNet::setTransmit(Test_ID,transmit);
    ui->player_widget_1->SetTransmit(transmit);
}

void MainWindow::on_envtemp_pushbutton_clicked()
{
    double envTemp = ui->envTemp_lineEdit->text().toDouble();
    //IRNet::setEnvironmentTemperature(Test_ID,envTemp);
    ui->player_widget_1->SetEnvironmentTemperature(envTemp);
}

void MainWindow::on_relhum_pushbutton_clicked()
{
    double relHumidity = ui->relHumidity_lineEdit->text().toDouble();
    //IRNet::setRelativeHumidity(Test_ID,relHumidity);
    ui->player_widget_1->SetRelativeHumidity(relHumidity);
}

void MainWindow::on_pushButton_15_clicked()
{

}

void MainWindow::on_pushButton_14_clicked()
{
    ui->player_widget_1->ClearAnalysis();

}

void MainWindow::on_pushButton_16_clicked()
{
    bool is_show = !ui->player_widget_1->IsShowMachineVideo();

    ui->player_widget_1->ShowMachineVideo(is_show);
}

void MainWindow::on_pushButton_17_clicked()
{
    if(ui->pushButton_17->text() == "图像增强"){

        ui->player_widget_1->SetImageEnhancement(true);
        ui->pushButton_17->setText("关闭");

    }else{
        ui->player_widget_1->SetImageEnhancement(false);
        ui->pushButton_17->setText("图像增强");
    }
}

void MainWindow::on_pushButton_18_clicked()
{
    ui->player_widget_1->ClearMachineAnalysis();
}

void MainWindow::on_pushButton_24_clicked()
{

    ui->player_widget_1->GetMachineAnalysis();

}

void MainWindow::on_pushButton_25_clicked()
{
    ui->player_widget_1->SaveMachintAnalysis();
}



void MainWindow::on_pushButton_19_clicked()
{
    ui->player_widget_1->SetMachineColorRegion(1,50,50,200,200);
}

void MainWindow::on_pushButton_20_clicked()
{
    ui->player_widget_1->CancelMachineColorRegion();
}

void MainWindow::on_pushButton_21_clicked()
{
    ui->player_widget_1->disConnectNet();
}


void MainWindow::on_pushButton_22_clicked()
{
    ui->player_widget_1->DoAdjust();

}

void MainWindow::on_framerate_pushbutton_clicked()
{
    int frame_rate = ui->framerate_lineEdit->text().toInt();
    ui->player_widget_1->SetTransmissionFrameRate(frame_rate);
}

void MainWindow::on_relhum_pushbutton_2_clicked(){

}

void MainWindow::on_pushButton_23_clicked()
{
    int size = 1920*1080;
    float *temps = new float[1920*1080];
    long len = ui->player_widget_1->GetAllTemp(temps,size);

    if(len > 0){

        float max = 0;
        float min = 65535;
        for(int i=0;i<len;i++){
            float temp = temps[i];
            if(temp > max){
                max = temp;
            }else if(temp < min){
                min = temp;
            }
        }
        QString msg = QString("buf size %1 max value %2 min value %3").arg(len).arg(max).arg(min);
        addLogInfo(msg);
    }else{
        addLogInfo("获取温度点阵失败");
    }

    delete[] temps;

}

void MainWindow::on_pushButton_26_clicked()
{
    int counter =ui->listWidget->count();
    for(int index=0;index<counter;index++)
    {
        QListWidgetItem *item = ui->listWidget->takeItem(0);
        delete item;
    }

    ui->player_widget_1->SearchCameraIP(5000);
}



void MainWindow::on_listWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
    ui->lineEdit_ip->setText(current->text());
}

void MainWindow::on_pushButton_get_h264_param_clicked()
{
    int value = ui->player_widget_1->getH264EncodecParam();

    ui->lineEdit_h264_param->setText(QString::number(value));

}

void MainWindow::on_pushButton_set_h264_param_clicked()
{

    int value = ui->lineEdit_h264_param->text().toInt();
    ui->player_widget_1->setH264EncodecParam(value);
}

void MainWindow::on_pushButton_reboot_clicked()
{
    ui->player_widget_1->rebootMachine();
}

void MainWindow::on_pushButton_27_clicked()
{
    ui->player_widget_1->shopPhotoOnMachine();
}

void MainWindow::on_pushButton_29_clicked()
{
    QString sn = ui->player_widget_1->GetMachineSN();
    addLogInfo(sn);
}

void MainWindow::on_pushButton_30_clicked()
{
    //ui->player_widget_1->getAnalysisFromMachine();

    std::thread th([=](){


        qint64 time = getMillTimeStamp();

        qDebug()<<"time " << time;

        int count = 30;
        for(int i=0;i<count;i++){
            int size = 1920*1080;
            float *temps = new float[1920*1080];
            long len = ui->player_widget_1->GetAllTemp(temps,size);

            if(len > 0){
                float a2 = temps[len-1];
                float a1 = temps[0];
                QString msg = QString("buf size %1 first value %2 last value %3").arg(len).arg(a1).arg(a2);
                qDebug()<<msg;
            }else{
                qDebug()<<"获取温度点阵失败";
            }

            delete[] temps;

        }

        qint64 delay = getMillTimeStamp() - time;
        qDebug()<<"getalltemp time " << delay;
    });
    th.detach();

}

void MainWindow::on_pushButton_28_clicked()
{
    if(ui->pushButton_28->text() == "机芯录像"){
        ui->pushButton_28->setText("停止");
        ui->player_widget_1->startRecordOnMachine();
    }else {
        ui->pushButton_28->setText("机芯录像");
        ui->player_widget_1->stopRecordOnMachine();
    }
}

void MainWindow::on_pushButton_RTC_clicked()
{
    ui->player_widget_1->setRTCTime();
}

void MainWindow::on_pushButton_sd_clicked()
{
    int space = ui->player_widget_1->getSDAvailableSpaceOfMachine();
    double a = space / 1024 / 1024 / 1024.0;
    qDebug()<<"getSDAvailableSpaceOfMachine:"<< a <<"Gb";
}

void MainWindow::on_pushButton_set_record_format_clicked()
{
    int format = ui->comboBox_record_fotmat->currentIndex();
    ui->player_widget_1->setVideoFormatOfMachine(format);
}

bool RunTask = false;

void MainWindow::on_pushButton_32_clicked()
{
    RunTask = true;

    std::thread th([=](){

        int count = 0;

        QString dir = "/home/chen/文档/log/";

        while (RunTask) {

            count ++;

            //            QString ip = ui->lineEdit_ip->text();
            //            int model = ui->comboBox_model->currentIndex();

            qDebug()<<"connectting";
            IRNet::connectWithModel_((char *)"192.168.0.6",2100,DEVIDE_ID1,CameraModel::IR640,3000,false);

            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            qDebug()<<"startTemperatureMeasurement";
            IRNet::startTemperatureMeasurement(DEVIDE_ID1);

            qDebug()<<"realPlay";
            IRNet::realPlay(DEVIDE_ID1);



            std::this_thread::sleep_for(std::chrono::milliseconds(1000));


            QString path = QString("%1%2%3").arg(dir).arg(count).arg(".jpg");

            qDebug()<<"path :"<<path;

            //IRNet::snapJpegTemperature(DEVIDE_ID1,path.toUtf8().data(),2,nullptr,0);

            IRNet::IRAnalysis analysisList[100];


            //获取左上角（50,50）右下角（100,100）的矩形范围内的温度信息
            IRNet::IRAnalysis rect_analysis;
            rect_analysis.points[0].x = 50;
            rect_analysis.points[0].y = 50;
            rect_analysis.points[1].x = 100;
            rect_analysis.points[1].y = 100;
            rect_analysis.type = IRNet::AnalysisType::RectAnalysis;
            rect_analysis.number = 1;

            //获取坐标（120,120）点的温度
            IRNet::IRAnalysis point_analysis;
            point_analysis.points[0].x = 120;
            point_analysis.points[0].y = 120;
            point_analysis.type = IRNet::AnalysisType::PointAnalysis;
            point_analysis.number = 2;

            //获取坐标中心点坐标（180,120）半径50的圆范围内的温度信息
            IRNet::IRAnalysis cire_analysis;
            cire_analysis.points[0].x = 130;
            cire_analysis.points[0].y = 120;
            cire_analysis.points[1].x = 230;
            cire_analysis.points[1].y = 120;
            cire_analysis.type = IRNet::AnalysisType::CireAnalysis;
            cire_analysis.number = 3;

            //获取起点（180,120）终点（230，260）线的温度信息

            IRNet::IRAnalysis line_analysis;
            line_analysis.points[0].x = 180;
            line_analysis.points[0].y = 120;
            line_analysis.points[1].x = 230;
            line_analysis.points[1].y = 260;
            line_analysis.type = IRNet::AnalysisType::LineAnalysis;
            line_analysis.number = 4;

            //获取最高温的温度信息
            IRNet::IRAnalysis max_analysis;
            max_analysis.type = IRNet::AnalysisType::MaxAnalysis;
            max_analysis.number = 5;


            //获取最低温的温度信息
            IRNet::IRAnalysis min_analysis;
            min_analysis.type = IRNet::AnalysisType::MinAnalysis;
            min_analysis.number = 6;


            analysisList[0] = rect_analysis;
            analysisList[1] = point_analysis;
            analysisList[2] = cire_analysis;
            analysisList[3] = line_analysis;
            analysisList[4] = max_analysis;
            analysisList[5] = min_analysis;

            IRNet::getAnalysis(DEVIDE_ID1,analysisList,6);

            //            for (int i = 0; i < 6; ++i) {
            //                IRNet::IRAnalysis* item = &analysisList[i];
            //                qDebug() << "analysis tag:" << item->number << "  type:" << item->type;
            //                qDebug() << "max info temp:" << item->maxPointInfo.temp_ << "  x:" << item->maxPointInfo.x << "  y:" << item->maxPointInfo.y;
            //                qDebug() << "min info temp:" << item->minPointInfo.temp_ << "  x:" << item->minPointInfo.x << "  y:" << item->minPointInfo.y;
            //                qDebug() << "avg temp:" << item->avgTemp_  << "\n";
            //            }


            std::this_thread::sleep_for(std::chrono::milliseconds(10000));


            IRNet::stopPlay(DEVIDE_ID1);
            IRNet::stopTemperatureMeasurement(DEVIDE_ID1);
            IRNet::disConnectNet(DEVIDE_ID1);

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        }

    });
    th.detach();

}

void MainWindow::on_pushButton_33_clicked()
{
    RunTask = false;
}

void MainWindow::on_pushButton_get_range_clicked()
{
    IRRange range = ui->player_widget_1->getRange();
    qDebug()<<"read range gear "<<range.gear;
    //QString rangeStr = QString("%1-%2 ℃").arg(range.low).arg(range.up);
    ui->comboBox_range->setCurrentIndex(range.gear+1);
    ui->lineEdit_range_low->setText(QString::number(range.low));
    ui->lineEdit_range_up->setText(QString::number(range.up));
}

void MainWindow::on_pushButton_set_range_clicked()
{
    int gear = ui->comboBox_range->currentIndex();
    float low = ui->lineEdit_range_low->text().toFloat();
    float up = ui->lineEdit_range_up->text().toFloat();

    if(gear > 0){
        IRRange range;
        range.gear = gear - 1;
        range.low = low;
        range.up = up;
        ui->player_widget_1->setRange(range);
    }

}

void MainWindow::on_pushButton_34_clicked()
{
    long w = 0 , h = 0;
    ui->player_widget_1->GetIrDataLen(&w,&h);
    long len = w*h;
    unsigned short *buf = new unsigned short[len];
    len = ui->player_widget_1->getAllAD(buf,len);
    for(int i=0;i<len;i++){
        qDebug()<<QString("i = %1  ad = %2").arg(i).arg(buf[i]);
    }


}

void MainWindow::on_pushButton_set_focus_moden_clicked()
{
    if(ui->comboBox_focus_moden->currentIndex() > 0){
        int index = ui->comboBox_focus_moden->currentIndex() - 1;
        ui->player_widget_1->focusModen(index);
    }
}

void MainWindow::on_pushButton_focus_in_clicked()
{
    ui->player_widget_1->focusIn();
}

void MainWindow::on_pushButton_focus_out_clicked()
{
    ui->player_widget_1->focusOut();
}

void MainWindow::on_pushButton_focus_auto_clicked()
{
    ui->player_widget_1->focusAuto();
}

void MainWindow::on_pushButton_region_clicked()
{
    bool enable = ui->checkBox_region->isChecked();
    int l = ui->lineEdit_region_l->text().toInt();
    int t = ui->lineEdit_region_t->text().toInt();
    int r = ui->lineEdit_region_r->text().toInt();
    int b = ui->lineEdit_region_b->text().toInt();


    ui->player_widget_1->setEffectiveArea(enable,l,t,r,b);

}


void MainWindow::on_pushButton_ls_set_clicked()
{
    unsigned short level = ui->lineEdit_ls_level->text().toInt();
    unsigned short span  =ui->lineEdit_ls_span->text().toInt();

    ui->player_widget_1->setLevelSpan(level,span);

}

void MainWindow::on_pushButton_ls_get_clicked()
{
    unsigned short max = 0;
    unsigned short min = 0;
    unsigned short level = 0;
    unsigned short span = 0;

    ui->player_widget_1->getLevelSpanInfo(&max,&min,&level,&span);

    ui->lineEdit_ls_max->setText(QString::number(max));
    ui->lineEdit_ls_min->setText(QString::number(min));
    ui->lineEdit_ls_level->setText(QString::number(level));
    ui->lineEdit_ls_span->setText(QString::number(span));

}

void MainWindow::on_checkBox_ls_stateChanged(int arg1)
{
    ui->player_widget_1->setLevelSpanModen(arg1);
}

void MainWindow::addLogInfo(QString msg)
{
    ui->listWidget_2->addItem(msg);

    ui->listWidget_2->scrollToBottom();

}

void MainWindow::on_pushButton_35_clicked()
{
    int counter =ui->listWidget_2->count();
    for(int index=0;index<counter;index++)
    {
        QListWidgetItem *item = ui->listWidget_2->takeItem(0);
        delete item;
    }
}

void MainWindow::on_pushButton_get_serialport_clicked()
{
    int mode = ui->player_widget_1->getSerialPortWorkMode();

    if(-1 == mode){
        ui->comboBox_serialport->setCurrentIndex(0);
    }else if(4 == mode){
        ui->comboBox_serialport->setCurrentIndex(1);
    }



}

void MainWindow::on_pushButton_set_serialport_clicked()
{
    int index = ui->comboBox_serialport->currentIndex();
    if(index > 0){
        int mode = 0;
        if(index == 1){
            mode =  4;
        }
        int ret = ui->player_widget_1->setSerialPortWorkMode(mode);
        qDebug()<<"setSerialPortWorkMode ret "<<ret;
    }

}

int MainWindow::hexStringToBytes(QString hex,unsigned char *buf){
    QString str = hex.remove(QRegExp("\\s"));;
    //字符串转换为hex
    int size = 0;
    for(int i = 0;i < str.size();i += 2)
    {
        uint num = str.mid(i,2).toUInt(nullptr,16);
        buf[i/2] = num&0xFF;
        size++;
    }
    return size;
}

qint64 MainWindow::getMillTimeStamp()
{
    return QDateTime::currentMSecsSinceEpoch();
}



void MainWindow::on_pushButton_set_rs485_clicked()
{

    unsigned char buf[256];
    QString hex = ui->lineEdit_rs485->text();
    int len = hexStringToBytes(hex,buf);
    int ret = ui->player_widget_1->penetrateOnRS485(buf,len);
    //qDebug()<<"penetrateOnRS485 ret "<<ret;
    //    int a = 0;
}

void MainWindow::ptzControl(int type,unsigned char mode,quint8 speed){
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    ui->player_widget_1->ptzControl(num,mode,speed);
}

void MainWindow::ptzControl(int type,quint8 speed)
{
    quint8 num = ui->lineEdit_ptz_num->text().toInt();

    switch (type) {
    case 0:
        ui->player_widget_1->ptzControlLeft(num,speed);
        break;
    case 1:
        ui->player_widget_1->ptzControlTop(num,speed);
        break;
    case 2:
        ui->player_widget_1->ptzControlRight(num,speed);
        break;
    case 3:
        ui->player_widget_1->ptzControlDown(num,speed);
        break;
    case 5:
        ui->player_widget_1->ptzControlStop(num);
        break;
    default:
        quint8 num = ui->lineEdit_ptz_num->text().toInt();
        ui->player_widget_1->ptzControl(num,type,speed);
        break;
    }

}

void MainWindow::on_pushButton_up_clicked()
{

}

void MainWindow::on_pushButton_left_pressed()
{
    quint8 speed = ui->horizontalSlider_ptz_speed->value();
    ptzControl(0,speed);
}

void MainWindow::on_pushButton_left_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_up_pressed()
{
    quint8 speed = ui->horizontalSlider_ptz_speed_2->value();
    ptzControl(1,speed);
}

void MainWindow::on_pushButton_up_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_right_pressed()
{
    quint8 speed = ui->horizontalSlider_ptz_speed->value();
    ptzControl(2,speed);
}

void MainWindow::on_pushButton_right_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_down_pressed()
{
    quint8 speed = ui->horizontalSlider_ptz_speed_2->value();
    ptzControl(3,speed);
}

void MainWindow::on_pushButton_down_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_pitch_get_clicked()
{
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    double value = ui->player_widget_1->getPTZPitchAngle(num);
    ui->lineEdit_pitch_angle->setText(QString::number(value));
}

void MainWindow::on_pushButton_hor_get_clicked()
{
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    double value = ui->player_widget_1->getPTZRollAngle(num);
    ui->lineEdit_hor_angle->setText(QString::number(value));
}

void MainWindow::on_pushButton_preset_set_clicked()
{
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    int index = ui->comboBox_preset->currentIndex()+1;
    ui->player_widget_1->setPTZPreset(num,index);
}

void MainWindow::on_pushButton_preset_get_clicked()
{
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    int index = ui->comboBox_preset->currentIndex()+1;
    ui->player_widget_1->executePTZPreset(num,index);
}

void MainWindow::on_pushButton_live_stop_clicked()
{
    ui->player_widget_1->stopPlay();
}

void MainWindow::on_pushButton_transport_stop_clicked()
{
    ui->player_widget_1->StopTemperatureMeasurement();
}

void MainWindow::on_pushButton_disconnect_clicked()
{
    ui->player_widget_1->disConnectNet();
}

void MainWindow::on_pushButton_preset_del_clicked()
{
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    int index = ui->comboBox_preset->currentIndex()+1;
    ui->player_widget_1->delPTZPreset(num,index);
}

void MainWindow::on_pushButton_leftup_clicked()
{



}

void MainWindow::on_pushButton_leftdown_clicked()
{

}

void MainWindow::on_pushButton_rightup_clicked()
{

}

void MainWindow::on_pushButton_rightdown_clicked()
{


}

void MainWindow::on_pushButton_485logclear_clicked()
{
    int counter =ui->listWidget_485log->count();
    for(int index=0;index<counter;index++)
    {
        QListWidgetItem *item = ui->listWidget_485log->takeItem(0);
        delete item;
    }
}

void MainWindow::on_pushButton_left_clicked()
{

}

void MainWindow::ptzControl(int type){
    quint8 hs = ui->horizontalSlider_ptz_speed->value();
    quint8 vs = ui->horizontalSlider_ptz_speed_2->value();
    quint8 num = ui->lineEdit_ptz_num->text().toInt();
    ui->player_widget_1->executeHorVerPTZControl(num,type,hs,vs);
}

void MainWindow::on_pushButton_leftup_pressed()
{
    ptzControl(IRNet::PTZControl::PTZ_LEFT_UP);
}

void MainWindow::on_pushButton_leftup_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_rightup_pressed()
{
    ptzControl(IRNet::PTZControl::PTZ_RIGHT_UP);
}

void MainWindow::on_pushButton_rightup_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_leftdown_pressed()
{
    ptzControl(IRNet::PTZControl::PTZ_LEFT_DOWN);
}

void MainWindow::on_pushButton_leftdown_released()
{
    ptzControl(5,0);
}

void MainWindow::on_pushButton_rightdown_pressed()
{
    ptzControl(IRNet::PTZControl::PTZ_RIGHT_DOWN);
}

void MainWindow::on_pushButton_rightdown_released()
{
    ptzControl(5,0);
}

void MainWindow::on_comboBox_adtransfer_currentIndexChanged(int index)
{
    ui->player_widget_1->setADTransferModen(index);
}

void MainWindow::on_pushButton_jpgtransfermodeget_clicked()
{

    unsigned char useTcp = 0;
    int ret = ui->player_widget_1->getJPGTransferMode(&useTcp);
    if(ret == 0){
        ui->comboBox_jpgtransfermode->setCurrentIndex(useTcp);
    }

}

void MainWindow::on_pushButton_jpgtransfermodeset_clicked()
{
    unsigned char index = ui->comboBox_jpgtransfermode->currentIndex();
    ui->player_widget_1->setJPGTransferMode(index);

}

void MainWindow::on_pushButton_jpgtransferparamget_clicked()
{
    unsigned char enabel = 0;
    unsigned char rate;
    int ret = ui->player_widget_1->getJPGTransferParam(&enabel,&rate);
    if(ret == 0){
        ui->comboBox_jpgtransferenable->setCurrentIndex(enabel);
        ui->lineEdit_jpgtransferrate->setText(QString::number(rate));
    }
}

void MainWindow::on_pushButton_pitch_jpgtransferparamset_clicked()
{
    unsigned char enabel = ui->comboBox_jpgtransferenable->currentIndex();
    unsigned char rate = ui->lineEdit_jpgtransferrate->text().toInt();
    int ret = ui->player_widget_1->setJPGTransferParam(enabel,rate);

}

void MainWindow::on_pushButton_jpgtransfershow_clicked()
{



    ui->player_widget_1->showJPGMode(true);
    autoQueryJPG = true;

    std::thread th([this](){


        unsigned char rate = ui->lineEdit_jpgtransferrate->text().toInt();
        int delay = rate <= 0 ? 0 : 1000 / rate;

        if(delay == 0)return ;

        while (autoQueryJPG) {
            ui->player_widget_1->NET_DVR_CaptureJPEGPicture_WithAppendData();
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }

    });
    th.detach();


}

void MainWindow::on_pushButton_jpgtransfernotshow_clicked()
{
    autoQueryJPG = false;
   ui->player_widget_1->showJPGMode(false);
}
