#include "playerwidget.h"
#include <QGridLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QStyleOption>
#include "analysisgraphicswidget.h"
#include "irnetsdkmanager.h"
#include <QMessageBox>
#include <QMouseEvent>

int rect_Xcenter = 350;  
int rect_Ycenter = 180;
int rect_bias = 80;

int rect_Xcenter2 = 100;  
int rect_Ycenter2 = 300;
int rect_bias2 = 60;


PlayerWidget::PlayerWidget(QWidget *parent) :
    QWidget(parent)
{
    m_pRGBBug = new unsigned char[1280 * 720 *3];
    // a = 1;

    srcdata.create(576,768,CV_8UC3);
    // dstdata.create(576,768,CV_8UC3);

    m_FrameTemps = new unsigned short[1280 * 960];
    maxPoint = new IRNet::IRPointInfo();
    minPoint = new IRNet::IRPointInfo();

    maxPoint2 = new IRNet::IRPointInfo();
    minPoint2 = new IRNet::IRPointInfo();
    initWidgets();
    ShowTip("Waitting");

    connect(IRNetSDKManager::GetInstance(),SIGNAL(sgnVideoData(uchar*,int,int,int,int)),this,SLOT(OnVideoShow(uchar*,int,int,int,int)));
    connect(IRNetSDKManager::GetInstance(),SIGNAL(sgnConnectStatusChange(int,int)),this,SLOT(OnConnectStatusChange(int,int)));
    connect(this,SIGNAL(sgnJPGShow(QByteArray,unsigned short*,int,int)),this,SLOT(OnJPGShow(QByteArray,unsigned short*,int,int)));

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(OnTimeout()));


}

PlayerWidget::~PlayerWidget()
{
     delete minPoint;
     delete maxPoint;
     delete minPoint2;
     delete maxPoint2;
    
    srcdata.release();
    delete[] m_FrameTemps;
    delete[] m_pRGBBug;

    StopTimer();
    std::cout<<"析构了！！！"<<std::endl;
    delete timer;
    delete player_label;
    delete analysis_graphicsWidget;
    delete root_gridLayout;
}

void PlayerWidget::paintEvent(QPaintEvent *event){
    QStyleOption opt;
    opt.initFrom(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget,&opt,&p,this);

}

void PlayerWidget::mousePressEvent(QMouseEvent *event)
{


    int x = event->x();
    int y = event->y();

    double scale_w_view = 1;
    double scale_h_view = 1;

    double w = this->width();
    double h = this->height();

    int data_width = m_IRRealWidth;     //m_ir_width
    int data_height = m_IRRealHeight;   //m_ir_height


    if(data_width > 0){
        scale_w_view = data_width / w;
    }

    if(data_height > 0){
        scale_h_view =  data_height / h;
    }

    int pos_x = x * scale_w_view;
    int pos_y = y * scale_h_view;

    float temp = 0;
    {

        QMutexLocker locker(&mtx);
        unsigned short *temps = m_FrameTemps;

        int offset = pos_y * data_width + pos_x;

        QString msg = QString("mousePressEvent pos_y=%1 pos_x=%2 data_width=%3").arg(pos_y).arg(pos_x).arg(data_width);

        qDebug()<<msg;

        qDebug()<<"mousePressEvent offset "<<offset;


        temp = temps[offset] / 10.0;
    }

    emit sgnPointTempInfo(pos_x,pos_y,temp);
}

void PlayerWidget::mouseReleaseEvent(QMouseEvent *event)
{

}

void PlayerWidget::mouseDoubleClickEvent(QMouseEvent *event)
{

}

void PlayerWidget::mouseMoveEvent(QMouseEvent *event)
{

}



void PlayerWidget::AddPointAnalysis(QList<QPoint> points){
    analysis_number++;
    AddAnalysis(IRNet::AnalysisType::PointAnalysis,points,analysis_number);
}

void PlayerWidget::AddLineAnalysis(QList<QPoint> points){
    analysis_number++;
    AddAnalysis(IRNet::AnalysisType::LineAnalysis,points,analysis_number);
}

void PlayerWidget::AddRectAnalysis(QList<QPoint> points){
    analysis_number++;
    AddAnalysis(IRNet::AnalysisType::RectAnalysis,points,analysis_number);
}

void PlayerWidget::AddCireAnalysis(QList<QPoint> points){
    analysis_number++;
    AddAnalysis(IRNet::AnalysisType::CireAnalysis,points,analysis_number);
}

void PlayerWidget::AddPolyAnalysis(QList<QPoint> points)
{
    analysis_number++;
    AddAnalysis(IRNet::AnalysisType::PolyAnalysis,points,analysis_number);
}

void PlayerWidget::AddAnalysis(int type,QList<QPoint> points,int tag){
    if(type == IRNet::AnalysisType::PointAnalysis){
        analysis_graphicsWidget->AddPointAnalysis(points,tag);
    }else if (type == IRNet::AnalysisType::LineAnalysis) {
        analysis_graphicsWidget->AddLineAnalysis(points,tag);
    }else if (type == IRNet::AnalysisType::RectAnalysis) {
        analysis_graphicsWidget->AddRectAnalysis(points,tag);
    }else if (type == IRNet::AnalysisType::CireAnalysis) {
        analysis_graphicsWidget->AddCireAnalysis(points,tag);
    }else if (type == IRNet::AnalysisType::PolyAnalysis) {
        analysis_graphicsWidget->AddPolyAnalysis(points,tag);
    }

}

void PlayerWidget::ClearAnalysis(){
    analysis_number = 0;
    analysis_graphicsWidget->ClearAnalysis();
}

void PlayerWidget::SetFollowMaxTemp(bool is_follow){
    is_follow_max = is_follow;
    analysis_graphicsWidget->SetFollowMaxTemp(is_follow);
}

void PlayerWidget::SetFollowMinTemp(bool is_follow){
    is_follow_min = is_follow;
    analysis_graphicsWidget->SetFollowMinTemp(is_follow);
}

bool PlayerWidget::IsFollowMaxTemp(){
    return is_follow_max;
}

bool PlayerWidget::IsFollowMinTemp(){
    return is_follow_min;
}

void PlayerWidget::ShowMachineVideo(bool is_show){
    is_show_machine_video = is_show;
}


void PlayerWidget::SetDeviceId(long device_id){
    m_device_id = device_id;
}


void PlayerWidget::Connect(QString ip,int port,int timeout){
    //IRNetSDKManager::GetInstance()->Connect(ip,port,m_device_id,0,timeout);
    Connect(ip,port,0,timeout);
}

void PlayerWidget::Connect(QString ip,int port,int cameraModel,int timeout){
    IRNetSDKManager::GetInstance()->Connect(ip,port,m_device_id,cameraModel,timeout);
}

QString PlayerWidget::GetMachineSN()
{
    return IRNetSDKManager::GetInstance()->GetMachineSN(m_device_id);
}

long PlayerWidget::disConnectNet(){

    return IRNetSDKManager::GetInstance()->disConnectNet(m_device_id);
}


void PlayerWidget::RealPlay(){
    IRNetSDKManager::GetInstance()->RealPlay(m_device_id);
}

void PlayerWidget::stopPlay()
{
    IRNetSDKManager::GetInstance()->stopPlay(m_device_id);
}

void PlayerWidget::StartTemperatureMeasurement(){
    IRNetSDKManager::GetInstance()->StartTemperatureMeasurement(m_device_id);
}

void PlayerWidget::StopTemperatureMeasurement(){
    IRNetSDKManager::GetInstance()->StopTemperatureMeasurement(m_device_id);
}

void PlayerWidget::SnapJpegTemperature(char* filename){

    long filetype = 0;

    if(is_show_machine_video){
        filetype = 2;
    }


    IRAnalysis analysis_list[100];
    int len = 0;

    GetAnalysis(analysis_list,len);

    IRNetSDKManager::GetInstance()->SnapJpegTemperature(m_device_id,filename,filetype,analysis_list,len);
}

int PlayerWidget::Record264video(char* FileName, long RecordTime){
    return IRNetSDKManager::GetInstance()->Record264video(m_device_id,FileName,RecordTime);
}

void PlayerWidget::StopRecord264Video(){
    IRNetSDKManager::GetInstance()->StopRecord264Video(m_device_id);
}

void PlayerWidget::SetPalette(long paletteindex){
    IRNetSDKManager::GetInstance()->SetPalette(m_device_id,paletteindex);
}

long PlayerWidget::GetPalette(){
    return IRNetSDKManager::GetInstance()->GetPalette(m_device_id);
}

void PlayerWidget::SetEmissivity(double useemiss){
    IRNetSDKManager::GetInstance()->SetEmissivity(m_device_id,useemiss);
}

double PlayerWidget::GetEmissivity(){
    return IRNetSDKManager::GetInstance()->GetEmissivity(m_device_id);
}

void PlayerWidget::SetDistance(double usedist){
    IRNetSDKManager::GetInstance()->SetDistance(m_device_id,usedist);
}

double PlayerWidget::GetDistance(){
    return IRNetSDKManager::GetInstance()->GetDistance(m_device_id);
}

void PlayerWidget::SetTransmit(double use_transmit){
    IRNetSDKManager::GetInstance()->SetTransmit(m_device_id,use_transmit);
}

double PlayerWidget::GetTransmit(){
    return IRNetSDKManager::GetInstance()->GetTransmit(m_device_id);
}

void PlayerWidget::SetEnvironmentTemperature(double use_env_tem){
    IRNetSDKManager::GetInstance()->SetEnvironmentTemperature(m_device_id,use_env_tem);
}

double PlayerWidget::GetEnvironmentTemperature(){
    return IRNetSDKManager::GetInstance()->GetEnvironmentTemperature(m_device_id);
}

void PlayerWidget::SetRelativeHumidity(double use_humidity){
    IRNetSDKManager::GetInstance()->SetRelativeHumidity(m_device_id,use_humidity);
}

double PlayerWidget::GetRelativeHumidity(){
    return IRNetSDKManager::GetInstance()->GetRelativeHumidity(m_device_id);
}


void PlayerWidget::SetImageEnhancement(long is_image_enhancement){
    IRNetSDKManager::GetInstance()->SetImageEnhancement(m_device_id,is_image_enhancement);
}

void PlayerWidget::Disconnect(){
    IRNetSDKManager::GetInstance()->Disconnect(m_device_id);
}


void PlayerWidget::SaveMachintAnalysis(){


    IRAnalysis analysis[50];
    int len = 0;
    GetAnalysis(analysis,len);
    int sum = 0;
    for(int i=0;i<len;i++){
        IRAnalysis *item = &analysis[i];
        if(item->type == PolyAnalysis || item->type == LineAnalysis || item->type == RectAnalysis || item->type == CireAnalysis){
            sum++;
        }
    }

    if(sum > 5){
        QMessageBox::information(NULL, "提示", "分析框不能大于5个", QMessageBox::Yes);
        return;
    }

    ClearMachineAnalysis();

    IRNetSDKManager::GetInstance()->SaveIrMachineAnalysis(m_device_id,analysis,len);


}

void PlayerWidget::GetMachineAnalysis(){


    double scale_w_view =  (double)this->width() / m_ir_width;
    double scale_h_view =  (double)this->height() /m_ir_height;


    ClearAnalysis();

    IRAnalysis analysis[50];
    int len = 0;
    IRNetSDKManager::GetInstance()->GetMachineAnalysis(m_device_id,analysis,&len);
    int max_id = 0;
    for(int i=0;i<len;i++){

        IRAnalysis item = analysis[i];
        item.points[0].x *= scale_w_view;
        item.points[0].y *= scale_h_view;
        item.points[1].x *= scale_w_view;
        item.points[1].y *= scale_h_view;

        max_id = item.number > max_id ? item.number:max_id;
        QList<QPoint> points;
        if (item.type == PointAnalysis){
            points.append(QPoint(item.points[0].x,item.points[0].y));
        }else if (item.type == CireAnalysis) {
            int radius = (item.points[1].x - item.points[0].x) / 2;
            points.append(QPoint(item.points[0].x,item.points[0].y + radius));
            points.append(QPoint(item.points[1].x,item.points[1].y - radius));
        }else if(item.type == PolyAnalysis){
            points.append(QPoint(item.points[0].x,item.points[0].y));
            points.append(QPoint(item.points[1].x,item.points[1].y));
            for(int i=2;i<item.polySize;++i){
                points.append(QPoint(item.points[i].x * scale_w_view,item.points[i].y * scale_h_view));
            }
        }
        else {
            points.append(QPoint(item.points[0].x,item.points[0].y));
            points.append(QPoint(item.points[1].x,item.points[1].y));
        }

        AddAnalysis(item.type,points,item.number);
    }

    analysis_number = max_id;
}

void PlayerWidget::ClearMachineAnalysis(){
    IRNetSDKManager::GetInstance()->ClearMachineAnalysis(m_device_id);
}


void PlayerWidget::SetMachineColorRegion(int color_index, int x1, int y1, int x2, int y2){
    IRNetSDKManager::GetInstance()->SetMachineColorRegion(m_device_id,color_index,x1,y1,x2,y2);
}

void PlayerWidget::CancelMachineColorRegion(){
    IRNetSDKManager::GetInstance()->CancelMachineColorRegion(m_device_id);
}


void PlayerWidget::SetTransmissionFrameRate(int frame_rate){
    IRNetSDKManager::GetInstance()->SetTransmissionFrameRate(m_device_id,frame_rate);
}

void PlayerWidget::DoAdjust(){
    IRNetSDKManager::GetInstance()->DoAdjust(m_device_id);
}


long PlayerWidget::GetAllTemp(float* tempbuf, long Len){
    return IRNetSDKManager::GetInstance()->GetAllTemp(m_device_id,tempbuf,Len);
}

bool PlayerWidget::getMaxAndMinTempPoint(IRPointInfo *maxPointInfo, IRPointInfo *minPointInfo, double *avgTemp_){
    return IRNetSDKManager::GetInstance()->getMaxAndMinTempPoint(m_device_id, maxPointInfo, minPointInfo, avgTemp_);
}

long PlayerWidget::GetIrDataLen(long* IrWidth, long* IrHeight){
    return IRNetSDKManager::GetInstance()->GetIrDataLen(m_device_id,IrWidth,IrHeight);
}

void PlayerWidget::SearchCameraIP(long search_time){
    IRNetSDKManager::GetInstance()->SearchCameraIP(search_time);
}

int PlayerWidget::getH264EncodecParam(){
    return IRNetSDKManager::GetInstance()->getH264EncodecParam(m_device_id);
}

void PlayerWidget::PlayerWidget::setH264EncodecParam(int value){
    IRNetSDKManager::GetInstance()->setH264EncodecParam(m_device_id,value);
}

int PlayerWidget::rebootMachine()
{
    return  IRNetSDKManager::GetInstance()->rebootMachine(m_device_id);
}

int PlayerWidget::shopPhotoOnMachine()
{
    return  IRNetSDKManager::GetInstance()->shopPhotoOnMachine(m_device_id);
}

int PlayerWidget::getMaxAndMinTempFromMachine()
{
    IRNetSDKManager::GetInstance()->getMaxAndMinTempFromMachine(m_device_id);
    return 0;
}

int PlayerWidget::getAnalysisFromMachine()
{
    IRNetSDKManager::GetInstance()->getAnalysisFromMachine(m_device_id);
    return 0;
}

void PlayerWidget::setRTCTime()
{
    IRNetSDKManager::GetInstance()->setRTCTime(m_device_id);
}

int PlayerWidget::startRecordOnMachine()
{
    return IRNetSDKManager::GetInstance()->startRecordOnMachine(m_device_id);
}

int PlayerWidget::stopRecordOnMachine()
{
    return IRNetSDKManager::GetInstance()->stopRecordOnMachine(m_device_id);
}

int PlayerWidget::setVideoFormatOfMachine(int format)
{
    return IRNetSDKManager::GetInstance()->setVideoFormatOfMachine(m_device_id,format);
}

int PlayerWidget::getSDAvailableSpaceOfMachine()
{
    return IRNetSDKManager::GetInstance()->getSDAvailableSpaceOfMachine(m_device_id);
}

IRRange PlayerWidget::getRange()
{
    return IRNetSDKManager::GetInstance()->getRange(m_device_id);
}

void PlayerWidget::setRange(IRRange range)
{
    IRNetSDKManager::GetInstance()->setRange(m_device_id,range);
}

long PlayerWidget::getAllAD(unsigned short *buf, long Len)
{
    return IRNetSDKManager::GetInstance()->getAllAD(m_device_id,buf,Len);
}

int PlayerWidget::focusAuto()
{
    return IRNetSDKManager::GetInstance()->focusAuto(m_device_id);
}

int PlayerWidget::focusOut()
{
    return IRNetSDKManager::GetInstance()->focusOut(m_device_id);
}

int PlayerWidget::focusIn()
{
    return IRNetSDKManager::GetInstance()->focusIn(m_device_id);
}

int PlayerWidget::focusModen(int moden)
{
    return IRNetSDKManager::GetInstance()->focusModen((IRFocusModen)moden,m_device_id);
}

void PlayerWidget::getADExtremum(unsigned short *max, unsigned short *min)
{
    IRNetSDKManager::GetInstance()->getADExtremum(m_device_id,max,min);
}

void PlayerWidget::getLevelSpanInfo(unsigned short *max, unsigned short *min, unsigned short *level, unsigned short *span)
{
    IRNetSDKManager::GetInstance()->getLevelSpanInfo(m_device_id,max,min,level,span);
}

int PlayerWidget::setEffectiveArea(bool enable, int left, int top, int width, int height)
{
    return IRNetSDKManager::GetInstance()->setEffectiveArea(m_device_id,enable,left,top,width,height);
}

void PlayerWidget::setLevelSpanModen(bool autoModen)
{
    IRNetSDKManager::GetInstance()->setLevelSpanModen(m_device_id,autoModen);

}

void PlayerWidget::setLevelSpan(unsigned short level, unsigned short span)
{
    IRNetSDKManager::GetInstance()->setLevelSpan(m_device_id,level,span);
}

float PlayerWidget::getTempFromRaw(unsigned short raw, float emissivity, float distance)
{
    return IRNetSDKManager::GetInstance()->getTempFromRaw(m_device_id,raw,emissivity,distance);
}

int PlayerWidget::setSerialPortWorkMode(unsigned char mode)
{
    return IRNetSDKManager::GetInstance()->setSerialPortWorkMode(m_device_id,mode);
}

int PlayerWidget::getSerialPortWorkMode()
{
    return IRNetSDKManager::GetInstance()->getSerialPortWorkMode(m_device_id);
}

int PlayerWidget::penetrateOnRS485(unsigned char *buf, int bufSize)
{
    return IRNetSDKManager::GetInstance()->penetrateOnRS485(m_device_id,buf,bufSize);
}

int PlayerWidget::setRtpOverMode(bool isTcp)
{
    return IRNetSDKManager::GetInstance()->setRtpOverMode(m_device_id,isTcp);
}

int PlayerWidget::getADTransferModen()
{
    return IRNetSDKManager::GetInstance()->getADTransferModen(m_device_id);
}

int PlayerWidget::setADTransferModen(int moden)
{
    return IRNetSDKManager::GetInstance()->setADTransferModen(m_device_id,moden);
}

int PlayerWidget::ptzControl(unsigned char num,unsigned char mode, unsigned char speed)
{
    return IRNetSDKManager::GetInstance()->ptzControl(m_device_id,num,mode,speed);
}

int PlayerWidget::ptzControlLeft(unsigned char num, unsigned char speed)
{
    return IRNetSDKManager::GetInstance()->ptzControlLeft(m_device_id,num,speed);
}

int PlayerWidget::ptzControlTop(unsigned char num, unsigned char speed)
{
    return IRNetSDKManager::GetInstance()->ptzControlTop(m_device_id,num,speed);
}

int PlayerWidget::ptzControlRight(unsigned char num, unsigned char speed)
{
    return IRNetSDKManager::GetInstance()->ptzControlRight(m_device_id,num,speed);
}

int PlayerWidget::ptzControlDown(unsigned char num, unsigned char speed)
{
    return IRNetSDKManager::GetInstance()->ptzControlDown(m_device_id,num,speed);
}

int PlayerWidget::ptzControlStop(unsigned char num)
{
    return IRNetSDKManager::GetInstance()->ptzControlStop(m_device_id,num);
}

float PlayerWidget::getPTZPitchAngle(unsigned char num)
{
    return IRNetSDKManager::GetInstance()->getPTZPitchAngle(m_device_id,num);
}

float PlayerWidget::getPTZRollAngle(unsigned char num)
{
    return IRNetSDKManager::GetInstance()->getPTZRollAngle(m_device_id,num);
}

int PlayerWidget::setPTZPreset(unsigned char num, unsigned char presetNum)
{
    return IRNetSDKManager::GetInstance()->setPTZPreset(m_device_id,num,presetNum);
}

int PlayerWidget::executePTZPreset(unsigned char num, unsigned char presetNum)
{
    return IRNetSDKManager::GetInstance()->executePTZPreset(m_device_id,num,presetNum);
}

int PlayerWidget::delPTZPreset(unsigned char num, unsigned char presetNum)
{
    return IRNetSDKManager::GetInstance()->delPTZPreset(m_device_id,num,presetNum);
}

int PlayerWidget::executeHorVerPTZControl(unsigned char num, int moden, unsigned char hs, unsigned char vs)
{
    return IRNetSDKManager::GetInstance()->executeHorVerPTZControl(m_device_id,num,(IRNet::PTZControl)moden,hs,vs);
}

void PlayerWidget::showJPGMode(bool show)
{
    isShowJpg = show;



}

int PlayerWidget::setJPGTransferParam(unsigned char enable, unsigned char rate)
{
    return IRNetSDKManager::GetInstance()->setJPGTransferParam(m_device_id,enable,rate);
}

int PlayerWidget::getJPGTransferParam(unsigned char *enable, unsigned char *rate)
{
    return IRNetSDKManager::GetInstance()->getJPGTransferParam(m_device_id,enable,rate);
}

int PlayerWidget::setJPGTransferMode(unsigned char useTcp)
{
    return IRNetSDKManager::GetInstance()->setJPGTransferMode(m_device_id,useTcp);
}

int PlayerWidget::getJPGTransferMode(unsigned char *useTcp)
{
    return IRNetSDKManager::GetInstance()->getJPGTransferMode(m_device_id,useTcp);
}

bool PlayerWidget::NET_DVR_CaptureJPEGPicture_WithAppendData()
{
    NET_DVR_JPEGPICTURE_WITH_APPENDDATA data;

    data.pJpegPicBuff = new char[2 * 1024 * 1024];
    data.pP2PDataBuff = new char[2 * 1024 * 1024];

    bool ret = IRNetSDKManager::GetInstance()->NET_DVR_CaptureJPEGPicture_WithAppendData(0,m_device_id,&data);

    if(ret > 0){
        m_JPGData = QByteArray(data.pJpegPicBuff,data.dwJpegPicLen);
        {

            QMutexLocker locker(&mtx);
            //m_FrameTemps = QByteArray(data.pP2PDataBuff,data.dwP2PDataLen);
            memcpy(m_FrameTemps,data.pP2PDataBuff,data.dwP2PDataLen);
        }

        emit sgnJPGShow(m_JPGData,m_FrameTemps,data.dwJpegPicWidth,data.dwJpegPicHeight);

    }



    delete[]  data.pJpegPicBuff;
    delete[] data.pP2PDataBuff;

    return ret;
}



void PlayerWidget::initWidgets(){

    root_gridLayout = new QGridLayout(this);
    root_gridLayout->setRowStretch(0, 100);//设置行比例系数
    root_gridLayout->setColumnStretch(0, 100);//设置列比例系数
    root_gridLayout->setHorizontalSpacing(0);
    root_gridLayout->setVerticalSpacing(0);
    root_gridLayout->setSpacing(0);
    root_gridLayout->setColumnStretch(0,0);
    root_gridLayout->setRowStretch(0,0);
    root_gridLayout->setMargin(0);

    setLayout(root_gridLayout);

    player_label = new QLabel(this);
    player_label->setStyleSheet("QLabel{background-color:rgb(0,0,0);Color:rgb(255,255,255);}");
    player_label->setAlignment(Qt::AlignCenter);
    player_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);//设置组件大小可扩展

    analysis_graphicsWidget = new AnalysisGraphicsWidget();
    //analysis_graphicsWidget->setStyleSheet("{background-color:rgb(0,255,0);Color:rgb(255,255,255);}");

    root_gridLayout->addWidget(player_label,0,0);
    root_gridLayout->addWidget(analysis_graphicsWidget,0,0);

    player_label->setScaledContents(true);
    
}

void PlayerWidget::ShowTip(QString text){
    player_label->setText(text);
}

void PlayerWidget::StartTimer(){
    if(timer && !timer->isActive()){
        timer->start(300);
    }
}

void PlayerWidget::StopTimer(){
    if(timer && timer->isActive()){
        timer->stop();
    }
}


void PlayerWidget::OnVideoShow(uchar* rgbData, int w, int h, int macIndex,int type){    
    if(m_device_id == macIndex){
        memcpy(m_pRGBBug,rgbData,w*h*3);
        srcdata.data=rgbData;
        //  printf("getting in OnVideoShow!!!\n");

        // double *avgTemp_ = new double[1920*1080];
        // getMaxAndMinTempPoint(maxPoint, minPoint, avgTemp_);
        // printf("maxpoint=%d,%d,%.2f\n",maxPoint->x,maxPoint->y,maxPoint->temp_);
        // printf("minPoint=%d,%d,%.2f\n",minPoint->x,minPoint->y,minPoint->temp_);
        //获取左上角（50,50）右下角（100,100）的矩形范围内的温度信息
        
        // rect_analysis.points[0].x = rect_Xcenter - shift_from_center;
        // rect_analysis.points[0].y = rect_Ycenter - shift_from_center;
        // rect_analysis.points[1].x = rect_Xcenter + shift_from_center;
        // rect_analysis.points[1].y = rect_Ycenter + shift_from_center;
        
        IRNet::IRAnalysis rect_analysis;
        rect_analysis.points[0].x = rect_Xcenter - rect_bias ;
        rect_analysis.points[0].y = rect_Ycenter - rect_bias ;
        rect_analysis.points[1].x = rect_Xcenter + rect_bias ;
        rect_analysis.points[1].y = rect_Ycenter + rect_bias ;
        rect_analysis.type = IRNet::AnalysisType::RectAnalysis;
        rect_analysis.number = 1;

        IRNet::IRAnalysis rect_analysis2;
        rect_analysis2.points[0].x = rect_Xcenter2 - rect_bias2 ;
        rect_analysis2.points[0].y = rect_Ycenter2 - rect_bias2 ;
        rect_analysis2.points[1].x = rect_Xcenter2 + rect_bias2 ;
        rect_analysis2.points[1].y = rect_Ycenter2 + rect_bias2 ;
        rect_analysis2.type = IRNet::AnalysisType::RectAnalysis;
        rect_analysis2.number = 1;

        // //获取最高温的温度信息
        // IRNet::IRAnalysis max_analysis;
        // max_analysis.type = IRNet::AnalysisType::MaxAnalysis;
        // max_analysis.number = 2;

        // //获取最低温的温度信息
        // IRNet::IRAnalysis min_analysis;
        // min_analysis.type = IRNet::AnalysisType::MinAnalysis;
        // min_analysis.number = 3;

        // IRNet::IRAnalysis analysisList;
        // analysisList = rect_analysis;
        // int resultOfAnalysis = IRNet::getAnalysis(m_device_id,&rect_analysis,1);
        // std::cout<<"the type of analysis: "<<type_of_analysis<<std::endl;
        // if(type_of_analysis==1){
        //     IRNet::getAnalysis(m_device_id,&rect_analysis,1);
        // }
        // else if(type_of_analysis == 3){
            // IRNet::getAnalysis(m_device_id,analysisList,3);
        // }

        IRNet::getAnalysis(m_device_id,&rect_analysis,1);
        IRNet::getAnalysis(m_device_id,&rect_analysis2,1);
        //IRNet::IRAnalysis* item = &rect_analysis;
        //IRNet::IRAnalysis* item2 = &rect_analysis2;
        *maxPoint  = rect_analysis.maxPointInfo;
        *minPoint  = rect_analysis.minPointInfo;
        avgPoint   = rect_analysis.avgTemp_;

        *maxPoint2  = rect_analysis2.maxPointInfo;
        *minPoint2  = rect_analysis2.minPointInfo;
        avgPoint2   = rect_analysis2.avgTemp_;
        //minPoint=&item->minPointInfo;
        // printf("avg temp:%.2f\n",item->avgTemp_);
        // minPoint->temp_ = item->avgTemp_;
        // minPoint->temp_ = item->avgTemp_;
        // printf("analysis tag:%d type=%d\n", item->number,item->type);
        // printf("max info temp:%.2f,x=%d,y=%d\n",maxPoint->temp_,maxPoint->x,maxPoint->y);
        // printf("min info temp:%.2f,x=%d,y=%d\n",minPoint->temp_ ,minPoint->x,minPoint->y);
        // printf("avg temp:%.2f\n",item->avgTemp_);

        // cv::imshow("src",srcdata);			
        QImage img =  QImage(m_pRGBBug,w,h,QImage::Format_RGB888);
        QPixmap pix =  QPixmap::fromImage(img);
        int with = player_label->width();
        int height = player_label->height();
        //QPixmap fitpixmap = pix.scaled(with, height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

        if(is_show_machine_video && type == 1){
            m_video_width = w;
            m_video_height = h;
            player_label->setPixmap(pix);
        }else if(!is_show_machine_video && type == 0){
            m_video_width = w;
            m_video_height = h;
            player_label->setPixmap(pix);
        }
    }

}

void PlayerWidget::OnJPGShow(QByteArray jpg, unsigned short *temps, int w, int h)
{

    m_video_width = w;
    m_video_height = h;

    QImage image;
    image.loadFromData((uchar*)jpg.data(),jpg.size(),"jpg");
    QPixmap pix = QPixmap::fromImage(image);
    player_label->setPixmap(pix);

}

void PlayerWidget::OnConnectStatusChange(int status,int deviceId){

    if(m_device_id == deviceId){

        if(status == 0){
            ShowTip("");

            IRNetSDKManager::GetInstance()->GetDataSize(m_device_id,&m_ir_width,&m_ir_height);

            IRNetSDKManager::GetInstance()->GetIrDataLen(m_device_id,&m_IRRealWidth,&m_IRRealHeight);


            qDebug()<<QString("GetDataSize w = %1 h = %2").arg(m_ir_width).arg(m_ir_height);

            StartTimer();

        }else if(status == 4){
            StopTimer();
        }
        else{
            ShowTip(QString("status %1").arg(status));
        }

    }
}

void PlayerWidget::GetAnalysis(IRNet::IRAnalysis *analysis_list,int &len){


    QList<AnalysisInfo> list;
    analysis_graphicsWidget->GetAnalysisInfo(list);


    int i = 0;

    double scale_w_view = 1;
    double scale_h_view = 1;

    double w = this->width();
    double h = this->height();

    int data_width = m_ir_width;     //m_ir_width
    int data_height = m_ir_height;   //m_ir_height


    if(data_width > 0){
        scale_w_view = data_width / w;
    }

    if(data_height > 0){
        scale_h_view =  data_height / h;
    }


    foreach (AnalysisInfo item , list) {

        IRNet::IRAnalysis* analysis = &analysis_list[i];

        if(item.analysisType == Rect){

            QList<QPoint> list = item.points;
            for(int i=0;i<item.points.size();i++){
                analysis->points[i].x = list[i].x() * scale_w_view;
                analysis->points[i].y = list[i].y() * scale_h_view;
                //qDebug()<<QString("pos i = %0 x = %1 y = %2 scale = %3").arg(i).arg(list[i].x()).arg(list[i].y()).arg(scale_w_view);
            }
            analysis->type = IRNet::AnalysisType::RectAnalysis;
            analysis->number = item.tag;

        }else if(item.analysisType == Point){
            QList<QPoint> list = item.points;
            for(int i=0;i<item.points.size();i++){
                analysis->points[i].x = list[i].x() * scale_w_view;
                analysis->points[i].y = list[i].y() * scale_h_view;
            }
            analysis->type = IRNet::AnalysisType::PointAnalysis;
            analysis->number = item.tag;
        }else if(item.analysisType == Line){
            QList<QPoint> list = item.points;
            for(int i=0;i<item.points.size();i++){
                analysis->points[i].x = list[i].x() * scale_w_view;
                analysis->points[i].y = list[i].y() * scale_h_view;
            }
            analysis->type = IRNet::AnalysisType::LineAnalysis;
            analysis->number = item.tag;
        }else if(item.analysisType == Cire){
            QList<QPoint> list = item.points;
            for(int i=0;i<item.points.size();i++){
                analysis->points[i].x = list[i].x() * scale_w_view;
                analysis->points[i].y = list[i].y() * scale_h_view;
            }
            analysis->type = IRNet::AnalysisType::CireAnalysis;
            analysis->number = item.tag;
        }else if(item.analysisType == Poly){
            QList<QPoint> list = item.points;
            for(int i=0;i<item.points.size();i++){
                analysis->points[i].x = list[i].x() * scale_w_view;
                analysis->points[i].y = list[i].y() * scale_h_view;
            }
            analysis->polySize = list.size();
            analysis->type = IRNet::AnalysisType::PolyAnalysis;
            analysis->number = item.tag;
        }
        else if(item.analysisType == Max){
            analysis->type = IRNet::AnalysisType::MaxAnalysis;
            analysis->number = item.tag;
        }else if(item.analysisType == Min){
            analysis->type = IRNet::AnalysisType::MinAnalysis;
            analysis->number = item.tag;
        }

        i++;
    }


    len = i;

}

void PlayerWidget::OnTimeout(){



    double scale_w_view = m_ir_width / (double)this->width();
    double scale_h_view = m_ir_height / (double)this->height();


    IRNet::IRAnalysis analysisList[100];
    int len = 0;
    GetAnalysis(analysisList,len);

    IRNetSDKManager::GetInstance()->getAnalysis(m_device_id,analysisList,len);

    QList<TempInfo> temp_infos;

    for(int i=0;i<len;i++){
        IRAnalysis* item = &analysisList[i];
        TempInfo info;
        info.max_temp = item->maxPointInfo.temp_;
        info.max_point = QPoint(item->maxPointInfo.x / scale_w_view,item->maxPointInfo.y / scale_h_view);
        info.min_temp = item->minPointInfo.temp_;
        info.min_point = QPoint(item->minPointInfo.x / scale_w_view,item->minPointInfo.y / scale_h_view);
        info.avg_temp = item->avgTemp_;
        info.tag = item->number;
        temp_infos.append(info);
        //qDebug()<<QString("max x=%1  y=%2").arg(item->maxPointInfo.x).arg(item->maxPointInfo.y);
    }

    analysis_graphicsWidget->UpdateTempInfo(temp_infos);

}
































