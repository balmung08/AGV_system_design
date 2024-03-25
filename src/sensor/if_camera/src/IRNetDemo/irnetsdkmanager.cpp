#include "irnetsdkmanager.h"
#include <QMessageBox>
#include <QDebug>
#include <QThread>

static IRNetSDKManager * m_pInstance;

#if defined (_WIN32) || defined (WIN32)
#else
    #define __stdcall
#endif


void __stdcall OnVideoCallBack(const char* rgbData,const unsigned short *adBuf,int adBufSize, int w, int h, int device_id) {

   // QThread::currentThread()->sleep(1);

   emit IRNetSDKManager::GetInstance()->sgnVideoData((uchar*)rgbData,w,h,device_id,0);
}

void __stdcall OnRTSPvideoCallBack(const char* rgbData, int w, int h, int device_id) {
    emit IRNetSDKManager::GetInstance()->sgnVideoData((uchar*)rgbData,w,h,device_id,1);
}

void __stdcall OnConnectStatusChangeCallBack(int status,int device_id){
    emit IRNetSDKManager::GetInstance()->sgnConnectStatusChange(status,device_id);
}

void __stdcall OnSearchCameraIP(const char *data,int len){

    if(len > 2){

        if((unsigned char)data[0] == 0xEB && (unsigned char)data[1] == 0x90){

            QString hex = IRNetSDKManager::GetInstance()->byteArrayToHexStr(data);
            emit IRNetSDKManager::GetInstance()->sgnCameraLogInfo(hex);

        }else{
            QString ipStr = QByteArray(data,len);
            qDebug()<<"OnSearchCameraIP :"<<ipStr;
            emit IRNetSDKManager::GetInstance()->sgnSearchCameraIP(ipStr);
        }

    }

}

void __stdcall OnRS485ReceiveCallBack(const char *buf,int len){

    QByteArray data(buf,len);
    QString hex = IRNetSDKManager::GetInstance()->byteArrayToHexStr(data);
    emit IRNetSDKManager::GetInstance()->sgnRS485Receive(hex);

}

IRNetSDKManager* IRNetSDKManager::GetInstance()
{
     if (m_pInstance == nullptr )
         m_pInstance = new IRNetSDKManager();
     return m_pInstance;
}


IRNetSDKManager::IRNetSDKManager()
{

}

void IRNetSDKManager::Destroy(){


    IRNet::setConnectStatusCallBack(nullptr);
    IRNet::setVideoDataCallBack(nullptr);
    IRNet::setMachineVideoDataCallBack(nullptr);
    IRNet::setSearchCameraIPCallBack(nullptr);
    IRNet::setRS485ReceiveCallBack(nullptr);
    IRNet::destroy();
    delete this;

}

void IRNetSDKManager::Init(){

    IRNet::init();
    IRNet::setVideoDataCallBack((IRNet::VideoDataCallBack)OnVideoCallBack);

    IRNet::setMachineVideoDataCallBack((IRNet::MachineVideoDataCallBack)OnRTSPvideoCallBack);

    IRNet::setConnectStatusCallBack(IRNet::ConnectStatusCallBack(OnConnectStatusChangeCallBack));

    IRNet::setSearchCameraIPCallBack(IRNet::SearchCameraIPCallBack(OnSearchCameraIP));

    IRNet::setRS485ReceiveCallBack(IRNet::RS485ReceiveCallBack(OnRS485ReceiveCallBack));
}

QString IRNetSDKManager::GetSDKVersion(){

    char version[255];
    int len = IRNet::getSDKVersion(version);
    QString v = QString(version);
    return v;
}

 void IRNetSDKManager::ShowMachineVideo(bool is_show){

 }

void IRNetSDKManager::Connect(QString ip,int port,long device_id,int timeout){

   Connect(ip,port,device_id,0,timeout);
}


void IRNetSDKManager::Connect(QString ip,int port,long device_id,int cameraModel,int timeout){
    IRNet::connectWithModel_(ip.toUtf8().data(),port,device_id,(CameraModel)cameraModel,timeout);
    IRNet::setIRVideoFormat(device_id,IRNet::Format_RGB888);
}


long IRNetSDKManager::disConnectNet(long device_id){
    return IRNet::disConnectNet(device_id);
}

void IRNetSDKManager::destroy(){
    IRNet::destroy();
}


void IRNetSDKManager::RealPlay(long device_id){
    IRNet::realPlay(device_id);
}

void IRNetSDKManager::stopPlay(long deviceId)
{
    IRNet::stopPlay(deviceId);
}

void IRNetSDKManager::StartTemperatureMeasurement(long device_id){
    IRNet::startTemperatureMeasurement(device_id);
}

void IRNetSDKManager::StopTemperatureMeasurement(long device_id){
    IRNet::stopTemperatureMeasurement(device_id);
}

void IRNetSDKManager::SnapJpegTemperature(long device_id, char* filename, long filetype,IRAnalysis *analysis,int len){
    IRNet::snapJpegTemperature(device_id,filename,filetype,analysis,len);
}

int IRNetSDKManager::Record264video(long device_id, char* file_name, long record_time){
    return IRNet::record264video(device_id,file_name,record_time);
}

void IRNetSDKManager::StopRecord264Video(long device_id){
    IRNet::stopRecord264Video(device_id);
}

void IRNetSDKManager::SetPalette(long device_id, long paletteindex){
    IRNet::setPalette(device_id,paletteindex);
}

long IRNetSDKManager::GetPalette(long device_id){
    return IRNet::getPalette(device_id);
}

 void IRNetSDKManager::SetEmissivity(long device_id, double useemiss){
     IRNet::setEmissivity(device_id,useemiss);
 }

 double IRNetSDKManager::GetEmissivity(long device_id){
     return IRNet::getEmissivity(device_id);
 }

 void IRNetSDKManager::SetDistance(long device_id, double usedist){
     IRNet::setDistance(device_id,usedist);
 }

 double IRNetSDKManager::GetDistance(long device_id){
     return IRNet::getDistance(device_id);
 }

 void IRNetSDKManager::SetTransmit(long device_id, double use_transmit){
     IRNet::setTransmit(device_id,use_transmit);
 }

 double IRNetSDKManager::GetTransmit(long device_id){
     return IRNet::getTransmit(device_id);
 }

 void IRNetSDKManager::SetEnvironmentTemperature(long device_id, double use_env_tem){
     IRNet::setEnvironmentTemperature(device_id,use_env_tem);
 }

 double IRNetSDKManager::GetEnvironmentTemperature(long device_id){
     return IRNet::getEnvironmentTemperature(device_id);
 }

 void IRNetSDKManager::SetRelativeHumidity(long device_id, double use_humidity){
     IRNet::setRelativeHumidity(device_id,use_humidity);
 }

 double IRNetSDKManager::GetRelativeHumidity(long device_id){
     return IRNet::getRelativeHumidity(device_id);
 }

 QString IRNetSDKManager::GetMachineSN(long device_id)
 {

     char sn_buf[32] = {0};
     bool ret = IRNet::getMacSN(device_id,sn_buf);
     QString sn = QString(sn_buf);
     return sn;
 }

void IRNetSDKManager::getAnalysis(long device_id,IRAnalysis* list,int len){
    IRNet::getAnalysis(device_id,list,len);
}

 void IRNetSDKManager::Disconnect(long device_id){
     IRNet::disConnectNet(device_id);
 }

 void IRNetSDKManager::SetImageEnhancement(long device_id, long is_image_enhancement){
     IRNet::setImageEnhancement(device_id,is_image_enhancement);
 }



 void IRNetSDKManager::ClearMachineAnalysis(long device_id){

     IRNet::clearMachineAnalysis(device_id);

 }

 void IRNetSDKManager::SaveIrMachineAnalysis(long deviceId,IRAnalysis* list,int len){
     IRNet::saveIrMachineAnalysis(deviceId,list,len);
 }


 void IRNetSDKManager::GetMachineAnalysis(long deviceId,IRAnalysis* list,int* len){

     IRNet::getMachineAnalysis(deviceId,list,len);

 }


 void IRNetSDKManager::SetMachineColorRegion(long device_id,int color_index, int x1, int y1, int x2, int y2){
     IRNet::setMachineColorRegion(device_id,color_index,x1,y1,x2,y2);
 }

 void IRNetSDKManager::CancelMachineColorRegion(long device_id){
     IRNet::cancelMachineColorRegion(device_id);
 }

 void IRNetSDKManager::SetTransmissionFrameRate(long deviceId,int frame_rate){
     IRNet::setTransmissionFrameRate(deviceId,frame_rate);
 }

 void IRNetSDKManager::DoAdjust(long deviceId){
     IRNet::doAdjust(deviceId);
 }



 long IRNetSDKManager::GetAllTemp(long deviceId, float* tempbuf, long Len){
     return IRNet::getAllTemp(deviceId,tempbuf,Len);
 }

bool IRNetSDKManager::getMaxAndMinTempPoint(long deviceId, IRPointInfo *maxPointInfo, IRPointInfo *minPointInfo, double *avgTemp_)
{
    return  IRNet::getMaxAndMinTempPoint(deviceId, maxPointInfo, minPointInfo, avgTemp_);
}


 long IRNetSDKManager::GetIrDataLen(long deviceId, long* IrWidth, long* IrHeight){
     return IRNet::getIrDataLen(deviceId,IrWidth,IrHeight);
 }


 void IRNetSDKManager::GetDataSize(long deviceId, long* irwidth, long* irheight){
     IRNet::getDataSize(deviceId,irwidth,irheight);
 }


 void IRNetSDKManager::SearchCameraIP(long search_time){
     IRNet::searchCameraIP(search_time);
 }

 int IRNetSDKManager::getH264EncodecParam(long deviceId){
     return IRNet::getH264EncodecParam(deviceId);
 }

 void IRNetSDKManager::setH264EncodecParam(long deviceId, int value){
     IRNet::setH264EncodecParam(deviceId,value);
 }

 int IRNetSDKManager::rebootMachine(long deviceId){
     return IRNet::rebootMachine(deviceId);
 }

 int IRNetSDKManager::shopPhotoOnMachine(long deviceId)
 {
     return IRNet::shopPhotoOnMachine(deviceId);
 }

 int IRNetSDKManager::getMaxAndMinTempFromMachine(long deviceId)
 {
     IRAnalysis analysis[10];

     IRNet::getMaxAndMinTempFromMachine(deviceId,analysis);

     return 0;
 }

 int IRNetSDKManager::getAnalysisFromMachine(long deviceId)
 {
     IRAnalysis analysis[10];

     IRNet::getAnalysisFromMachine(deviceId,analysis);

     return 0;
 }

 void IRNetSDKManager::setRTCTime(long deviceId)
 {
     IRNet::setRTCTime(deviceId);
 }

 int IRNetSDKManager::startRecordOnMachine(long deviceId)
 {
     return IRNet::startRecordOnMachine(deviceId);
 }

 int IRNetSDKManager::stopRecordOnMachine(long deviceId)
 {
     return IRNet::stopRecordOnMachine(deviceId);
 }

 int IRNetSDKManager::setVideoFormatOfMachine(long deviceId, int format)
 {
     return IRNet::setVideoFormatOfMachine(deviceId,(IRRerodFormat)format);
 }

 int IRNetSDKManager::getSDAvailableSpaceOfMachine(long deviceId)
 {
     return IRNet::getSDAvailableSpaceOfMachine(deviceId);
 }

 IRRange IRNetSDKManager::getRange(long deviceId)
 {
     return IRNet::getRange(deviceId);
 }

 void IRNetSDKManager::setRange(long deviceId, IRRange range)
 {
     IRNet::setRange(deviceId,range);
 }

 QString IRNetSDKManager::byteArrayToHexStr(const QByteArray &data)
 {
     QString temp = "";
     QString hex = data.toHex();

     for (int i = 0; i < hex.length(); i = i + 2) {
         temp += hex.mid(i, 2) + " ";
     }

     return temp.trimmed().toUpper();
 }

 long IRNetSDKManager::getAllAD(long deviceId, unsigned short *buf, long Len)
 {
     return IRNet::getAllAD(deviceId,buf,Len);
 }

 int IRNetSDKManager::focusAuto(long deviceId)
 {
     return IRNet::focusAuto(deviceId);
 }

 int IRNetSDKManager::focusOut(long deviceId)
 {
     return IRNet::focusOut(deviceId);
 }

 int IRNetSDKManager::focusIn(long deviceId)
 {
     return IRNet::focusIn(deviceId);
 }

 int IRNetSDKManager::focusModen(IRFocusModen moden, long deviceId)
 {
     return IRNet::focusModen(deviceId,moden);
 }

 void IRNetSDKManager::getADExtremum(long deviceId, unsigned short *max, unsigned short *min)
 {
     IRNet::getADExtremum(deviceId,max,min);
 }

 void IRNetSDKManager::getLevelSpanInfo(long deviceId, unsigned short *max, unsigned short *min, unsigned short *level, unsigned short *span)
 {
     IRNet::getLevelSpanInfo(deviceId,max,min,level,span);
 }

 int IRNetSDKManager::setEffectiveArea(long deviceId, bool enable, int left, int top, int width, int height)
 {
     return  IRNet::setEffectiveArea(deviceId,enable,left,top,width,height);
 }

 void IRNetSDKManager::setLevelSpanModen(long deviceId, bool autoModen)
 {
     IRNet::setLevelSpanModen(deviceId,autoModen);
 }

 void IRNetSDKManager::setLevelSpan(long deviceId, unsigned short level, unsigned short span)
 {
     IRNet::setLevelSpan(deviceId,level,span);
 }

 float IRNetSDKManager::getTempFromRaw(long deviceId,unsigned short raw, float emissivity, float distance)
 {
     return IRNet::getTempFromRaw(deviceId,raw,emissivity,distance);
 }

 int IRNetSDKManager::setSerialPortWorkMode(long deviceId, unsigned char mode)
 {
     return IRNet::setSerialPortWorkMode(deviceId,mode);
 }

 int IRNetSDKManager::getSerialPortWorkMode(long deviceId)
 {
     return IRNet::getSerialPortWorkMode(deviceId);
 }

 int IRNetSDKManager::penetrateOnRS485(long deviceId, unsigned char *buf, int bufSize)
 {
     return IRNet::penetrateOnRS485(deviceId,buf,bufSize);
 }

 int IRNetSDKManager::setRtpOverMode(long deviceId, bool isTcp)
 {
     return IRNet::setRtpOverMode(deviceId,isTcp);
 }

 int IRNetSDKManager::getADTransferModen(long deviceId)
 {
     return IRNet::getADTransferModen(deviceId);
 }

 int IRNetSDKManager::setADTransferModen(long deviceId, int moden)
 {
     return IRNet::setADTransferModen(deviceId,moden);
 }

 int IRNetSDKManager::ptzControl(long deviceId,unsigned char num,unsigned char mode, unsigned char speed)
 {
     return IRNet::executePTZControl(deviceId,num,(IRNet::PTZControl)mode,speed);
 }

 int IRNetSDKManager::ptzControlLeft(long deviceId, unsigned char num, unsigned char speed)
 {
     return IRNet::executePTZControl(deviceId,num,PTZ_LEFT,speed);
 }

 int IRNetSDKManager::ptzControlTop(long deviceId, unsigned char num, unsigned char speed)
 {
     return IRNet::executePTZControl(deviceId,num,PTZ_UP,speed);
 }

 int IRNetSDKManager::ptzControlRight(long deviceId, unsigned char num, unsigned char speed)
 {
     return IRNet::executePTZControl(deviceId,num,PTZ_RIGHT,speed);
 }

 int IRNetSDKManager::ptzControlDown(long deviceId, unsigned char num, unsigned char speed)
 {
     return IRNet::executePTZControl(deviceId,num,PTZ_DOWN,speed);
 }

 int IRNetSDKManager::ptzControlStop(long deviceId, unsigned char num)
 {
     return IRNet::executePTZControl(deviceId,num,PTZ_STOP,0);
 }

 float IRNetSDKManager::getPTZPitchAngle(long deviceId, unsigned char num)
 {
     return IRNet::getPTZPitchAngle(deviceId,num);
 }

 float IRNetSDKManager::getPTZRollAngle(long deviceId, unsigned char num)
 {
     return IRNet::getPTZRollAngle(deviceId,num);
 }

 int IRNetSDKManager::setPTZPreset(long deviceId, unsigned char num, unsigned char presetNum)
 {
     return IRNet::setPTZPreset(deviceId,num,presetNum);
 }

 int IRNetSDKManager::executePTZPreset(long deviceId, unsigned char num, unsigned char presetNum)
 {
     return IRNet::executePTZPreset(deviceId,num,presetNum);
 }

 int IRNetSDKManager::delPTZPreset(long deviceId, unsigned char num, unsigned char presetNum)
 {
     return IRNet::delPTZPreset(deviceId,num,presetNum);
 }

 int IRNetSDKManager::executeHorVerPTZControl(long deviceId, unsigned char num, PTZControl moden, unsigned char hs, unsigned char vs)
 {
     return IRNet::executeHorVerPTZControl(deviceId,num,moden,hs,vs);
 }

 int IRNetSDKManager::setJPGTransferParam(long deviceId, unsigned char enable, unsigned char rate)
 {
     return IRNet::setJPGTransferParam(deviceId,enable,rate);
 }

 int IRNetSDKManager::getJPGTransferParam(long deviceId, unsigned char *enable, unsigned char *rate)
 {
     return IRNet::getJPGTransferParam(deviceId,enable,rate);
 }

 int IRNetSDKManager::setJPGTransferMode(long deviceId, unsigned char useTcp)
 {
     return IRNet::setJPGTransferMode(deviceId,useTcp);
 }

 int IRNetSDKManager::getJPGTransferMode(long deviceId, unsigned char *useTcp)
 {
     return IRNet::getJPGTransferMode(deviceId,useTcp);
 }

 bool IRNetSDKManager::NET_DVR_CaptureJPEGPicture_WithAppendData(long lUserID, long lChannel, NET_DVR_JPEGPICTURE_WITH_APPENDDATA *lpJpegWithAppend)
 {

     return IRNet::NET_DVR_CaptureJPEGPicture_WithAppendData(lUserID,lChannel,lpJpegWithAppend);
 }











