#ifndef IRNETSDKMANAGER_H
#define IRNETSDKMANAGER_H

#include <QObject>
#include "IRNET.H"

using namespace IRNet;

class AnalysisInfo;

class IRNetSDKManager : public QObject
{
    Q_OBJECT
private:
    IRNetSDKManager();


public:
    static IRNetSDKManager* GetInstance();

    void Init();

    void Destroy();

    QString GetSDKVersion();

    void ShowMachineVideo(bool is_show);

    void Connect(QString ip,int port,long device_id,int timeout);

    void Connect(QString ip,int port,long device_id,int cameraModel,int timeout);


    long disConnectNet(long device_id);

    void destroy();

    void RealPlay(long device_id);

    void stopPlay(long deviceId);

    void StartTemperatureMeasurement(long device_id);

    void StopTemperatureMeasurement(long device_id);

    void SnapJpegTemperature(long device_id, char* filename, long filetype,IRAnalysis *analysis,int len);

    int Record264video(long device_id, char* FileName, long RecordTime);

    void StopRecord264Video(long device_id);

    void SetPalette(long device_id, long paletteindex);

    long GetPalette(long device_id);

    void SetEmissivity(long device_id, double useemiss);

    double GetEmissivity(long device_id);

    void SetDistance(long device_id, double usedist);

    double GetDistance(long device_id);

    void SetTransmit(long device_id, double use_transmit);

    double GetTransmit(long device_id);

    void SetEnvironmentTemperature(long device_id, double use_env_tem);

    double GetEnvironmentTemperature(long device_id);

    void SetRelativeHumidity(long device_id, double use_humidity);

    double GetRelativeHumidity(long device_id);

    QString GetMachineSN(long device_id);

    void getAnalysis(long device_id,IRAnalysis* list,int len);

    void Disconnect(long device_id);

    void SetImageEnhancement(long device_id, long is_image_enhancement);


    void GetMachineAnalysis(long deviceId,IRAnalysis* list,int* len);

    void ClearMachineAnalysis(long deviceId);

    void SaveIrMachineAnalysis(long deviceId,IRAnalysis* list,int len);

    void SetMachineColorRegion(long device_id,int color_index, int x1, int y1, int x2, int y2);

    void CancelMachineColorRegion(long device_id);

    void SetTransmissionFrameRate(long deviceId,int frame_rate);

    void DoAdjust(long deviceId);

    long GetAllTemp(long deviceId, float* tempbuf, long Len);

    bool getMaxAndMinTempPoint(long deviceId, IRPointInfo *maxPointInfo, IRPointInfo *minPointInfo, double *avgTemp_);

    long GetIrDataLen(long deviceId, long* IrWidth, long* IrHeight);

    void GetDataSize(long deviceId, long* irwidth, long* irheight);

    void SearchCameraIP(long search_time);

    int getH264EncodecParam(long deviceId);

    void setH264EncodecParam(long deviceId, int value);

    int rebootMachine(long deviceId);

    int shopPhotoOnMachine(long deviceId);

    int getMaxAndMinTempFromMachine(long deviceId);

    int getAnalysisFromMachine(long deviceId);

    void setRTCTime(long deviceId);

    int startRecordOnMachine(long deviceId);

    int stopRecordOnMachine(long deviceId);

    int setVideoFormatOfMachine(long deviceId,int format);

    int getSDAvailableSpaceOfMachine(long deviceId);

    IRNet::IRRange getRange(long deviceId);

    void setRange(long deviceId,IRRange range);

    QString byteArrayToHexStr(const QByteArray &data);

    long getAllAD(long deviceId, unsigned short *buf, long Len);

    /*!
         * 自动调焦
         * @param deviceId 连接设备id
         * @return
         */
    int focusAuto(long deviceId);

    /*!
         * 正向调焦
         * @param deviceId 连接设备id
         * @return
         */
    int focusOut(long deviceId);

    /*!
         * 反向调焦
         * @param deviceId 连接设备id
         * @return
         */
    int focusIn(long deviceId);

    /*!
         * 设置调焦模式
         * @param moden 参数见：IRFocusModen
         * @param deviceId 连接设备id
         * @return
         */
    int focusModen(IRFocusModen moden,long deviceId);

    /*!
         * 获取画面最大和最小AD值
         * @param deviceId 连接设备id
         * @param max 最大值
         * @param min 最小值
         */
    void getADExtremum(long deviceId,unsigned short *max,unsigned short *min);

    /*!
         *获取levelspan信息
         * @param deviceId 连接设备id
         * @param max 最大值
         * @param min 最小值
         * @param level level值
         * @param span span值 大于256
         */
    void getLevelSpanInfo(long deviceId,unsigned short *max,unsigned short *min,unsigned short *level,unsigned short *span);

    /*!
         * 设置成像有效区域
         * @param deviceId 连接设备id
         * @param enable 是否启用有效区域
         * @param left 有效区域左上x
         * @param top 有效区域左上x
         * @param width 有效区域宽 大于100
         * @param height 有效区域高 大于100
         */
    int setEffectiveArea(long deviceId,bool enable,int left,int top,int width,int height);

    /*!
         * 设置自动levelspan
         * @param deviceId 连接设备id
         * @param autoModen  true自动levelspan
         */
    void setLevelSpanModen(long deviceId,bool autoModen);

    /*!
         * 设置levespan值 禁用自动模式时生效
         * @param deviceId
         * @param level
         * @param span
         */
    void setLevelSpan(long deviceId,unsigned short level,unsigned short span);

    float getTempFromRaw(long deviceId,unsigned short raw,float emissivity,float distance);

    int setSerialPortWorkMode(long deviceId, unsigned char mode);

    int getSerialPortWorkMode(long deviceId);

    int penetrateOnRS485(long deviceId,unsigned char *buf,int bufSize);

    int setRtpOverMode(long deviceId,bool isTcp);


    int  getADTransferModen(long deviceId);

    int  setADTransferModen(long deviceId,int moden);

    int ptzControl(long deviceId, unsigned char num,unsigned char mode,unsigned char speed);

    int ptzControlLeft(long deviceId,unsigned char num,unsigned char speed);

    int ptzControlTop(long deviceId,unsigned char num,unsigned char speed);

    int ptzControlRight(long deviceId,unsigned char num,unsigned char speed);

    int ptzControlDown(long deviceId,unsigned char num,unsigned char speed);

    int ptzControlStop(long deviceId,unsigned char num);

    float getPTZPitchAngle(long deviceId, unsigned char num);

    float getPTZRollAngle(long deviceId, unsigned char num);

    int setPTZPreset(long deviceId, unsigned char num, unsigned char presetNum);

    int executePTZPreset(long deviceId, unsigned char num, unsigned char presetNum);

    int delPTZPreset(long deviceId, unsigned char num, unsigned char presetNum);

    int executeHorVerPTZControl(long deviceId, unsigned char num, PTZControl moden, unsigned char hs, unsigned char vs);

    int  setJPGTransferParam(long deviceId, unsigned char enable, unsigned char rate);

    int  getJPGTransferParam(long deviceId, unsigned char *enable, unsigned char *rate);

    int  setJPGTransferMode(long deviceId, unsigned char useTcp);

    int  getJPGTransferMode(long deviceId, unsigned char *useTcp);

    bool  NET_DVR_CaptureJPEGPicture_WithAppendData(long lUserID, long lChannel, NET_DVR_JPEGPICTURE_WITH_APPENDDATA* lpJpegWithAppend);



signals:
    void sgnVideoData(uchar* rgbData, int w, int h, int macIndex,int type);
    void sgnConnectStatusChange(int status,int device_id);
    void sgnSearchCameraIP(QString ip);
    void sgnCameraLogInfo(QString msg);
    void sgnRS485Receive(QString msg);

};

#endif // IRNETSDKMANAGER_H
