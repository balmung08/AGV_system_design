#ifndef ITTPLAYERWIDGET_H
#define ITTPLAYERWIDGET_H

#include <QWidget>
#include <QMap>
#include <QMutex>
#include <QMutexLocker>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "IRNET.H"

class QGridLayout;
class QLabel;
class AnalysisGraphicsWidget;
class AnalysisGraphicsItem;
class QTimer;
class AnalysisInfo;

extern int rect_Xcenter;
extern int rect_Ycenter;
extern int rect_bias;

extern int rect_Xcenter2;
extern int rect_Ycenter2;
extern int rect_bias2;

class PlayerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlayerWidget(QWidget *parent = 0);
    ~PlayerWidget();
    void AddPointAnalysis(QList<QPoint> points);
    void AddLineAnalysis(QList<QPoint> points);
    void AddRectAnalysis(QList<QPoint> points);
    void AddCireAnalysis(QList<QPoint> points);
    void AddPolyAnalysis(QList<QPoint> points);

    void ClearAnalysis();

    void SetFollowMaxTemp(bool is_follow);
    void SetFollowMinTemp(bool is_follow);
    bool IsFollowMaxTemp();
    bool IsFollowMinTemp();

    void ShowMachineVideo(bool is_show);
    bool IsShowMachineVideo(){return is_show_machine_video;}

    void SetDeviceId(long device_id);//infrared_image

    void Connect(QString ip,int port,int timeout);

    void Connect(QString ip,int port,int model,int timeout);

    QString GetMachineSN();

    long disConnectNet();

    void RealPlay();

    void stopPlay();

    void StartTemperatureMeasurement();

    void StopTemperatureMeasurement();

    void SnapJpegTemperature(char* filename);

    int Record264video(char* FileName, long RecordTime);

    void StopRecord264Video();

    void SetPalette(long index);

    long GetPalette();

    void SetEmissivity(double useemiss);

    double GetEmissivity();

    void SetDistance(double usedist);

    double GetDistance();

    void SetTransmit( double use_transmit);

    double GetTransmit();

    void SetEnvironmentTemperature( double use_env_tem);

    double GetEnvironmentTemperature();

    void SetRelativeHumidity(double use_humidity);

    double GetRelativeHumidity();

    void Disconnect();

    void SetImageEnhancement(long is_image_enhancement);

    void GetMachineAnalysis();

    void SaveMachintAnalysis();

    void ClearMachineAnalysis();

    void SetMachineColorRegion(int color_index, int x1, int y1, int x2, int y2);

    void CancelMachineColorRegion();

    void SetTransmissionFrameRate(int frame_rate);

    void DoAdjust();

    long GetAllTemp(float* tempbuf, long Len);

    bool getMaxAndMinTempPoint(IRNet::IRPointInfo *maxPointInfo, IRNet::IRPointInfo *minPointInfo, double *avgTemp_);

    long GetIrDataLen(long* IrWidth, long* IrHeight);

    void SearchCameraIP(long search_time);

    int getH264EncodecParam();

    void setH264EncodecParam(int value);

    int rebootMachine();

    int shopPhotoOnMachine();

    int getMaxAndMinTempFromMachine();

    int getAnalysisFromMachine();

    void setRTCTime();

    int startRecordOnMachine();

    int stopRecordOnMachine();

    int setVideoFormatOfMachine(int format);

    int getSDAvailableSpaceOfMachine();

    IRNet::IRRange getRange();

    void setRange(IRNet::IRRange range);

    long getAllAD(unsigned short *buf, long Len);


    int focusAuto();


    int focusOut();


    int focusIn();

    int focusModen(int moden);

    void getADExtremum(unsigned short *max,unsigned short *min);

    void getLevelSpanInfo(unsigned short *max,unsigned short *min,unsigned short *level,unsigned short *span);

    int setEffectiveArea(bool enable,int left,int top,int width,int height);

//infrared_image
    void setLevelSpanModen(bool autoModen);

    void setLevelSpan(unsigned short level,unsigned short span);

    float getTempFromRaw(unsigned short raw,float emissivity,float distance);

    int setSerialPortWorkMode(unsigned char mode);

    int getSerialPortWorkMode();

    int penetrateOnRS485(unsigned char *buf,int bufSize);

    int setRtpOverMode(bool isTcp);

    int  getADTransferModen();

    int  setADTransferModen(int moden);
//infrared_image
    int ptzControl(unsigned char num,unsigned char mode,unsigned char speed);

    int ptzControlLeft(unsigned char num,unsigned char speed);

    int ptzControlTop(unsigned char num,unsigned char speed);

    int ptzControlRight(unsigned char num,unsigned char speed);

    int ptzControlDown(unsigned char num,unsigned char speed);

    int ptzControlStop(unsigned char num);

    float getPTZPitchAngle(unsigned char num);

    float getPTZRollAngle(unsigned char num);

    int setPTZPreset(unsigned char num, unsigned char presetNum);

    int executePTZPreset(unsigned char num, unsigned char presetNum);

    int delPTZPreset(unsigned char num, unsigned char presetNum);

    int executeHorVerPTZControl(unsigned char num, int moden, unsigned char hs, unsigned char vs);

    void showJPGMode(bool show);

    int  setJPGTransferParam(unsigned char enable, unsigned char rate);

    int  getJPGTransferParam(unsigned char *enable, unsigned char *rate);

    int  setJPGTransferMode(unsigned char useTcp);

    int  getJPGTransferMode(unsigned char *useTcp);

    bool  NET_DVR_CaptureJPEGPicture_WithAppendData();

    void paintEvent(QPaintEvent *event);

    cv::Mat srcdata;
    // cv::Mat dstdata;

    IRNet::IRPointInfo *maxPoint;
    IRNet::IRPointInfo *minPoint;
    double avgPoint;
    
    IRNet::IRPointInfo *maxPoint2;
    IRNet::IRPointInfo *minPoint2;
    double avgPoint2;

    //IRNet::IRAnalysis analysisList_t[100];

    // double *avgTemp_;

signals:
    void sgnJPGShow(QByteArray jpg,unsigned short *temps,int w, int h);

    void sgnPointTempInfo(int x,int y,float temp);

protected:
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseDoubleClickEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);

private:

    QMutex mtx;

    bool isShowJpg = false;

    QGridLayout* root_gridLayout;
    QLabel* player_label;
    AnalysisGraphicsWidget* analysis_graphicsWidget;
    bool is_show_machine_video = false;
    bool is_follow_max = false;
    bool is_follow_min = false;
    long m_device_id = -1;

    long m_ir_width = 0;
    long m_ir_height = 0;

    long m_IRRealWidth = 0;
    long m_IRRealHeight = 0;

    int m_video_width = 0;
    int m_video_height = 0;

    int analysis_number = 0;

    // int rect_Xcenter =0;
    // int rect_Ycenter =0;
    // int shift_from_center = 50;
    // int type_of_analysis = 1;   //分析模式选择，若为1，则只检测rect内的温度；若为3，检测rect、最大、最小三种
    unsigned char *m_pRGBBug;

    QTimer *timer = nullptr;

    QByteArray m_JPGData;
    unsigned short *m_FrameTemps;

    void initWidgets();
    void ShowTip(QString text);
    void StartTimer();
    void StopTimer();

    void AddAnalysis(int type,QList<QPoint> points,int tag);

    void GetAnalysis(IRNet::IRAnalysis *analysis,int &len);



private slots:
    void OnVideoShow(uchar* rgbData, int w, int h, int macIndex,int type);

    void OnJPGShow(QByteArray jpg,unsigned short *temps,int w, int h);

    void OnConnectStatusChange(int status,int deviceId);

    void OnTimeout();
};

#endif // ITTPLAYERWIDGET_H
