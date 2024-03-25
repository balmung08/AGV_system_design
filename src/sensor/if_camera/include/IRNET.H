
#ifndef IRNET_IRNET_H
#define IRNET_IRNET_H

#if defined(WIN32) || defined(_WIN32)
#define IMGSITC extern "C" __declspec(dllexport)
#define STDCALL  __stdcall
#else
#define IMGSITC
#define STDCALL

typedef  unsigned int       DWORD;
typedef  unsigned short     WORD;
typedef  unsigned short     USHORT;
typedef  short              SHORT;
typedef  int                LONG;
typedef  unsigned char      BYTE;
typedef  unsigned int       UINT;
typedef  void* LPVOID;
typedef  void* HANDLE;
typedef  unsigned int* LPDWORD;
typedef  unsigned long long UINT64;
typedef  signed long long   INT64;

#endif

#if defined(_WIN64)

#endif // defined(_WIN64)




//typedef void (STDCALL *ccback)(char *name ,int length);
//int dllFunction(char *ip, int port, ccback cc);

namespace IRNet {

    enum ExceptionCode {
        EXP_CAMERA_UNCONNECT            = -128,
        EXP_SEND_CMD_FAIL               = -129,
        EXP_RS485_PENETRATE_LEN_LIMIT   = -485,

    };

    enum PTZControl {
        PTZ_UP          = 0x00,
        PTZ_DOWN        = 0x01,
        PTZ_LEFT        = 0x02,
        PTZ_RIGHT       = 0x03,
        PTZ_LEFT_UP     = 0x10,
        PTZ_LEFT_DOWN   = 0x11,
        PTZ_RIGHT_UP    = 0x12,
        PTZ_RIGHT_DOWN  = 0x13,
        PTZ_STOP        = 0xff,
    };



    /**
     * 红外成像视频流回调
     * rgbData setIRVideoFormat Format_RGB888 为 RGB数据  Format_BGR888 为BGR数据
     * ads 探测器采集数据矩阵buf 长度为 视频宽度*视频长度
     * w 视频宽度
     * h 视频长度
     * deivceId 连接时设置的设备id
     */
    typedef void (STDCALL* VideoDataCallBack)(const char* rgbData,const unsigned short *adBuf,int adBufSize, int w, int h, long deivceId);

    /**
     * rtsp 视频流回调
     * rgbData setIRVideoFormat Format_RGB888 为 RGB数据  Format_BGR888 为BGR数据
     * w 视频宽度
     * h 视频长度
     * deivceId 连接时设置的设备id
     */
    typedef void (STDCALL* MachineVideoDataCallBack)(const char* rgbData, int w, int h, long deivceId);

    /**
     * 设备连接状态回调
     * status 连接状态 0 连接成功 1 连接中 2 未连接 3 连接失败 4 连接断开
     */
    typedef void (STDCALL* ConnectStatusCallBack)(int status, long deivceId);

    /**
    * 搜索相机ip回调
    * ip 相机ip
    */
    typedef void (STDCALL* SearchCameraIPCallBack)(const char *ip,int len);

   /**
   * sdk log 信息回调
   * log信息
   */
    typedef void (STDCALL* SDKLogInfoCallBack)(const char* msg);

   /**
   * 接收RS485回传数据
   * buf 接收数据buf
   * len 接收数据长度
   */
    typedef void (STDCALL* RS485ReceiveCallBack)(const char* buf, int len);

    enum IRRerodFormat{
        IRRerodFormat_H264 =0,
        IRRerodFormat_MP4
    };

    /**
     * 调焦模式
     */
    enum IRFocusModen{
        IRFocusModenManual = 0, //手动调焦
        IRFocusModenAuto //自动调焦
    };

    /*!
     * 红外视频回调数据格式
     */
    enum IRVideoFormat{
        Format_RGB888 = 0,  //视频回调返回RGB
        Format_BGR888       //视频回调返回BGR
    };
    
    //区域框结构
    typedef struct tagNET_VCA_RECT
    {
        float fX;               //边界框左上角点的X轴坐标, 0.000~1
        float fY;               //边界框左上角点的Y轴坐标, 0.000~1
        float fWidth;           //边界框的宽度, 0.000~1
        float fHeight;          //边界框的高度, 0.000~1
    }NET_VCA_RECT, * LPNET_VCA_RECT;

//    typedef struct tagNET_DVR_JPEGPICTURE_WITH_APPENDDATA
//    {
//        unsigned long   dwSize;
//        unsigned long   dwChannel;//通道号
//        unsigned long   dwJpegPicLen;//Jpeg图片长度
//        char* pJpegPicBuff;//Jpeg图片指针
//        unsigned long   dwJpegPicWidth;  // 图像宽度
//        unsigned long   dwJpegPicHeight;  //图像高度
//        unsigned long   dwP2PDataLen;//全屏测温数据长度
//        char* pP2PDataBuff; //全屏测温数据指针   
//        unsigned char    byIsFreezedata;//是否数据冻结 0-否 1-是
//        unsigned char    byRes1[3];
//        unsigned long   dwVisiblePicLen;//可见光图片长度
//#if (defined(OS_WINDOWS64) || defined(OS_POSIX64))//win64及linux64下指针为8字节
//        char* pVisiblePicBuff;//可见光图片指针
//#else
//        char* pVisiblePicBuff;//可见光图片指针
//        unsigned char    byRes2[4];
//#endif 
//        NET_VCA_RECT struThermalValidRect;//热成像有效区域
//        NET_VCA_RECT struVisibleValidRect;//可见光有效区域
//        unsigned char    byRes[208];
//    }NET_DVR_JPEGPICTURE_WITH_APPENDDATA, * LPNET_DVR_JPEGPICTURE_WITH_APPENDDATA;

   
    
    typedef struct tagNET_DVR_JPEGPICTURE_WITH_APPENDDATA
    {
        unsigned long   dwSize;
        unsigned long   dwChannel;//通道号
        unsigned long   dwJpegPicLen;//Jpeg图片长度
        char* pJpegPicBuff;//Jpeg图片指针
        unsigned long   dwJpegPicWidth;  // 图像宽度
        unsigned long   dwJpegPicHeight;  //图像高度
        unsigned long   dwP2PDataLen;//全屏测温数据长度
        char* pP2PDataBuff; //全屏测温数据指针   
        unsigned char    byIsFreezedata;//是否数据冻结 0-否 1-是
        unsigned char    byRes1[3];
       
    }NET_DVR_JPEGPICTURE_WITH_APPENDDATA, * LPNET_DVR_JPEGPICTURE_WITH_APPENDDATA;

 

    /*!
     * 红外点信息
     */
    struct IRPointInfo{
        int x = -1; //x 坐标
        int y= -1; //y 坐标
        int ad=0; // 该点ad值
        double  temp_=0; //该点温度值

        IRPointInfo(){

        }

        IRPointInfo(int x,int y,int ad,double temp_){
            this->x = x;
            this->y = y;
            this->ad = ad;
            this->temp_ = temp_;
        }
    };

    /*!
     * 红外点信息
     */
    struct IRPonit
    {
        int x;  //x坐标
        int y;  //y坐标

        IRPonit(){

        }

        IRPonit(int x, int y) {
            this->x = x;
            this->y = y;
        }
    };

    struct IRRange{
        float low  = 0 ;    //测温档温度下限
        float up = 0;       //测温档温度上限
        int gear = 0;       //测温档
    };

    enum AnalysisType {
        PointAnalysis = 0,  //点分析
        LineAnalysis,       //线分析
        RectAnalysis,       //框分析
        CireAnalysis,       //圆分析
        PolyAnalysis,       //多边形分析
        MaxAnalysis,        //最高温
        MinAnalysis,        //最低温
        NoneAnalysis
    };

    /*!
     * 红外测温分析信息
     */
    struct IRAnalysis
    {
        IRPonit points[200];    //测温分析坐标数组
        AnalysisType type = AnalysisType::NoneAnalysis;     //测温分析类型
        int polySize = 0;       //多边形测温专用，点的个数
        int number = -1;        //测温分析唯一序号，用于区别
        double avgTemp_;        //区域测温分析平均温
        IRPointInfo maxPointInfo;   //测温分析最高温信息
        IRPointInfo minPointInfo;   //测温分析最低温信息
    };

    /*!
     * 红外相机型号
     */
    enum CameraModel{
        IR320 = 0,  //320机芯
        IR640       //640机芯
    };


    /*!
     * 获取SDK版本号
     * @param version sdk版本号字符串
     * @return 0 成功
     */
    IMGSITC int STDCALL getSDKVersion(char* version);

    /*!
     * 获取机芯RTSP地址
     * @param deviceId 连接的设备id
     * @param url 设备rtsp地址
     */
    IMGSITC void STDCALL  getRTSPUrl(long deviceId, char* url);

    /*!
     * 设置视频流回调函数中返回数据的格式
     * @param deviceId 连接的设备id
     * @param videoFormat  Format_RGB888 视频回调返回RGB Format_BGR888 视频回调返回BGR
     */
    IMGSITC void STDCALL setIRVideoFormat(long deviceId,IRVideoFormat videoFormat);



    /*!
     * 设置机芯区域伪彩
     * @param deviceId 连接设备id
     * @param colorIndex 伪彩索引
     * @param x1 左上角x
     * @param y1 左上角y
     * @param x2 右下角x
     * @param y2 右下角y
     */
    IMGSITC void STDCALL setMachineColorRegion(long deviceId,int colorIndex, int x1, int y1, int x2, int y2);

    /*!
     * 取消区域伪彩
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL cancelMachineColorRegion(long deviceId);


    /*!
     * 获取机芯测温分析
     * @param deviceId 连接设备id
     * @param list 测温分析信息
     * @param len 测温分析个数
     */
    IMGSITC void STDCALL getMachineAnalysis(long deviceId,IRAnalysis* list,int* len);
    
    /*!
     * 清除机芯测温分析
     * @param deviceId 连接的设备id
     */
    IMGSITC void STDCALL clearMachineAnalysis(long deviceId);


    /*!
   * 保存测温分析到机芯
   * @param deviceId 连接设备id
   * @param list 测温分析信息
   * @param len 测温分析长度
   */
    IMGSITC void STDCALL saveIrMachineAnalysis(long deviceId,IRAnalysis* list,int len);


    /*!
     * 获取全画幅最高温和最低温
     * @param deviceId 连接设备id
     * @param maxPointInfo 最高温点信息
     * @param minPointInfo 最低温点信息
     * @param avgTemp_ 平均温度
     * @return true调用成功
     */
    IMGSITC bool STDCALL getMaxAndMinTempPoint(long deviceId,IRPointInfo* maxPointInfo,IRPointInfo* minPointInfo,double* avgTemp_);
    
    /*!
     * 获取区域测温分析信息
     * @param deviceId 连接设备id
     * @param list 测温分析信息
     * @param len 测温分析的个数
     * @return 测温分析结果长度 不等于len为失败
     */
    IMGSITC int STDCALL getAnalysis(long deviceId,IRAnalysis* list,int len);

    /*!
     * 获取全画幅温度
     * @param deviceId 连接设备id
     * @param tempbuf 温度矩阵
     * @param Len tempbuf长度
     * @return  大于0 成功 等于0 失败
     */
    IMGSITC  long STDCALL getAllTemp(long deviceId, float* tempbuf, long Len);

    /*!
    * 获取全画幅AD
    * @param deviceId 连接设备id
    * @param buf AD数组
    * @param Len buf
    * @return AD数组长度
    */
    IMGSITC  long STDCALL getAllAD(long deviceId, unsigned short *buf, long Len);

    /*!
     * 获取红外矩阵的长、宽
     * @param deviceId 连接设备
     * @param IrWidth 红外矩阵宽度
     * @param IrHeight 红外矩阵长度
     * @return 红外矩阵的长
     */
    IMGSITC long STDCALL getIrDataLen(long deviceId, long* IrWidth, long* IrHeight);



    /*!
     * 添加点测温分析  弃用
     * @param deviceId 连接设备id
     * @param point 点坐标
     * @param id 测温分析id
     * @param len 测温分析id字符串长度
     */
    IMGSITC void STDCALL addPointXYd(long deviceId, IRPonit point, char* id, int* len);

    /*!
     * 添加线测温分析 弃用
     * @param deviceId 连接设备id
     * @param startPonit 线起始点
     * @param endPoint 线终结点
     * @param id 测温分析id
     * @param len 测温分析id字符串长度
     */
    IMGSITC void STDCALL addLineXY(long deviceId, IRPonit startPonit, IRPonit endPoint, char* id, int* len);

    /*!
     * 添加圆测温分析 弃用
     * @param deviceId 连接设备id
     * @param centerPoint 圆心坐标
     * @param radius 圆半径
     * @param id 测温分析id
     * @param len 测温分析id字符串长度
     */
    IMGSITC void STDCALL addCircleXY(long deviceId, IRPonit centerPoint, int radius, char* id, int* len);

    /*!
     * 添加矩形测温分析 弃用
     * @param deviceId 连接设备id
     * @param leftTopPoint 左上角左边
     * @param rightBottomPoint 右下角坐标
     * @param id 测温分析id
     * @param len 测温分析id字符串长度
     */
    IMGSITC void STDCALL addRectXY(long deviceId, IRPonit leftTopPoint, IRPonit rightBottomPoint, char* id, int* len);

    /*!
     * 添加多边形测温分析 弃用
     * @param deviceId 连接设备id
     * @param pt 多边形顶点数组
     * @param plen 顶点数组长度
     * @param id 测温分析id
     * @param len 测温分析id字符串长度
     */
    IMGSITC void STDCALL addPolyXY(long deviceId, IRPonit* pt, int plen, char* id, int* len);

    /*!
     * 开始录制RTSP视频流为MP4
     * @param deviceId 连接设备id
     * @param FileName mp4保存路径
     * @param RecordTime 录制时长 默认为0 保留
     * @return 0成功
     */
    IMGSITC int STDCALL record264video(long deviceId, char* FileName, long RecordTime);

    /*!
     * 结束录制MP4
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL stopRecord264Video(long deviceId);

    /*!
     * 初始化SDK
     */
    IMGSITC void STDCALL init();

    /*!
     * 连接机芯 弃用
     * @param ip 机芯ip
     * @param port 机芯数据端口
     * @param deviceId 设备id由应用分配和管理
     * @param timeOut 连接超时
     * @param isAutoConnect 是否由SDK自动连接，自动断线重连
     */
    IMGSITC void STDCALL connect_(char* ip, int port, long deviceId, int timeOut,bool isAutoConnect = true);

    /*!
     * 获取设备连接状态
     * @param deviceId
     * @return
     */
    IMGSITC int STDCALL  getConnectStatus(long deviceId);

    /*!
    * 连接机芯
    * @param ip 机芯ip
    * @param port 机芯数据端口
    * @param deviceId 设备id由应用分配和管理
    * @param model 设备型号详见CameraModel
    * @param timeOut 连接超时
    * @param isAutoConnect 是否由SDK自动连接，自动断线重连
    */
    IMGSITC void STDCALL connectWithModel_(char* ip, int port, long deviceId,CameraModel model, int timeOut,bool isAutoConnect = true);

    /*!
     * 断开设备连接
     * @param deviceId 连接设备id
     * @return 0 成功
     */
    IMGSITC long STDCALL disConnectNet(long deviceId);

    /*!
     * 关闭网络接收 弃用
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL disconnect_(long deviceId);

    /*!
     * 设置红外成像视频回调
     * @param cb 回调函数指针
     */
    IMGSITC void STDCALL setVideoDataCallBack(VideoDataCallBack cb);

    /*!
     * 设置sdk log信息回调
     * @param cb log信息
     */
    IMGSITC void STDCALL setSDKLogInfoCallBack(SDKLogInfoCallBack cb);

    /*!
     * 设置485接收数据回调
     * @param cb 485接收数据
     */
    IMGSITC void STDCALL setRS485ReceiveCallBack(RS485ReceiveCallBack cb);

    /*!
     * 设置RTSP视频流回调
     * @param cb 回调函数指针
     */
    IMGSITC void STDCALL setMachineVideoDataCallBack(MachineVideoDataCallBack cb);

    /*!
     * 连接状态回调设置
     * @param cb 回调函数指针
     */
    IMGSITC void STDCALL setConnectStatusCallBack(ConnectStatusCallBack cb);

    /*!
     * 搜索相机ip回调
     * @param cb 回调函数指针
     */
    IMGSITC void STDCALL setSearchCameraIPCallBack(SearchCameraIPCallBack cb);

    /*!
     * 拉取机芯RTSP视频流
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL realPlay(long deviceId);

    /*!
     * 停止拉取机芯RTSP视频流
     */
    IMGSITC void STDCALL stopPlay(long deviceId);

    /*!
     * 设置RTSP客户端RTP传输方式
     * @param deviceId
     * @param isTcp true 使用tcp传输
     * @return 0 success
     */
    IMGSITC int STDCALL setRtpOverMode(long deviceId,bool isTcp);

    /*!
   * 获取RTSP RTP Over 模式
    * @param deviceId 连接设备id
    * @return bool 使用TCP传输
   */
    IMGSITC bool STDCALL rtpOverTcp(long deviceId);

     /*!
      * 设置温度数据传输帧率
      * @param deviceId 连接设备id
      * @param frame_rate 帧率 1-25帧/秒
      */
     IMGSITC void STDCALL setTransmissionFrameRate(long deviceId,int frame_rate);

     IMGSITC int STDCALL test(long frame_rate,int b);

    /*!
     * 开始测温
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL startTemperatureMeasurement(long deviceId);

    /*!
     * 停止测温
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL stopTemperatureMeasurement(long deviceId);

    /*!
     * 查询当前测温档
     * @param deviceId 连接设备id
     * @return 测温档信息
     */
    IMGSITC IRRange STDCALL getRange(long deviceId);

    /*!
     * 设置测温档
     * @param deviceId 连接设备id
     * @param range 测温档信息
     * @return  >0 success other fail
     */
    IMGSITC int STDCALL setRange(long deviceId,IRRange range);

    /*!
     * 设置伪彩
     * @param deviceId 连接设备id
     * @param paletteindex 伪彩序号
     */
    IMGSITC void STDCALL setPalette(long deviceId, long paletteindex);

    /*!
     * 获取伪彩值
     * @param deviceId 连接设备id
     * @return 伪彩值
     */
    IMGSITC long STDCALL getPalette(long deviceId);

    /*!
     * 设置辐射率
     * @param deviceId 连接设备id
     * @param useemiss 辐射率
     */
    IMGSITC void STDCALL setEmissivity(long deviceId, double useemiss);

    /*!
     * 获取辐射率
     * @param deviceId 连接设备id
     * @return 辐射率
     */
    IMGSITC double STDCALL getEmissivity(long deviceId);

    /*!
     * 设置距离
     * @param deviceId 连接设备id
     * @param usedist 距离
     */
    IMGSITC void STDCALL setDistance(long deviceId, double usedist);

    /*!
     * 获取距离
     * @param deviceId 连接设备id
     * @return 距离
     */
    IMGSITC  double STDCALL getDistance(long deviceId);

    /*!
     * 设置透过率
     * @param deviceId 连接设备id
     * @param use_transmit 透过率
     */
    IMGSITC void STDCALL setTransmit(long deviceId, double use_transmit);

    /*!
     * 获取透过率
     * @param deviceId 连接设备id
     * @return 透过率
     */
    IMGSITC double STDCALL getTransmit(long deviceId);

    /*!
     * 设置环境温度
     * @param deviceId 连接设备id
     * @param use_env_tem 环境温度
     */
    IMGSITC void STDCALL setEnvironmentTemperature(long deviceId, double use_env_tem);

    /*!
     * 获取环境温度
     * @param deviceId 连接设备id
     * @return 环境温度
     */
    IMGSITC double STDCALL getEnvironmentTemperature(long deviceId);

    /*!
     * 设置相对湿度
     * @param deviceId 连接设备id
     * @param use_humidity 相对湿度
     */
    IMGSITC void STDCALL setRelativeHumidity(long deviceId, double use_humidity);

    /*!
     * 获取相对湿度
     * @param deviceId 连接设备id
     * @return 相对湿度
     */
    IMGSITC double STDCALL getRelativeHumidity(long deviceId);

     /*!
      * 设置修正温度
      * @param deviceId 连接设备id
      * @param use_modify_temp 修正温度
      */
    IMGSITC void STDCALL setModifyTemperature(long deviceId, double use_modify_temp);

    /*!
     * 获取修正温度
     * @param deviceId 连接设备id
     * @return 修正温度
     */
    IMGSITC double STDCALL getModifyTemperature(long deviceId);

    /*!
     * 温度校正
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL doAdjust(long deviceId);

    /*!
     * 销毁SDK
     */
    IMGSITC void STDCALL destroy();

    /*!
     * 获取红外数据矩阵的宽高
     * @param deviceId 连接设备id
     * @param irwidth 宽
     * @param irheight 高
     */
    IMGSITC void STDCALL getDataSize(long deviceId, long* irwidth, long* irheight);

    /*!
     * 是否正在进行温度数据传输
     * @param deviceId 连接设备id
     * @return true 正在传输
     */
    IMGSITC bool STDCALL istrasferdata(long deviceId);

   /*!
    * 查询视频制式
    * @param deviceId 连接设备id
    * @return 0：PAL制 1：NTSC制
    */
    IMGSITC long STDCALL getVideoFormat(long deviceId);

    /*!
     * 设置视频制式
     * @param deviceId 连接设备id
     * @param videoformat 0：PAL制 1：NTSC制
     * @return true 成功
     */
    IMGSITC bool STDCALL setVideoFormat(long deviceId, long videoformat);


    /*!
     * 视频输出设置
     * @param deviceId 连接设备id
     * @param IsVideoOut 1 输出 0 不输出
     * @return true 成功
     */
    IMGSITC bool STDCALL setVideoOut(long deviceId, long IsVideoOut);

    /*!
    * 设置温度参数 弃用
    * @param deviceId 连接设备id
    * @param datatype 类型
    * @param paradata data
    */
    IMGSITC void STDCALL setOneTempParameter(long deviceId, unsigned char datatype, float paradata);

    /*!
     * 同步机芯时钟
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL setRTCTime(long deviceId);

    /*!
     * 恢复默认测温参数 弃用
     * @param deviceId 连接设备id
     */
    IMGSITC void STDCALL setDefaultTempParameter(long deviceId);

    /*!
     * 获取机芯电子变倍参数
     * @param deviceId 连接设备
     * @return 电子变倍参数
     */
    IMGSITC long STDCALL getElectronicZoom(long deviceId);

    /*!
     * 设备机芯图像电子变倍
     * @param deviceId 连接设备id
     * @param ZoomData 1 2 4  8 16
     * @return
     */
    IMGSITC bool STDCALL setElectronicZoom(long deviceId, long ZoomData);

    /*!
    * 获取机芯网络信息
    * @param deviceId 连接设备id
    * @param ip_str 机芯IP
    * @param netmask_str 机芯掩码
    * @param gate_way_str 机芯网关
    * @param mac_str 机芯mac地址
    * @param len 字符串数组长度
    * @return true成功
    */
    IMGSITC bool STDCALL getIrNetInfo(long deviceId, char* ip_str, char* netmask_str, char* gateway_str, char* mac_str, long* len);

    /*!
     * 设置机芯网络信息
     * @param deviceId 连接设备id
     * @param ip_str 机芯IP
     * @param netmask_str 机芯掩码
     * @param gate_way_str 机芯网关
     * @param mac_str 机芯mac地址
     * @param len 字符串数组长度
     * @return true成功
     */
    IMGSITC  bool STDCALL setIrNetInfo(long deviceId, char* ip_str, char* netmask_str, char* gate_way_str, char* mac_str, long len);

    /*!
     * 获取红外成像视频宽高
     * @param deviceId 连接设备id
     * @param IrDataWidth 宽
     * @param IrDataHeight 高
     */
    IMGSITC void STDCALL  getIrDataSize(long deviceId, long* IrDataWidth, long* IrDataHeight);

    /*!
     * 获取热成像RGB buf
     * @param deviceId 连接设备
     * @param rgbbuf rgb buf
     * @return true成功
     */
    IMGSITC bool STDCALL getIrRgbBuff(long deviceId, unsigned char* rgbbuf);



    /*!
     * 获取机芯SN码
     * @param deviceId 连接设备id
     * @param ir_mac_sn 机芯SN码
     * @return
     */
    IMGSITC bool STDCALL getMacSN(long deviceId, char* ir_mac_sn);

    /*!
     * 发送自定义数据到机芯
     * @param deviceId 连接设备id
     * @param buf 数据
     * @param len 数据长度
     */
    IMGSITC void STDCALL sendMachineData(long deviceId, unsigned char* buf, long len);

    /*!
     * 保存jpg
     * @param deviceId 连接设备id
     * @param filename 保存路径
     * @param filetype 0 国网格式igp 1 普通jpg 2 rtsp抓图
     */
    IMGSITC void STDCALL snapJpegTemperature(long deviceId, char* filename, long filetype,IRAnalysis *analysis,int len);

    /*!
     * set distance switch
     * @param deviceId 连接设备id
     * @param Dswitch 0 switch close; 1 switch open.
     */
    IMGSITC void STDCALL setDistance_switch(long deviceId, bool Dswitch);

    /*!
    * 获取相对湿度
    * @param deviceId 连接设备id
    * @return Germaiumwindow temperature
    */
    IMGSITC float STDCALL getGermaiumwindow(long deviceId);

    /*!
    * set Germaiumwindow
    * @param deviceId 连接设备id
    * @param Gswitch : Germaiumwindow switch
    * @param paradata :Germaiumwindow temperature.
    */
    IMGSITC void STDCALL setGermaiumwindow(long deviceId, int Gswitch,float paradata);

    /*!
     * 是否启用图像增强
     * @param deviceId 连接设备id
     * @param is_image_enhancement 是否启用图像增强
     */
    IMGSITC void STDCALL setImageEnhancement(long deviceId, long is_image_enhancement);

    /*!
     * 获取264编码参数
     * @param deviceId 连接设备id
     */
    IMGSITC int STDCALL getH264EncodecParam(long deviceId);

    /*!
     * 设置264编码参数
     * @param deviceId 连接设备id
     * @param value 参数值
     */
    IMGSITC void STDCALL setH264EncodecParam(long deviceId, int value);

    /*!
     * 搜索局域网内相机ip
     * @param cb
     * @param searchTime 搜索ip的时长 单位：毫秒
     */
    IMGSITC void STDCALL searchCameraIP(long searchTime);


    /*!
     * 机芯抓图
     * @param deviceId 连接设备id
     * @return sd卡可用空间
     */
    IMGSITC int STDCALL shopPhotoOnMachine(long deviceId);


    /*!
     * 开始机芯视频录制
     * @param deviceId 连接设备id
     * @return sd卡可用空间
     */
    IMGSITC int STDCALL startRecordOnMachine(long deviceId);


    /*!
     * 停止机芯视频录制
     * @param deviceId 连接设备id
     * @return sd卡可用空间
     */
    IMGSITC int STDCALL stopRecordOnMachine(long deviceId);

    /*!
     * 设置机芯录制视频格式
     * @param deviceId 连接设备id
     * @param format 录制视频格式
     * @return 大于0 成功
     */
    IMGSITC int STDCALL setVideoFormatOfMachine(long deviceId,IRRerodFormat format);

    /*!
     * 获取机芯sd卡容量
     * @param deviceId 连接设备id
     * @return sd卡可用空间
     */
    IMGSITC int STDCALL getSDAvailableSpaceOfMachine(long deviceId);

    /*!
     * 重启机芯
     * @param deviceId 连接设备id
     * @return 0 成功
     */
    IMGSITC int STDCALL rebootMachine(long deviceId);

    /*!
     * 自动调焦
     * @param deviceId 连接设备id
     * @return 大于0 成功
     */
    IMGSITC int STDCALL focusAuto(long deviceId);

    /*!
     * 正向调焦
     * @param deviceId 连接设备id
     * @return 大于0 成功
     */
    IMGSITC int STDCALL focusOut(long deviceId);

    /*!
     * 反向调焦
     * @param deviceId 连接设备id
     * @return 大于0 成功
     */
    IMGSITC int STDCALL focusIn(long deviceId);

    /*!
     * 设置调焦模式
     * @param moden 参数见：IRFocusModen
     * @param deviceId 连接设备id
     * @return 大于0 成功
     */
    IMGSITC int STDCALL focusModen(long deviceId,IRFocusModen moden);

    /*!
     * 获取画面最大和最小AD值
     * @param deviceId 连接设备id
     * @param max 最大值
     * @param min 最小值
     */
    IMGSITC void STDCALL getADExtremum(long deviceId,unsigned short *max,unsigned short *min);

    /*!
     *获取levelspan信息
     * @param deviceId 连接设备id
     * @param max 最大值
     * @param min 最小值
     * @param level level值
     * @param span span值 大于256
     */
    IMGSITC void STDCALL  getLevelSpanInfo(long deviceId,unsigned short *max,unsigned short *min,unsigned short *level,unsigned short *span);

    /*!
     * 设置成像有效区域
     * @param deviceId 连接设备id
     * @param enable 是否启用有效区域
     * @param left 有效区域左上x
     * @param top 有效区域左上x
     * @param width 有效区域宽 大于100
     * @param height 有效区域高 大于100
     */
    IMGSITC int STDCALL setEffectiveArea(long deviceId,bool enable,int left,int top,int width,int height);


    /*!
     * 设置自动levelspan
     * @param deviceId 连接设备id
     * @param autoModen  true自动levelspan
     */
    IMGSITC void STDCALL setLevelSpanModen(long deviceId,bool autoModen);

    /*!
     * 设置levespan值 禁用自动模式时生效
     * @param deviceId 连接设备id
     * @param level
     * @param span
     */
    IMGSITC void STDCALL setLevelSpan(long deviceId,unsigned short level,unsigned short span);

    /*!
     * 获取报警开关状态
     * @param deviceId 连接设备id
     * @return true 开 other 关
     */
    IMGSITC bool STDCALL getAlarmStatus(long deviceId);

    /*!
     * 报警开关操作
     * @param deviceId 连接设备id
     * @param enable true 打开报警开关 other 关闭报警开关
     * @return >0 success
     */
    IMGSITC int STDCALL setAlarmStatus(long deviceId,bool enable);

    /*!
     * 获取H264 I帧间隔
     * @param deviceId 连接设备id
     * @return I帧间隔
     */
    IMGSITC int STDCALL getH264IFrameInterval(long deviceId);

    /*!
     * 设置I帧间隔
     * @param deviceId 连接设备id
     * @param interval 设置I帧间隔
     * @return >0 success
     */
    IMGSITC int STDCALL setH264IFrameInterval(long deviceId,unsigned char interval);

    /*!
     * 报警开关设置
     * @param deviceId 连接设备id
     * @param enable true 开启
     * @return 0 成功
     */
    IMGSITC int STDCALL setAlarmStatus(long deviceId,bool enable);

    /*!
     * 根据红外采集数据获取温度
     * @param deviceId 连接设备id
     * @param raw 红外数据
     * @param emissivity 辐射率
     * @param distance 距离
     * @return 温度
     */
    IMGSITC float STDCALL getTempFromRaw(long deviceId, unsigned short raw, float emissivity, float distance);


    /*!
     * 设置串口工作模式
     * @param deviceId 连接设备id
     * @param mode 工作模式 4：RS485
     * @return 0 成功
     */
    IMGSITC int STDCALL setSerialPortWorkMode(long deviceId, unsigned char mode);


    /*!
     * 获取串口工作模式
     * @param deviceId 连接设备id
     * @return 0 工作模式
     */
    IMGSITC int STDCALL getSerialPortWorkMode(long deviceId);

    /*!
     * RS485数据透传
     * @param deviceId 连接设备id
     * @param buf 透传数据
     * @param bufSize 透传数据长度
     * @return 0 成功
     */
    IMGSITC int STDCALL penetrateOnRS485(long deviceId,unsigned char *buf,int bufSize);


    /*!
     * 从机芯获取最高温和最低温
     * @param deviceId 连接设备id
     * @param analysis 分析信息
     * @return 0 成功
     */
    IMGSITC int STDCALL getMaxAndMinTempFromMachine(long deviceId,IRAnalysis *analysis);

    /*!
     * 从机芯获取测温分析数据
     * @param deviceId 连接设备id
     * @param pointInfo 分析信息
     * @return 0 成功
     */
    IMGSITC int STDCALL getAnalysisFromMachine(long deviceId,IRAnalysis *pointInfo);

    /*!
     * 获取ad传输模式
     * @param deviceId 连接设备id
     * @return 0 udp 1 tcp
     */
    IMGSITC int STDCALL getADTransferModen(long deviceId);

    /*!
     * 设置ad传输模式
     * @param deviceId 连接设备id
     * @param moden
     * @return 0 成功
     */
    IMGSITC int STDCALL setADTransferModen(long deviceId,int moden);

    /*!
     * 云台控制
     * @param deviceId 连接设备id
     * @param num 云台地址号
     * @param moden 云台控制模式
     * @param speed 云台速度
     * @return >0 success other fail
     */
    IMGSITC int STDCALL executePTZControl(long deviceId, unsigned char num , PTZControl moden, unsigned char speed);

    /*!
    * 云台控制
    * @param deviceId 连接设备id
    * @param num 云台地址号
    * @param moden 云台控制模式
    * @param hs 水平速度
    * @param vs 垂直速度
    * @return >0 success other fail
    */
    IMGSITC int STDCALL executeHorVerPTZControl(long deviceId, unsigned char num, PTZControl moden, unsigned char hs, unsigned char vs);

    /*!
     * 获取云台俯仰角度
     * @param deviceId 连接设备id
     * @param num 云台地址号
     * @return 云台俯仰角度
     */
    IMGSITC float STDCALL getPTZPitchAngle(long deviceId, unsigned char num);

    /*!
    * 获取云台水平角度
    * @param deviceId 连接设备id
    * @param num 云台地址号
    * @return 云台水平角度
    */
    IMGSITC float STDCALL getPTZRollAngle(long deviceId, unsigned char num);


    /*!
     * 设置云台预置位
     * @param deviceId 连接设备id
     * @param num 云台地址号
     * @param presetNum 预置位号
     * @return >0 success other fail
     */
    IMGSITC int STDCALL setPTZPreset(long deviceId, unsigned char num, unsigned char presetNum);

    /*!
     * 执行云台预置位
     * @param deviceId 连接设备id
     * @param num 云台地址号
     * @param presetNum 预置位号
     * @return >0 success other fail
     */
    IMGSITC int STDCALL executePTZPreset(long deviceId, unsigned char num, unsigned char presetNum);

    /*!
    * 删除云台预置位
    * @param deviceId 连接设备id
    * @param num 云台地址号
    * @param presetNum 预置位号
    * @return >0 success other fail
    */
    IMGSITC int STDCALL delPTZPreset(long deviceId, unsigned char num, unsigned char presetNum);

    /*!
     *设置jgp传输模式的参数
     * @param deviceId 连接设备id
     * @param enable 1 启用 0 禁用
     * @param rate 传输帧率
     * @return > 0 success other fail
     */
    IMGSITC int STDCALL setJPGTransferParam(long deviceId, unsigned char enable, unsigned char rate);

    /*!
     * 获取jgp传输模式的参数
     * @param deviceId 连接设备id
     * @param enable 1 启用 0 禁用
     * @param rate 传输帧率
     * @return 等于 0 success other fail
     */
    IMGSITC int STDCALL getJPGTransferParam(long deviceId, unsigned char *enable, unsigned char *rate);

    /*!
     * 设置jpg传输模式
     * @param deviceId 连接设备id
     * @param useTcp 1 使用tcp传输 0 使用udp传输
     * @return > 0 success other fail
     */
    IMGSITC int STDCALL setJPGTransferMode(long deviceId, unsigned char useTcp);

    /*!
    * 获取jpg传输模式
    * @param deviceId 连接设备id
    * @param useTcp 1 使用tcp传输 0 使用udp传输
    * @return 等于 0 success other fail
    */
    IMGSITC int STDCALL getJPGTransferMode(long deviceId, unsigned char *useTcp);

    IMGSITC bool STDCALL NET_DVR_CaptureJPEGPicture_WithAppendData(int lUserID, int lChannel, NET_DVR_JPEGPICTURE_WITH_APPENDDATA* lpJpegWithAppend);

    //IMGSITC void STDCALL NET_DVR_CaptureJPEGPicture(int lUserID, int lChannel,NET_DVR_JPEGPICTURE_WITH_APPENDDATA* lpJpegWithAppend);
}


#endif //IRNET_IRNET_H
