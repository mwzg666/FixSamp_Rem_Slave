#ifndef __LCDAPP_H__
#define __LCDAPP_H__

#define BUTTON_OK   0x0D0A
#define LOGIN_PASSWORD  1689



// 设备寄存器
// 启动界面
#define REG_STRART    0x1900    // 开始按钮
#define REG_PASSWORD   0x1901
#define REG_MODEHINT   0x1800

// 主界面
#define REG_DEV_CTL    0x1080   // 启停按钮
#define REG_RETRUN     0x1081   // 返回按钮
#define REG_TIME_BTN   0x1082   // 点击时间 
#define REG_CH_FLOW    0x1000   // 流量 0x1000 -- 0x1007
#define REG_CH_STATUS  0x1200   // 通道状态

#define REG_HINT_ENTER  0x1083  // 取样结束提示框上的确认按钮 0x0D0A
#define REG_HINT_END    0x1084  // 取样结束提示框变量地址
#define MP_HINT_END     0xA800  // 取样结束提示框描述指针


// 参数设置界面
#define REG_SYS_PARAM  0x4000   // 屏幕参数起始地址
#define REG_SP_FLOW1  0x4000
#define REG_SP_FLOW2  0x4001
#define REG_SP_FLOW3  0x4002
#define REG_SP_FLOW4  0x4003
#define REG_SP_FLOW5  0x4004
#define REG_SP_FLOW6  0x4005
#define REG_SP_FLOW7  0x4006
#define REG_SP_FLOW8  0x4007
#define REG_SP_ADDR   0x4008
#define REG_SP_TIME   0x4009
#define REG_SP_VOL    0x400A
#define REG_SP_LIGHT  0x400B
#define REG_SP_THRES  0x400C
#define REG_SP_MODE   0x400D
#define REG_CH_ONOFF  0x400E

#define REG_MODE_TXT 0x4010
#define REG_SP_RET    0x4020

#define REG_SYS_CHANNEL  0x4030   //各通道参数起始地址
#define REG_SP_CHANNEL1  0x4030
#define REG_SP_CHANNEL2  0x4031
#define REG_SP_CHANNEL3  0x4032
#define REG_SP_CHANNEL4  0x4033
#define REG_SP_CHANNEL5  0x4034
#define REG_SP_CHANNEL6  0x4035
#define REG_SP_CHANNEL7  0x4036
#define REG_SP_CHANNEL8  0x4037

#define REG_SP_CHANNEL1_MODE    0x4038    // 各通道下拉菜单
#define REG_SP_CHANNEL2_MODE 0x4039
#define REG_SP_CHANNEL3_MODE 0x403A
#define REG_SP_CHANNEL4_MODE 0x403B
#define REG_SP_CHANNEL5_MODE 0x403C
#define REG_SP_CHANNEL6_MODE 0x403D
#define REG_SP_CHANNEL7_MODE 0x403E
#define REG_SP_CHANNEL8_MODE 0x403F

#define  REG_SP_CHANNEL1_MODETXT       0x4040    // 各通道显示取样模式
#define  REG_SP_CHANNEL2_MODETXT    0x4048
#define  REG_SP_CHANNEL3_MODETXT    0x4050
#define  REG_SP_CHANNEL4_MODETXT    0x4058
#define  REG_SP_CHANNEL5_MODETXT    0x4060
#define  REG_SP_CHANNEL6_MODETXT    0x4068
#define  REG_SP_CHANNEL7_MODETXT    0x4070
#define  REG_SP_CHANNEL8_MODETXT    0x4078


#define REG_SP_VALVE1  0x4500       //远程控制显示(8个通道)
#define REG_SP_VALVE2  0x4501
#define REG_SP_VALVE3  0x4502
#define REG_SP_VALVE4  0x4503
#define REG_SP_VALVE5  0x4504
#define REG_SP_VALVE6  0x4505
#define REG_SP_VALVE7  0x4506
#define REG_SP_VALVE8  0x4507

// 修改时间
#define REG_ADJ_TIME   0x2010  // 返回按钮
#define REG_DATE_TIME  0x2000  // 7个地址

// IO测试
#define REG_IO_BUMP     0x5000
#define REG_IO_BLUE     0x5001
#define REG_IO_YELLOW   0x5002
//#define REG_IO_RED      0x5003
#define REG_IO_SOUND    0x5004
#define REG_IO_FAN      0x5005
#define REG_IO_EMSTOP   0x5006
#define REG_IO_TEMP     0x5007

// 关于界面
#define REG_DEV_TYPE  0x7000    // 设备型号
#define REG_SOFT_VER  0x7010    // 软件版本
#define REG_CH_COUNT  0x7020    // 通道数量

#define PAGE_START   0
#define PAGE_MAIN   1
#define PAGE_SET    4
#define PAGE_TIME   6
#define PAGE_REM    8

#pragma pack(1)
typedef struct
{
    WORD Flow;    // 流量：0.1L/min
    WORD Vol;     // 体积：0.1m3,最大6553m3
}CHANNEL_FLOW;

typedef struct
{
    DWORD Flow;  // 流量：0.1L/min
    DWORD Vol;   // 体积：0.1m3
}TOTLEFLOW;


// 显示屏运行界面信息
typedef struct
{
    CHANNEL_FLOW ChFlow[CHANNLE_NUM];
    TOTLEFLOW    TotFlow;
    char         RunTime[16];
}RUN_INFO;


typedef struct
{
    WORD Flow[CHANNLE_NUM];
    WORD Address;
    WORD SampTime;
    WORD SampVol;
    WORD BkLight;
    WORD AlarmThres;
    WORD SampMode;
    WORD ChOnOff;    // 使用低8位，1位指示一个通道
    WORD res;
    char ModeTxt[16];  // 取样模式
}SHOW_PARAM;

typedef struct
{
    WORD ChannelParm[CHANNLE_NUM];
    WORD ChannelMode[8];
    char ChannelModeTxt[8][16];
}CHANNEL_PARAM;

typedef struct
{
    WORD YearH;
    WORD YearL;
    WORD Month;
    WORD Day;
    WORD Hour;
    WORD Minute;
    WORD Secend;
}LCD_DATE_TIME;

typedef struct
{
    char DevType[16];
    WORD res[2];
    char SoftVer[16];
}DEV_INFO;
#pragma pack()


extern RUN_INFO xdata RunInfo;
extern char xdata ChannelStatus[CHANNLE_NUM][16];
extern BYTE ChannelAlarm[CHANNLE_NUM];
extern BYTE HisAlarm[CHANNLE_NUM];
extern bool NeedGetRet;
extern WORD xdata  RemChStatus[CHANNLE_NUM];

void EnterPage(BYTE Page);
void ShowFlow();
void ShowStatus();
void SendParam();
void SendChannelParam();

void ShowDevInfo();
void SetStartBtn(BYTE s);
void StatusColor(bool force);

void ReadReg();
void WriteReg();
void ModeHint();
void GetRetCode();

void ShowTemp(u16 TEMPER);
void ShowEmStop(bool on);
void ShowRemCh();


#endif
