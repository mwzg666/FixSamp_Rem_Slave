#include "main.h"
#include "mcp4725.h"
#include "Lcd.h"
#include "LcdApp.h"
#include "ModBus.h"
#include "Temper.h"
#include "FlowMeter.h"
#include "ModBusDev.h"
#include "ModBusHost.h"


BYTE code VERSION = 101;  // V1.0.0

BYTE xdata StrTmp[64] = {0};
//BYTE xdata Valve[8] = {0};

BYTE ChannelError[FLOW_METER_CNT] ={0};

#define Log //((CSampDemoDlg *)theApp.m_pMainWnd)->AddLog

#define PARAM_SIGN  0x3132
SYS_PARAM xdata SysParam;
RUN_STATUS xdata RunStatus;
REM_REGADDR xdata RemRegAddr;
REMRUN_STATUS xdata RemRunStatus;

float SimFlow = 35.0;

u16 SendFlowTim = 0;    //读流量计开始时间
BYTE SendFlowFlag = 0;  //读流量计开始标志

u16 DelayCount = 0;   
BYTE Delayflag = 0;  

BYTE RemAckOut = 0;    //远程控制从机响应超时标志
u16 RemAckTimout = 0;   //远程控制从机响应超时时间 

u16 LcdBusyTim = 0;
BYTE LcdBusyFlag = 0;
BYTE ChNum = 1;

BYTE g_Output[OUT_IO_COUNT]      = {0,0,0,0,0,0,0,0,0,0,0,0,0};   // 上电蓝灯亮
BYTE g_OutStatus[OUT_IO_COUNT]   = {0,0,0,0,0,0,0,0,0,0,0,0,0};

BYTE PageSwitch = 0;                              //远程控制界面选择

BYTE RemPage = 0;
BYTE RemStart = 0;
BYTE RemStop = 0;

// Clear Rem ARM 
BYTE xdata Remchenable[CHANNLE_NUM] = {0};
BOOL xdata Remchflag[CHANNLE_NUM] = {0};
WORD Remchtim = 0;
BOOL ValveIOFlag = false;
BYTE ChannelStop = 0;

u16  Timer0Cnt = 0;

BYTE g_Key_Confrom  = 0; 
BYTE g_Key_Power  = 0; 
BYTE g_Key_Input  = 0; 
BYTE Input_Status = 0;

WORD gRunTime = 0;

void DebugMsg(char *msg)
{
    BYTE len = (BYTE)strlen(msg);
    //Uart1Send((BYTE *)msg,len);
}

void DebugInt(int msg)
{
    memset(StrTmp,0,64);
    sprintf(StrTmp,"%x\r\n",msg);
    DebugMsg(StrTmp);
}

void DumpCmd(BYTE *dat, BYTE len)
{
    BYTE i;
    memset(StrTmp,0,64);
    for (i=0;i<len;i++)
    {
        if (strlen(StrTmp) >= 60)
        {
            break;
        }
        sprintf(&StrTmp[i*3], "%02X ", dat[i]);
    }
    sprintf(&StrTmp[i*3], "\r\n");
    DebugMsg(StrTmp);
}


void Error()
{
    while(1)
    {
        RUN_LED(1);
        Delay(50);
        RUN_LED(0);
        Delay(50);
    }
    
}


void SysInit()
{
    HIRCCR = 0x80;           // 启动内部高速IRC
    while(!(HIRCCR & 1));
    CLKSEL = 0;              
}

void IoInit()
{
    EAXFR = 1;
    WTST = 0;   //设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快

    P0M1 = 0x00;   P0M0 |= (1<<4) ;                     // P0.0 P0.1 P0.4 推挽输出
    P1M1 = (1<<4)|(1<<3);   P1M0 = 0x00;                       //设置为准双向口
    P2M1 = 0x00;   P2M0 |= 0x00;                      // P2.2 推挽输出
    P3M1 = 0x00;   P3M0 |= (1<<2)|(1<<3)|(1<<4);        //设置为准双向口
    P4M1 = 0x00;   P4M0 = 0x00;                       //设置为准双向口
    P5M1 = 0x00;   P5M0 |= (1<<0) | (1<<2);             //设置为准双向口
    P6M1 = 0x00;   P6M0 |= (1<<7);     //设置为准双向口
    P7M1 = 0x00;   P7M0 = 0x00;                         //设置为准双向口
}


void SensorInit()
{
    // P1.0 -- 下降缘中断
    P1IM0 = 0;
    P1IM1 = 0;

    // 优先级2
    //PIN_IP  |= (1<<1);
    PINIPH |= (1<<1);
    //P1_IP  = 1; // |= (1<<1);
    //P1_IPH = 1; //|= (1<<1);

    // 允许中断
    P1INTE |= (1<<0) | (1<<1) | (1<<4) | (1<<5);
}



void Timer0Init()
{
    AUXR = 0x00;    //Timer0 set as 12T, 16 bits timer auto-reload, 
    TH0 = (u8)(Timer0_Reload / 256);
    TL0 = (u8)(Timer0_Reload % 256);
    ET0 = 1;    //Timer0 interrupt enable
    TR0 = 1;    //Tiner0 run
    
    // 中断优先级3
    PT0  = 1;
    PT0H = 1;
}

// 10ms 中断一下
void Timer0Int (void) interrupt 1
{
    Timer0Cnt ++;
   
    if(Delayflag)
    {
        DelayCount -= 10;
        if(!DelayCount)
        {
            Delayflag = 0;
        }
    }
}

#if 0
// 公用中断服务程序
void CommInt (void) interrupt 13
{
    u8 intf =  P1INTF;
    
    if (intf)
    {
        P1INTF = 0;

        if (intf & (1<<0))  // P1.0 中断
        {
            Counter[0] ++;
        }

        if (intf & (1<<1))  // P1.1 中断
        {
            Counter[1] ++;
        }

        if (intf & (1<<4))  // P1.4 中断
        {
            Counter[2] ++;
        }

        if (intf & (1<<5))  // P1.5 中断
        {
            Counter[3] ++;
        }
    }
    
}
#endif

//========================================================================
// 函数名称:void OutCtl(alt_u8 id, alt_u8 st)
// 函数功能:IO输出控制 
// 入口参数: @id：控制IO序号 @st：IO口上一个状态
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023
// 当前作者: 
// 其他备注: 
//========================================================================

void OutCtl(alt_u8 id, alt_u8 st)
{
    if (g_OutStatus[id] == st)
    {
        return;
    }

    g_OutStatus[id] = st;
    
    switch(id)
    {   
        case LIGHT_BLUE: 
        {
            (st)? BLU_LIGHT(1) : BLU_LIGHT(0); 
            break;
        }
    
        case LIGHT_YELLOW: 
        {
            (st)? YEL_LIGHT(1):YEL_LIGHT(0);
            break;
        }

        case GAS_BUMP:      //泵
        {
            (st)? BUMP_M(1) : BUMP_M(0);
            break;
        }

        case EX_FAN:        //风扇
        {
            (st)? FANS_M(1) : FANS_M(0);
            break;
        }

        case ALARM_SOUND:   //报警
        {
            (st)? ALARM(1) : ALARM(0);       
            break;
        }

        case VALVE_0:   
        {
            (st)? VALVE0(1) : VALVE0(0);       
            break;
        }

        case VALVE_1:   
        {
            (st)? VALVE1(1) : VALVE1(0);       
            break;
        }  

        case VALVE_2:   
        {
            (st)? VALVE2(1) : VALVE2(0);       
            break;
        }

        case VALVE_3:   
        {
            (st)? VALVE3(1) : VALVE3(0);       
            break;
        }   

        case VALVE_4:   
        {
            (st)? VALVE4(1) : VALVE4(0);       
            break;
        }

        case VALVE_5:   
        {
            (st)? VALVE5(1) : VALVE5(0);       
            break;
        }   

        case VALVE_6:   
        {
            (st)? VALVE6(1) : VALVE6(0);       
            break;
        }

        case VALVE_7:   
        {
            (st)? VALVE7(1) : VALVE7(0);       
            break;
        }   
    }

    
}

void OutFlash(alt_u8 id)
{
    static alt_u16 OutTimer[OUT_IO_COUNT] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
    if (OutTimer[id] ++ > LED_FLASH_TIME/10)
    {
        OutTimer[id] = 0;
        if (g_OutStatus[id] == 1)
        {
            OutCtl(id, 0);
        }
        else
        {
            OutCtl(id, 1);
        }
    }
}

void IoCtlTask()
{
    alt_u8 i;
    for (i=0;i<OUT_IO_COUNT;i++)
    {
        if (g_Output[i] == 2)
        {
            OutFlash(i);
        }
        else
        {
            OutCtl(i, g_Output[i]);
        }
    }
}

// 板载指示灯
void RunLed(u16 dt)
{   
    static u16 tm = 0;
    u16 to = 3000;
    tm += dt;

    if (tm > to)
    {
        tm = 0;
        RUN_LED(0);
    }
    else if (tm > (to-100))
    {
        RUN_LED(1);
    }
}



void Task1s()
{
    static BYTE tm = 0;

    CLR_WDT = 1;  // 喂狗
    tm++;
    if(tm == 10)
    {
        ADC_Temp();
        //SyncModBusDev();
        tm = 0;
    } 
    
    if((tm == 6)||(tm == 3))
    {
        GetRetCode();
    }
    
    if(tm == 9)
    {
        if (RunStatus.Running)
        {  
            DevRun();
        }
    }

}


void TimerTask()
{
    u16 delta = 0;
    static u16 Time1s = 0;
    static u16 RemTime = 0;
    if (Timer0Cnt)
    {
        delta = Timer0Cnt * 10;
        Timer0Cnt = 0;

        Time1s += delta;
        if (Time1s >= 100)
        {
            Time1s = 0;
            Task1s();
        }
        
        if (RX2_Cnt > 0)
        {
            Rx2_Timer += delta;
        }

        if(RX3_Cnt > 0)
        {
            Rx3_Timer += delta;
        }
        
        if(RX4_Cnt > 0)
        {
            Rx4_Timer += delta;
            
        }
        
        SendFlowTim += delta;
        if(SendFlowTim > 220)
        {
            SendFlowTim = 0;
            SendFlowFlag = 1;
        }
        
        if(RemAckOut)
        {
            RemAckTimout += delta;
            if(RemAckTimout > 3000)
            {
                RemAckTimout = 0;
                SysParam.RemCtlFlag = false;
            }
                
        }  
        
        if (gRunTime < 5000)
        {
            gRunTime += delta;
        }

        //if (g_CommIdleTime < 300)
        //{
            //g_CommIdleTime += delta;
        //}

        #ifdef IRDA_FUN
        if (IrDAStart == 1)
        {
            IrDATimer += delta;
        }
        #endif

        RunLed(delta);
        IoCtlTask();
		
        GetValve();
        if(SysParam.RemCtlFlag)
        {
            ShowRemCh();
        }
    }
}

void delay_ms(u16 ms)
{
    DelayCount = ms;
    Delayflag = 1;  
    while(Delayflag);        
}


void Delay(WORD ms)
{
    WORD t = 1000;
    while(ms--)
    {
        for (t=0;t<1000;t++) ;
    }
}


WORD ParamCheck(BYTE *buf, WORD len)
{
    WORD dwSum = 0;
    WORD i;

    for (i = 0; i < len; i++)
    {
        dwSum += buf[i];
    }

    return dwSum;
}

/*
void DefSenParam()
{
    BYTE i;
    for (i=0; i<SENSOR_COUNT; i++)
    {
        SysParam.SenParam[i].LOW_REVISE_COE_A = 1;
        SysParam.SenParam[i].LOW_REVISE_COE_B = 1;
        SysParam.SenParam[i].LOW_REVISE_COE_C = 1;

        SysParam.SenParam[i].HIG_REVISE_COE_A = 1;
        SysParam.SenParam[i].HIG_REVISE_COE_B = 1;
        SysParam.SenParam[i].HIG_REVISE_COE_C = 1;

        SysParam.SenParam[i].SUPER_REVISE_COE_A = 1;
        SysParam.SenParam[i].SUPER_REVISE_COE_B = 1;
        SysParam.SenParam[i].SUPER_REVISE_COE_C = 1;

        SysParam.SenParam[i].DET_THR_1 = 500;
        SysParam.SenParam[i].DET_THR_2 = 150;
        SysParam.SenParam[i].DET_THR_3 = 150;

        SysParam.SenParam[i].DET_TIME = 1000;
        SysParam.SenParam[i].HV_THR = 1000;
    }
}
*/

/*
void DefSenAlarm()
{
    BYTE i;
    for (i=0; i<SENSOR_COUNT; i++)
    {
        SysParam.AlmParam[i].DOSE_RATE_ALARM_1 = 300;
        SysParam.AlmParam[i].DOSE_RATE_ALARM_2 = 400;
        SysParam.AlmParam[i].CUM_DOSE_RATE_ALARM_1 = 300;
        SysParam.AlmParam[i].CUM_DOSE_RATE_ALARM_2 = 400;
        SysParam.AlmParam[i].INVALID_ALRAM_1 = 8000;
        SysParam.AlmParam[i].INVALID_ALRAM_2 = 10000;
    }
}
*/

void ReadParam()
{
    EEPROM_read(0, (BYTE *)&SysParam, sizeof(SYS_PARAM));

    #if 0
    memset(StrTmp,0,32);
    sprintf((char *)StrTmp,"%d \r\n",sizeof(SYS_PARAM));
    DebugMsg((char *)StrTmp);

    memset(StrTmp,0,32);
    sprintf((char *)StrTmp,"%d \r\n",sizeof(HOST_SENSOR_PARAM));
    DebugMsg((char *)StrTmp);

    memset(StrTmp,0,32);
    sprintf((char *)StrTmp,"%d \r\n",sizeof(HOST_ALRAM_PARA));
    DebugMsg((char *)StrTmp);

    memset(StrTmp,0,32);
    sprintf((char *)StrTmp,"%d \r\n",sizeof(float));
    DebugMsg((char *)StrTmp);
    
    //Rs485Send((BYTE *)&SysParam, sizeof(SYS_PARAM));
    
    
    if (SysParam.Sign != PARAM_SIGN)
    {
        DebugMsg("Sign error. \r\n");
    }

    if (SysParam.Sum != ParamCheck((BYTE *)&SysParam, sizeof(SYS_PARAM)-2))
    {
        DebugMsg("Param Check error. \r\n");
    }
    #endif

   
    if ( (SysParam.Sign != PARAM_SIGN) ||
         (SysParam.Sum != ParamCheck((BYTE *)&SysParam, sizeof(SYS_PARAM)-2)) )
    {
        //SysParam.Sign = PARAM_SIGN;
        //SysParam.Address = 1;
        ParamDef();
        //DefSenParam();
        //DefSenAlarm();
        WriteParam();

        //DebugMsg("Def Param. \r\n");
    }
}


void WriteParam()
{
    EA = 0;    
    
    EEPROM_SectorErase(0);
    EEPROM_SectorErase(512);
    SysParam.Sum = ParamCheck((BYTE *)&SysParam, sizeof(SYS_PARAM)-2);
    if (!EEPROM_write(0, (BYTE *)&SysParam, sizeof(SYS_PARAM)))
    {
        Error();
    }
    //printf("Write34= OK\r\n");
    EA = 1;     //打开总中断
}

BYTE GetInput()
{
    // 当前只有一个开关机状态 P2.1
    static BYTE his = LOCK_BIT();
    BYTE st = POWER_LOCK();

    if (st != his)
    {
        Delay(50);
        if ( st == POWER_LOCK() )
        {
            his = st;
            return st;
        }
    }

    return 0xFF;
}


void PowerOff()
{
    PW_MAIN(0);

    while(1)
    {
        ;
    }
}

void HndInput()
{
    static bool em = false;
    if(STOP_M() == 0)
    {
        Delay(10);
        if (STOP_M() == 0)
        {
            if (RunStatus.Running)
            {
                StopSamp(false);
            }
        }

        if (em == false)
        {
            em = true;
            ShowEmStop(em);
        }
    }
    else
    {
        if (em)
        {
            em = false;
            ShowEmStop(em);
        }
    }
}

/*
void ReportInput()
{
    BYTE PwOff = POWER_OFF;
    
    if (g_CommIdleTime > 200)
    {
        if (g_Key_Confrom)
        {
            g_Key_Confrom = 0;
            SendPcCmd(0, CMD_CERTAINKEY, NULL, 0);
            return;
        }

        if (g_Key_Power)
        {
            g_Key_Power = 0;
            SendPcCmd(0, CMD_POWER, &PwOff, 1);
            return;
        }

        #if 0
        if (g_Key_Input)
        {
            g_Key_Input = 0;
            SendPcCmd(0, CMD_INPUT, &Input_Status, 1);
        }
        #endif
    }
}
*/

void LedInit()
{
    // 初始状态都为0

    // 指示灯
    YEL_LIGHT(0);   // 黄灯
    BLU_LIGHT(0);   // 蓝灯
    
    CloseValve();   // 电磁阀
    BUMP_M(0);      // 泵
    FANS_M(0);      // 风扇
    ALARM(0);       // 报警音
}


void ParamDef()
{
    BYTE i;
    
    SysParam.Sign     = PARAM_SIGN;
    SysParam.Address = 1;
    SysParam.BkLight = 50;

    SysParam.SampMode = MODE_TIME;
    SysParam.SampTime = 5;  
    SysParam.SampVol   = 2;
    SysParam.AlarmThres   = 10;
    for (i=0;i<CHANNLE_NUM;i++)
    {
        SysParam.SampFlow[i] = 35;
        SysParam.Channel_SampMode[i] = MODE_NOCHANNEL;
        SysParam.Channel_SampFlowVol[i] = 2;
    }

    SysParam.Enable = 0x00;
    SysParam.ChModeCtl = 0x00;
	SysParam.RemCtlFlag = false;
    RemRunStatus.RemRun = false; 
    RemRunStatus.HostRun = true;
	ChannelStop = 0;
}

void SaveParam()
{
    //CString t;
    //t.Format(_T("SaveParam: %02X\r\n"), SysParam.Enable);
    //Log(t);
    //DebugMsg("123");
    WriteParam();
}


void UpdataUI()
{
    BYTE i;
    for (i=0;i<CHANNLE_NUM;i++)
    {
        ChannelAlarm[i] = ((SysParam.Enable & (1<<i)) == 0)?0:1;
    }
    ShowStatus();
    Delay(200);
    StatusColor(true);
}

void InitLcd()
{   
    memset(&RunStatus, 0, sizeof(RUN_STATUS));
    memset(&RunInfo, 0, sizeof(RUN_INFO));
    memset(&RealFlow, 0, sizeof(RealFlow));
    
    ModeHint();
    Delay(200);
    HideModule(MP_HINT_END);
    Delay(200);
    UpdataUI();    
    Delay(200);
    SendParam();
    Delay(200);
	SendChannelParam();
	Delay(200);
    SetBkLight(false);
    Delay(200);
    ShowDevInfo();
    Delay(200);
}


void GetFlow()
{
    BYTE i;
    WORD  w;
    DWORD d;

    
    for (i=0;i<CHANNLE_NUM;i++)
    {
//        if (SysParam.Enable & (1<<i))
//        {
            RunStatus.Flow[i] = RealFlow[i].val;  // 模拟 -- 实际要从流量计中读取
            w = (WORD)(RunStatus.Flow[i]*10);
            RunInfo.ChFlow[i].Flow = SwWord(w);
            
            RunStatus.Volume[i] =  RealFlow[i].Totol; 
            w = (WORD)(RunStatus.Volume[i]*10);
            RunInfo.ChFlow[i].Vol = SwWord(w);
//        }
    }

    // 总流量
    RunStatus.TotleFlow = RealFlow[8].val;
    d = (DWORD)(RealFlow[8].val*10);
    RunInfo.TotFlow.Flow = SwDWord(d);

    // 总体积
    RunStatus.TotleVol = RealFlow[8].Totol;
    d = (DWORD)(RealFlow[8].Totol*10);
    RunInfo.TotFlow.Vol   = SwDWord(d);
}

void StartSamp()
{
    BYTE i = 0;
    memset(&RunStatus, 0, sizeof(RUN_STATUS));
    memset(&RunInfo, 0, sizeof(RUN_INFO));
    memset(&RealFlow, 0, sizeof(RealFlow));
    SysParam.ChModeCtl = 0x00;
    RemRunStatus.HostRun = true;
    RunStatus.Running = true;
    g_Output[LIGHT_BLUE] = 1;
    //CheckValve();
    OpenPump();
    memcpy(HisAlarm,ChannelAlarm,CHANNLE_NUM);
    ShowStatus();
    StatusColor(true);
    //g_Output[ALARM_SOUND] = 0; 

    if(!SysParam.RemCtlFlag)
    {
        SetStartBtn(0);
    }

}

void StopSamp(bool Auto)
{
    BYTE i = 0;
    ClosePump();
    memset(RealFlow,0, sizeof(RealFlow));
	ChannelStop = 0;
    RunStatus.Running = false;
    g_Output[LIGHT_BLUE] = 0;
    g_Output[LIGHT_YELLOW] = 0; 
    g_Output[ALARM_SOUND] = 0; 
    for (i=0;i<CHANNLE_NUM;i++)
    {
        ChannelAlarm[i] = ((SysParam.Enable & (1<<i)) == 0)?0:1;
    }
   
    if(!SysParam.RemCtlFlag)
    {
        SetStartBtn(1);  // 按钮自动变为“开始”
    }
    if (Auto)  // 自动结束
    {
        // 显示取样结束提示框
        ShowModule(MP_HINT_END, REG_HINT_END);
		//Delay(200);
    }
	SaveParam();
}


void CheckModeStop()
{
	BYTE i = 0;
	static BYTE sta = 0; 
    if(SysParam.Enable == 0)
    {
        RemRunStatus.HostRun = false;
        if(SysParam.RemCtlFlag)
        {
            StopSamp(false);
        }
        else
        {
            StopSamp(true);
        }
    }
}

// 定时模式
void TimingMode()
{
    BYTE i = 0,j = 0;
    for(i = 0;i<CHANNLE_NUM;i++)
    {
       
        if(SysParam.Channel_SampMode[i] == MODE_VOL)
        {
            if(RunStatus.Volume[i] >= (SysParam.Channel_SampFlowVol[i]))
            {
             	if(!SysParam.RemCtlFlag)
			    {
			        SysParam.Enable &= ~(1<<i);   
			    }               
                SysParam.ChModeCtl |= (1<<i);
				
            }
        }
        else if(SysParam.Channel_SampMode[i] == MODE_TIME)
        {
            if (RunStatus.RunTime[i] >= ((DWORD)SysParam.Channel_SampFlowVol[i]) * 60)
            {
           	 	if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);
            }
        }
		else
        {
            if (RunStatus.RunTime[8] >= ((DWORD)SysParam.SampTime) * 60)
            {
                if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);

            }
        }
    }
    CheckModeStop();
}

void Channel_ManMode()
{
    BYTE i = 0;
    for(i = 0;i<CHANNLE_NUM;i++)
    {
        if(SysParam.Channel_SampMode[i] == MODE_VOL)
        {
            if(RunStatus.Volume[i] >= (SysParam.Channel_SampFlowVol[i]))
            {
                if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);
            }
        }
        else if(SysParam.Channel_SampMode[i] == MODE_TIME)
        {
            if (RunStatus.RunTime[8] >= ((DWORD)SysParam.Channel_SampFlowVol[i]) * 60)
            {
                if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);
            }
        }
    }
    CheckModeStop();
}

// 定量模式
void VolumeMode()
{
    BYTE i = 0,j = 0;
    for(i = 0;i<CHANNLE_NUM;i++)
    {
        
        if(SysParam.Channel_SampMode[i] == MODE_VOL)
        {
            if(RunStatus.Volume[i] >= (SysParam.Channel_SampFlowVol[i]))
            {
                if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);
            }
        }
        else if(SysParam.Channel_SampMode[i] == MODE_TIME)
        {
            if (RunStatus.RunTime[i] >= ((DWORD)SysParam.Channel_SampFlowVol[i]) * 60)
            {
                if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);
            }
        }
		else
        {
        	//ChannelStop |= (1<<i);
            if (RunStatus.Volume[i] >= SysParam.SampVol)
            {
                if(!SysParam.RemCtlFlag)
			    {
                	SysParam.Enable &= ~(1<<i);
           	 	}
                SysParam.ChModeCtl |= (1<<i);
            }
        }
    }
    CheckModeStop();
}


void RunCheck()
{
    switch (SysParam.SampMode)
    {
        case MODE_TIME:  TimingMode();  break;
        case MODE_VOL:   VolumeMode();  break;
        default: Channel_ManMode();break;
    }
	//Delay(200);
	SendParam();
	
//	SendChannelParam();
//	Delay(200);
}

void AbnorAlaerm()
{   
    BYTE i;
    bool HaveAlarm = false;

    for(i = 0;i < CHANNLE_NUM;i++)
    {
        if( (ChannelAlarm[i] == ALM_FLOW_ABNOR) ||
            (ChannelAlarm[i] ==  ALM_FLOW_LOW) || 
            (ChannelAlarm[i] == ALM_FLOW_HIGH)  )
        {
            HaveAlarm = true;
            break;
        }
    }

    if (HaveAlarm)
    {
        //printf("HAveALM_true\r\n");
        g_Output[LIGHT_YELLOW] = 1;
        g_Output[ALARM_SOUND] = 2;
    }
    else
    {
        //printf("HAveALM_false\r\n");
        g_Output[LIGHT_YELLOW] = 0;
        g_Output[ALARM_SOUND] = 0;
    }
}
void CheckAlarm()
{
    BYTE i;
    float flow = 0;
    static BYTE time[8] = {0};
    for (i=0;i<CHANNLE_NUM;i++)
    {
        if (SysParam.Enable & (1<<i))
        {
            if(Remchflag[i])
            {
                time[i]++;
                if(time[i] > 2)
                {
                    time[i] = 0;
                    Remchflag[i] = false;
                    flow = RunStatus.Flow[i];
                    if (flow > SysParam.SampFlow[i]*(100+SysParam.AlarmThres)/100)
                    {
                        ChannelAlarm[i] = ALM_FLOW_HIGH;
                    }
                    else if (flow < SysParam.SampFlow[i]*(100-SysParam.AlarmThres)/100)
                    {
                        ChannelAlarm[i] = ALM_FLOW_LOW;
                    }
                    else
                    {
                        ChannelAlarm[i] = ALM_FLOW_NOR;
                    }
                }
            }
            else
            {    
                flow = RunStatus.Flow[i];
                if (flow > SysParam.SampFlow[i]*(100+SysParam.AlarmThres)/100)
                {
                    ChannelAlarm[i] = ALM_FLOW_HIGH;
                }
                else if (flow < SysParam.SampFlow[i]*(100-SysParam.AlarmThres)/100)
                {
                    ChannelAlarm[i] = ALM_FLOW_LOW;
                }
                else
                {
                    ChannelAlarm[i] = ALM_FLOW_NOR;
                }
            }
            
        }
        else
        {
            ChannelAlarm[i] = ALM_CH_DISABLE;
        }
        
        if( (ChannelError[i] > 3) && (ChannelAlarm[i] != ALM_CH_DISABLE) )
        {
            ChannelAlarm[i] = ALM_FLOW_ABNOR;
        }
    }
    if (memcmp(HisAlarm,ChannelAlarm,CHANNLE_NUM) != 0)
    {
        // 报警有变化才更新界面
        Delay(200);
        ShowStatus();
        Delay(200);
        AbnorAlaerm();
        StatusColor(false);

        memcpy(HisAlarm,ChannelAlarm,8);
    }
}

// 1秒运行一次
void DevRun()
{
    BYTE i = 0;
    RunStatus.RunTime[8] ++;
    for(i = 0;i<CHANNLE_NUM;i++)
    {
        if(SysParam.Enable &(1<<i))
        {
            RunStatus.RunTime[i] ++;
        }
    }
    
    // 1. 获取流量
    GetFlow();

    // 2. 显示流量和状态
    ShowFlow();

	// 4. 根据模式判断是否结束取样
    RunCheck();
	
    // 3. 检查报警状态  
    if (RunStatus.RunTime[8] > 10)
    {
        // 运行时间大于10秒才检测
        CheckAlarm();
    }
    

//	SendParam();
//	SendChannelParam();
}


//获取电磁阀状态
void GetValve()
{
    BYTE i;
    for(i = 0;i<CHANNLE_NUM;i++)
    {
        if(SysParam.Enable & (1<<i))
        {
            RemChStatus[i] = 1;
           
        }
        else
        {
            RemChStatus[i] = 0; 
            RunStatus.RunTime[i] = 0;
			RealFlow[i].Totol = 0;
        }
        if(SysParam.RemCtlFlag)
        {
            if(Remchenable[i] != RemChStatus[i])
            {
                Remchenable[i] = RemChStatus[i];
                Remchflag[i] = true;
            }
        }
    }
    
        CheckValve();
}

//查询电磁阀状态
void CheckValve()
{
    BYTE i;
    for(i = 0;i<CHANNLE_NUM;i++)
    {
        if(RemChStatus[i])
        {
            switch(i)
            {                            
                case 0 : g_Output[VALVE_0] = 1; break;
                case 1 : g_Output[VALVE_1] = 1; break;
                case 2 : g_Output[VALVE_2] = 1; break;
                case 3 : g_Output[VALVE_3] = 1; break;
                case 4 : g_Output[VALVE_4] = 1; break;
                case 5 : g_Output[VALVE_5] = 1; break;
                case 6 : g_Output[VALVE_6] = 1; break;
                case 7 : g_Output[VALVE_7] = 1; break;  
            }
        }
        else
        {
            switch(i)
            {
                case 0 : g_Output[VALVE_0] = 0; break;
                case 1 : g_Output[VALVE_1] = 0; break;
                case 2 : g_Output[VALVE_2] = 0; break;
                case 3 : g_Output[VALVE_3] = 0; break;
                case 4 : g_Output[VALVE_4] = 0; break;
                case 5 : g_Output[VALVE_5] = 0; break;
                case 6 : g_Output[VALVE_6] = 0; break;
                case 7 : g_Output[VALVE_7] = 0; break;  
            }
        }
    }
}

//关闭电磁阀
void CloseValve()
{
    VALVE0(0);
    VALVE1(0);
    VALVE2(0);
    VALVE3(0);
    VALVE4(0);
    VALVE5(0);
    VALVE6(0);
    VALVE7(0);
}
// 开启气泵
void OpenPump()
{
    g_Output[GAS_BUMP] = 1;
}


// 停止气泵
void ClosePump()
{
    //CloseValve();
    g_Output[GAS_BUMP] = 0;
}

void SendReadFlowCmd(BYTE ch)
{
    ChannelError[ch-1] ++;
    SendReadFlow(ch);    
}

BYTE GetAlarm(BYTE i)
{
    if(ChannelAlarm[i] == ALM_CH_DISABLE)
    {
        return ALM_CH_DISABLE;
    }
    else if (ChannelAlarm[i] ==  ALM_FLOW_ABNOR)
    {
        return ALM_FLOW_ABNOR;
    }
    
    else if (ChannelAlarm[i] ==  ALM_FLOW_HIGH)
    {
        return ALM_FLOW_HIGH;
    }

    else if (ChannelAlarm[i] ==  ALM_FLOW_LOW)
    {
        return ALM_FLOW_LOW;
    }
    else
    {
        return ALM_FLOW_NOR;
    }
}

void SyncModBusDev()
{
    BYTE i;
    memset(&ModBusParam, 0, sizeof(MODBUS_PARAM));
    ModBusParam.Addr = RemRegAddr.SypAddr;
    ModBusParam.ChModeCtl = SysParam.ChModeCtl;
    ModBusParam.RunStatus = RemRunStatus.HostRun;
    for(i = 0;i < 8;i++)    
    {
        ModBusParam.Alarm[i] = GetAlarm(i);
    }
    ModBusParam.Address = 2;
}
void FlowTask()
{
    if (RunStatus.Running)
    {
        if(SendFlowFlag == 1)
        {
            SendFlowFlag = 0;
            SendReadFlowCmd(ChNum++);
        }
        
        if (ChNum>9)
        {
            ChNum = 1;
        }
    }
}

void RemPageCtl()
{
    BYTE i = 0;
    if(SysParam.RemCtlFlag)
    {     
        if(!RemPage)
        {
            RemPage = 1;
            EnterPage(PAGE_REM);
        }
       if(RemRunStatus.RemRun)
       {
            RemStop = 0;
            if(RemStart == 0)
            {
                StartSamp();
                RemStart++;
            }
        }
       else
       {
            RemStart = 0;
            
            if(RemStop == 0)
            {    
                RemRunStatus.HostRun = true;
                g_Output[ALARM_SOUND] = 0; 
                memset(&RunStatus, 0, sizeof(RUN_STATUS));
                StopSamp(false);
                SendParam();
				//SendChannelParam();
                ModeHint(); 
                
                RemStop++;
            }
        }
    }     
    else
    {
        RemStart = 0;
        RemStop = 0;
        for (i=0;i<CHANNLE_NUM;i++)
        {
            Remchflag[i] = false;
        }
        
        if(RemPage)
        {   
            RemPage = 0; 
            RemRunStatus.RemRun = false;
            SysParam.ChModeCtl = 0x00;
            
            ClosePump();
            memset(&RunStatus, 0, sizeof(RUN_STATUS));
            memset(RealFlow,0, sizeof(RealFlow));
            for(i=0;i<CHANNLE_NUM;i++)
            {
                ChannelAlarm[i] = ((SysParam.Enable & (1<<i)) == 0)?0:1;
            }
            g_Output[ALARM_SOUND] = 0; 
            g_Output[LIGHT_BLUE] = 0; 
            g_Output[LIGHT_YELLOW] = 0;
            SendParam();
			//SendChannelParam();
            ModeHint(); 
            EnterPage(PAGE_START);
        } 
    }
}


#if 0
//远程控制界面切换
void RemPageCtl()
{
    switch(PageSwitch)
    {
        case 0:
        {
            if(RunStatus.Running)
            {
                if(SysParam.RemCtlFlag)
                {
                    if(!RemFlag[4])
                    {
                        RemFlag[4] = 1;
                        StartRem[4]++;
                        EnterPage(PAGE_REM);
                       
                    }
                }
                else
                {
                    if(StartRem[4] != 0)
                    {
                        RemFlag[4]  = 0; 
                        SendParam();
                        ModeHint(); 
                        CheckAlarm();
                        EnterPage(PAGE_MAIN);
                        StartRem[4] = 0;
                    }
                }
            }
            else
            {
                 if(SysParam.RemCtlFlag)
                {
                    if(!RemFlag[0])
                    {
                        RemFlag[0] = 1;
                        StartRem[0]++;
                        EnterPage(PAGE_REM);
                       
                    }
                }
                else
                {
                    if(StartRem[0] != 0)
                    {
                        RemFlag[0]  = 0; 
                        SendParam();

                        ModeHint();

                        EnterPage(PAGE_START);
                        StartRem[0] = 0;
                    }
                }
                break;
            }
        }
        
        case 1:
        {
            if(SysParam.RemCtlFlag)
            {
                 if(!RemFlag[1])
                {
                    RemFlag[1] = 1;
                    StartRem[1]++;
                    EnterPage(PAGE_REM);
                    
                 }
            }
            else
            {
                 if(StartRem[1] != 0)
                {
                    
                    RemFlag[1] = 0;
                    if(RunStatus.Running)
                    {
                        SendParam();

                        ModeHint();

                        CheckAlarm();

                        EnterPage(PAGE_MAIN);
                    }
                    else
                    {
                        SendParam();

                        ModeHint();

                        UpdataUI();

                        EnterPage(PAGE_MAIN);

                    }
                    StartRem[1] = 0;
                 }
            }
            break;
        }
        
         case 2:
        {
            if(SysParam.RemCtlFlag)
            {
                 if(!RemFlag[2])
                {
                    RemFlag[2] = 1;
                    StartRem[2]++;
                    EnterPage(PAGE_REM);

                   
                 }
            }
            else
            {
                 if(StartRem[2] != 0)
                {
                    RemFlag[2] = 0;
                    SendParam();

                    ModeHint();

                    EnterPage(PAGE_SET);

                    StartRem[2] = 0;
                 }
            }
            break;
        }
         
         case 3:
        {
            if(SysParam.RemCtlFlag)
            {
                 if(!RemFlag[3])
                {
                    RemFlag[3] = 1;
                    StartRem[3]++;
                    EnterPage(PAGE_REM);

                    
                 }
            }
            else
            {
                 if(StartRem[3] != 0)
                {
                    RemFlag[3] = 0;
                    SendParam();

                    ModeHint();

                    EnterPage(PAGE_TIME);
                    StartRem[3] = 0;
                 }
            }
           break; 
        }  
    }
}
#endif


//远程控制读RemCtlTask从机
void RemCtlTask()
{   
    RemPageCtl();
}

void main(void)
{
    SysInit();
    IoInit();
    PW_MAIN(0);  // 主电源
    LedInit();
    
    RUN_LED(1);
   
    Delay(200);
    
    Timer0Init();
    Delay(200);
    Adc_Init();
    Delay(200);
    
    UART1_config();
    UART2_config();
    UART3_config();
    UART4_config();
    ClearUart1Buf();
    ClearUart2Buf();
    ClearUart3Buf();
    ClearUart4Buf();
    
    // 待CPU稳定了再读参数
    Delay(500);
    ReadParam();
    Delay(200);

    SyncModBusDev();
    Delay(200);
    
    RUN_LED(0);

    #if 0
    while(1)
    {
        RUN_LED(0);
        Delay(800);
        RUN_LED(1);
        Delay(200);
    }
    #endif
    
    
    EA = 1;     //打开总中断

    WDT_CONTR |= (1<<5) |  7;  // 启动开门狗，约8秒
    
    Delay(200);
    InitLcd();
    SysParam.RemCtlFlag = false;
    
    PageSwitch = 0;
    while(1)
    {
        TimerTask();
        HndInput();
      
        Uart1Hnd();
        Uart2Hnd();
        Uart3Hnd(); 
        FlowTask();
        
        Uart4Hnd();
        RemCtlTask(); 
        

    }
}


