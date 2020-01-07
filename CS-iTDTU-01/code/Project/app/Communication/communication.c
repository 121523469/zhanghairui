#include "duart.h"
#include "communication.h"
#include "SrvIODevice.h"
#include "ioctlcmd.h"
#include "string.h"
#include "tim.h"
#include "DLib_Config_FULL.h"
#include "common_interface.h"
#include "stdlib.h"
#include "musart.h"

#define ADMIN_KEY         "123456"                 /**< 管理员密码 */     
#define EXIT              "exit"                   /**< 输入“exit” 退出配置模式 */ 

#define MENU_STATE_EXIT  (-1)                      /**< 退出人机交互 */
#define MENU_STATE_CONTINUE  (0)                       /**< 继续人机交互 */
#define MENU_MAX        100
#define COM_BUFF_SIZE   (256)     
#define COM_GET_SERILATRANS_FLAG()        m_stComItem.u8ComSerialTransFlag
#define COM_SET_SERILATRANS_FLAG(flag) do \
{\
    m_stComItem.u8ComSerialTransFlag = flag;\
}while(0)

static void com_SerialTranscallback(void *p_pvParamSrc, void *p_pvParamLen);
static int16_t com_UserConfig(void);
static int16_t com_AdminConfig(void);
static int16_t com_MenuConfig(uint8_t p_u8MenuIndex);
static void com_ClearBuffer(void);
static int16_t com_VerifyAccount(void);
static int16_t com_SetHardWare(void);
static int16_t com_SetDeviceSN(void);
static int16_t com_SetDeviceID(void);
static int16_t com_SetHeartAlarmTime(void);
static int16_t com_SetAICH1(void);
static int16_t com_SetAICH2(void);

#ifdef IOT_NET_TYPE_NBIOT
static int16_t com_SetDeviceIP(void);
static int16_t com_SetNBMode(void);
#endif
//static int16_t com_SetBaseMagnitic(void);
#ifdef IOT_NET_TYPE_LORAWAN
static int16_t com_SetLWMode(void);
#endif
static int16_t com_ShowSetPara(void);
static int16_t com_SetSerialTrans(void);
static int16_t com_ParaBackup(void);
static uint8_t com_TimeoutHandle(uint8_t p_u8Time);
static int16_t com_ParaFactory(void);
static int16_t com_EXITMenu(void);
static void com_ClearStdin(void);
static void com_ShowIOChannelPara(EnChannelType_t p_enChannelType);

#if 0
/** 通道设置描述和提示 */
StMenuItem_t m_ChannelItem[EN_USERMENU_MAX] = 
{
    {EN_CHMENU_DISABLE,"通道禁用","禁用输入通道"},
    {EN_CHMENU_ENABLE,"通道使能","使能输入通道"},
    {EN_CHMENU_SETALAUP,"设置通道报警上限值","取值范围0-65535"},
    {EN_CHMENU_SETALALOW,"设置通道报警下限值","取值范围0-65535"},
    {EN_CHMENU_SETZERO,"设置通道标定零点值","取值范围0-65535"},
    {EN_CHMENU_SETFS,"设置通道标定满点值","取值范围0-65535"},
};

/** 菜单描述和提示 */
StMenuItem_t m_MenuItem[] = 
{
    {EN_USERMENU_SETDEVICEID, "设置终端ID", "    取值范围 0 - 65535",(pfComMenuIndexFunc)com_SetDeviceID},
#ifdef IOT_NET_TYPE_NBIOT
    {EN_USERMENU_SETIP,"配置远程IP和端口.", "    例如 120.26.207.183:8080",(pfComMenuIndexFunc)com_SetDeviceIP},
    {EN_USERMENU_NBMODE,"设置模块入网方式", "    输入COAP选择COAP入网，输入UDP选择UDP入网",(pfComMenuIndexFunc)com_SetNBMode},
#endif

#ifdef MENU_CHANNEL_OFF
    {EN_USERMENU_SETCH,"配置采集通道", "配置AD通道、PWM通道、开关量使能、通道报警值和标定值"},
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    {EN_USERMENU_SETLWMODE,"设置LoraWan入网方式","    输入OTAA选择OTAA入网，输入ABP选择ABP入网",(pfComMenuIndexFunc)com_SetLWMode},
#endif
    {EN_USERMENU_SETINTERVAL,"配置定时上报间隔","    取值范围1-1440，单位分钟(min)",\
        (pfComMenuIndexFunc)com_SetHeartAlarmTime},
#if 0
    {EN_USERMENU_ALARMVALUE,"配置报警阈值","    当检测到距离小于此值时，触发报警,取值范围10-80，单位厘米(cm)",\
        (pfComMenuIndexFunc)com_SetAlarmValue},
#endif

    {EN_USERMENU_SETBASEMAGNIT,"采集背景磁场","    背景磁场采集中...",\
        (pfComMenuIndexFunc)com_SetBaseMagnitic},
//    {EN_USERMENU_SETLORAWAN_TRANSMODE,"设置lorawan传输方式","    取值范围1--OTAA,0--ABP",\
//        (pfComMenuIndexFunc)com_SetLorawanTransMode},
    {EN_USERMENU_SET_SLEEP_MODE,"设置休眠模式","    取值范围1--使能休眠模式,0--退出休眠模式",\
        (pfComMenuIndexFunc)com_SetSleepMode},
    {EN_USERMENU_SHOWPARA, "查看参数", "    查看所有参数", (pfComMenuIndexFunc)com_ShowSetPara},
    {EN_USERMENU_SERTRANS,"透传模式", "    透传模式可以发送AT指令直接操作无线模组",(pfComMenuIndexFunc)com_SetSerialTrans},
    {EN_USERMENU_FAC,"恢复出厂设置", "    初始化参数为出厂值",(pfComMenuIndexFunc)com_ParaFactory},
    {EN_USERMENU_EXIT,"退出", "    退出人机交互模式",(pfComMenuIndexFunc)com_EXITMenu},
    {EN_ADMINMENU_SETHWVERSION,"设置硬件版本", "    取值范围’A’-‘Z’，代表REV A到REV Z",(pfComMenuIndexFunc)com_SetHardWare},
    {EN_ADMINMENU_SETDEVICESN,"设置终端SN","    取值范围 0 - 4294967295",(pfComMenuIndexFunc)com_SetDeviceSN},
    {EN_ADMINMENU_BACKUP,"备份","    备份出厂参数",(pfComMenuIndexFunc)com_ParaBackup},
};
#endif

/** 菜单描述和提示 */
StMenuItem_t m_MenuItem[] = 
{
    {EN_USERMENU_SETDEVICEID, "Set device ID", \
    "    Range:(1 - 65534); Eg: Input 5 to set the device ID 5",(pfComMenuIndexFunc)com_SetDeviceID},
#ifdef IOT_NET_TYPE_NBIOT
    {EN_USERMENU_SETIP,"Set IP and PORT", "    Eg: Input 120.26.207.183:8080 to set IP 120.26.207.183 and port 8080",(pfComMenuIndexFunc)com_SetDeviceIP},
    {EN_USERMENU_NBMODE,"Set communication mode", "    Eg: Input COAP(UDP/TCP) to select COAP(UDP/TCP) protocol",(pfComMenuIndexFunc)com_SetNBMode},
#endif
    
#ifdef IOT_NET_TYPE_GPRS
    {EN_USERMENU_SETIP,"Set IP and PORT", "    Eg: Input 120.26.207.183:8080 to set IP 120.26.207.183 and port 8080",(pfComMenuIndexFunc)com_SetDeviceIP},
    {EN_USERMENU_GPRSMODE,"Set communication mode", "    Eg: Input TCP(UDP) to select TCP(UDP) mode",(pfComMenuIndexFunc)com_SetGPRSMode},
    {EN_USERMENU_GPRSAPN,"Set APN","    Eg: China Mobile Card is \"CMNET\"，China Unicom IOT is \"UNIM2M.NJM2MAPN\",China Unicom is \"CUNINET\"",\  /*zhongguodianxin?*/
    (pfComMenuIndexFunc)com_SetGPRSAPN},
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    {EN_USERMENU_SETLWMODE,"Set LoraWan work mode","    Eg: Input OTAA(ABP) to select OTAA(ABP) work mode",(pfComMenuIndexFunc)com_SetLWMode},
#endif
    {EN_USERMENU_SETINTERVAL,"Set timely report interval and sample interval",\
    "    Range:(1-1440)min; Eg: Input 5,1 to set timely report interval 5min and sample interval 1min",\
    (pfComMenuIndexFunc)com_SetHeartAlarmTime},
    {EN_USERMENU_SETAICH1,"Set Analog input channel 1",\
    "    Eg: Input 0.5,2.5 enable Analog input channel1 and set alarm low-level 0.5V,set alarm high-level 2.5V,\
Voltage range:0~4.5V,Current range:0~20mA",(pfComMenuIndexFunc)com_SetAICH1},
    {EN_USERMENU_SETAICH2,"Set Analog input channel 2",\
    "    Eg: Input 0.5,2.5 enable Analog input channel1 and set alarm low-level 0.5V,set alarm high-level 2.5V,\r\n\
    Voltage range:0~4.5V,Current range:0~20mA",(pfComMenuIndexFunc)com_SetAICH2},
    
    {EN_USERMENU_SHOWPARA, "View parameters", "    View all parameters", (pfComMenuIndexFunc)com_ShowSetPara},
    {EN_USERMENU_SERTRANS,"Pass-through mode", "    Directly send AT command to module passing through MCU",(pfComMenuIndexFunc)com_SetSerialTrans},
    {EN_USERMENU_FAC,"Factory default", "    Restore factory default configuration parameters",(pfComMenuIndexFunc)com_ParaFactory},
    {EN_USERMENU_EXIT,"Exit", "    Exit the serial configuration mode",(pfComMenuIndexFunc)com_EXITMenu},
    {EN_ADMINMENU_SETHWVERSION,"Set hardware version", \
    "    Range: (’A’-‘Z’)，indicates hardware version from REV A to REV Z; Eg: Enter A to set the hardware version is A.",\
    (pfComMenuIndexFunc)com_SetHardWare},
    {EN_ADMINMENU_SETDEVICESN,"Set device serial number(SN)",\
    "    Range:(0 - 4294967295) Eg: Enter 18060001 to set serial number(SN) is 18060001",\
    (pfComMenuIndexFunc)com_SetDeviceSN},
    {EN_ADMINMENU_BACKUP,"Backup factory configuration parameters","    Backup factory configuration parameters",(pfComMenuIndexFunc)com_ParaBackup},
};

StComMenuItem_t m_stComItem = 
{
    .u8ComEXTIFlag = 0,
    .u8ComSerialTransFlag = 0,
    .u8ComMenuIndex = EN_USERMENU_SETDEVICEID,
    .u8ComConfigFlag = 1,
    .u8ComSecCount = 0,
    .u8nbPSMstate = 0,
};

static char m_u8ComMenuBuf[COM_BUFF_SIZE];
static uint32_t m_u32ComSerTrsRcvLen = 0;
uint32_t u32arg = 0;

/*************************************************************************************************/
/** @brief Communication Callback
*/
/*************************************************************************************************/
void Com_Init(void)
{
    /* Open DUART Device */
    SrvIODevice_open(&stDebugUartDevice,EN_ID_DUART,IO_READ | IO_WRITE);
    com_ClearBuffer();
}

static void com_ClearStdin(void)
{
    char ch = 0;
    scanf("%c",&ch);
}

/*************************************************************************************************/
/** @brief Communication Timeout Handle.
*/
/*************************************************************************************************/
static uint8_t com_TimeoutHandle(uint8_t p_u8Time)
{
    int16_t u16waitnum = 0;

    /*if received ack or timeout,exit loop*/
    while(!((u16waitnum > (p_u8Time * 1000)) ||COM_GET_SERILATRANS_FLAG()))
    {
         u16waitnum++;
         HAL_Delay(1);
    }
    COM_SET_SERILATRANS_FLAG(0);
    if (u16waitnum > (p_u8Time * 1000))
    {
        return EN_MENU_STATE_TIMEOUT;
    }
    return EN_MENU_STATE_ACK_OK;
}

/*************************************************************************************************/
/** @brief Communication Serial-Trans Callback.
*/
/*************************************************************************************************/
static void com_SerialTranscallback(void *p_pvParamSrc, void *p_pvParamLen)
{
  /* copy uart buffer data to module driver rcv buff */
    memset(m_u8ComMenuBuf,0,sizeof(m_u8ComMenuBuf));
    memcpy(m_u8ComMenuBuf, (uint8_t*)p_pvParamSrc, *(uint32_t *)p_pvParamLen);
    m_u32ComSerTrsRcvLen = *(uint32_t *)p_pvParamLen;

    /* Flag=1 indecated one frame rev finished */
    COM_SET_SERILATRANS_FLAG(1);
    printf("%s\r\n",m_u8ComMenuBuf);
}

/** @brief Clear Communication Buffer.
*/
static void com_ClearBuffer(void)
{
    memset(m_u8ComMenuBuf, 0, sizeof(m_u8ComMenuBuf));
}

/** @brief 交互处理
*/
void Com_process(void)
{
    uint32_t u32arg = 0;
    int16_t ret = 0;
    
    //SrvIODevice_open(&stDebugUartDevice, EN_ID_DUART, IO_WRITE | IO_READ);
    ret = SrvIODevice_ioctl(&stDebugUartDevice, EN_CMD_GET_DUART_MENU_STATE, (uint32_t)(&u32arg));
    if(0 != ret)
    {
        printf("Com_process get menu state err,ret= %d!\r\n", ret);
        return;
    }

    if(EN_DUART_MENU_ENTER == u32arg)
    {
        com_ClearBuffer();
        if(com_VerifyAccount() == MENU_STATE_EXIT)
        {
            return;
        }
    }
}

/** @brief 验证账户，用户或者是管理员，或者退出交互,否则进入菜单.
*/
static int16_t com_VerifyAccount(void)
{
    printf("Please input your username->");
    while(1)
    {
        com_ClearBuffer();
        scanf("%s",m_u8ComMenuBuf);

        if((strstr(m_u8ComMenuBuf,"user") != NULL) && (strlen(m_u8ComMenuBuf) < 5))
        {
            if(com_UserConfig() == MENU_STATE_EXIT)
            {
                break;
            }
        }
        else if((strstr(m_u8ComMenuBuf,"admin") != NULL) && (strlen(m_u8ComMenuBuf) < 6))
        {
            if(com_AdminConfig() == MENU_STATE_EXIT)
            {
                break;
            }
        }
        else if(strstr(m_u8ComMenuBuf,EXIT) != NULL)
        {
            break;
        }
        else
        {
            printf("Incorrect username,please input again->");
        }
    }

    printf("Serial configuration mode exits!\r\n");  
    SrvIODevice_ioctl(&stDebugUartDevice, EN_CMD_EXIT_MENU, u32arg);

    /*Set duart menu state*/
    return MENU_STATE_EXIT;
}


/** @brief 配置终端硬件版本
*/
static int16_t com_SetHardWare(void)
{
    uint8_t u8HWVersion = 0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please input the hardware version->");
    scanf("%1s",&u8HWVersion);
    com_ClearStdin();
    if((u8HWVersion < 'A') || (u8HWVersion > 'Z'))
    {
        printf("    Incorrect hardware version,please input again!\r\n");
    }
    else
    {
        stSysParaData.stFactorySetData.u8HWVersion = u8HWVersion;
        printf("    Hardware version is: %c\r\n",(char)stSysParaData.stFactorySetData.u8HWVersion);
        SaveFactorySetData(stSysParaData.stFactorySetData);
    }
    
    return MENU_STATE_CONTINUE;
}

#ifdef IOT_NET_TYPE_LORAWAN
static int16_t com_SetLWMode(void)
{
    char cRcvBuf[5];
    memset(cRcvBuf,0,sizeof(cRcvBuf));
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please input OTAA or ABP to select LoraWan work mode->");
    scanf("%s",cRcvBuf);
    com_ClearStdin();
    if((strstr(cRcvBuf,"OTAA") != NULL) && strlen(cRcvBuf) < 5)
    {
        stSysParaData.stUserSetData.stLoraWanCfg.EnTransMode = EN_TRANS_MODE_OTAA;
        SaveUserSetData(stSysParaData.stUserSetData);
        printf("    LoraWan work mode is %s!\r\n",cRcvBuf);
    }
    else if((strstr(cRcvBuf,"ABP") != NULL) && strlen(cRcvBuf) < 4)
    {
        stSysParaData.stUserSetData.stLoraWanCfg.EnTransMode = EN_TRANS_MODE_ABP;
        printf("    LoraWan work mode %s!\r\n",cRcvBuf);
        SaveUserSetData(stSysParaData.stUserSetData);
    }
    else
    {
        printf("    Lorawan mode error,please input again!\r\n");
    }
    return MENU_STATE_CONTINUE;
}
#endif
/** @brief 备份数据
*/
static int16_t com_ParaBackup(void)
{
    memcpy(&(stSysParaData.stFactorySetData.stDefaultUserData),\
           &(stSysParaData.stUserSetData),
           sizeof((stSysParaData.stUserSetData)));
    SaveFactorySetData(stSysParaData.stFactorySetData);
    printf("    Backup user configuration parameters succeeds!\r\n");
    return MENU_STATE_CONTINUE;
}

/** @brief 设置终端序列号
*/
static int16_t com_SetDeviceSN(void)
{
    uint32_t Sn = 0;
    com_ClearBuffer();
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please input the serial number(SN)->");
    scanf("%s",m_u8ComMenuBuf);
    com_ClearStdin();
    if(strlen(m_u8ComMenuBuf) <= 10)
    {
        Sn = atoi(m_u8ComMenuBuf);
    }
    else
    {
        printf("    Incorrect serial number(SN),please input again!\r\n");
        return MENU_STATE_CONTINUE;
    }
    if((Sn < 1) || (0xFFFFFFFF == Sn))
    {
        printf("Invalid SN!\r\n");
        return MENU_STATE_CONTINUE;
    }

    stSysParaData.stFactorySetData.u32DeviceSN = Sn;
    printf("    Device serial number(SN) is : %u\r\n",stSysParaData.stFactorySetData.u32DeviceSN);
    SaveFactorySetData(stSysParaData.stFactorySetData);
    
    return MENU_STATE_CONTINUE;
}

/** @brief 配置终端ID
*/
static int16_t com_SetDeviceID(void)
{
    uint16_t u16DeviceID = 0;
    
    com_ClearBuffer();
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please input the device ID->");
    scanf("%s",m_u8ComMenuBuf);
    com_ClearStdin();
    if(strlen(m_u8ComMenuBuf) <= 5)
    {
        u16DeviceID = atoi(m_u8ComMenuBuf);
    }
    else
    {
        printf("    Incorrect device ID!\r\n");
        return MENU_STATE_CONTINUE;
    }
    if((u16DeviceID == 0) || (u16DeviceID == 0xFFFF))
    {
        printf("    Invaild ID!\r\n");
        return MENU_STATE_CONTINUE;
    }
    stSysParaData.stUserSetData.u16DeviceID = u16DeviceID;
    printf("    Device ID is : %hu\r\n",stSysParaData.stUserSetData.u16DeviceID);
    SaveUserSetData(stSysParaData.stUserSetData);
    
    return MENU_STATE_CONTINUE;
}

#ifdef IOT_NET_TYPE_NBIOT
/** @brief 配置终端IP
*/
static int16_t com_SetDeviceIP(void)
{
    UNU32U8_t IP_PORT;
    memset(&IP_PORT,0,sizeof(IP_PORT));
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please input IP and PORT->");
    scanf("%hhu.%hhu.%hhu.%hhu:%hu",&IP_PORT.u8[0],&IP_PORT.u8[1],&IP_PORT.u8[2],&IP_PORT.u8[3],&stSysParaData.stUserSetData.stNbiotCfg.u16Port);
    com_ClearStdin();
    printf("    IP and Port is : %hhu.%hhu.%hhu.%hhu:%hu\r\n",IP_PORT.u8[0],IP_PORT.u8[1],IP_PORT.u8[2],IP_PORT.u8[3],\
    stSysParaData.stUserSetData.stNbiotCfg.u16Port);
    stSysParaData.stUserSetData.stNbiotCfg.u32IpAddr = IP_PORT.u32;
    SaveUserSetData(stSysParaData.stUserSetData);
    
    return MENU_STATE_CONTINUE;
}

static int16_t com_SetNBMode(void)
{
    char cRcvBuf[5];
    memset(cRcvBuf,0,sizeof(cRcvBuf));
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please type COAP,UDP or TCP to select communication mode->");
    scanf("%s",cRcvBuf);
    com_ClearStdin();
    if((strstr(cRcvBuf,"COAP") != NULL) && strlen(cRcvBuf) < 5)
    {
        stSysParaData.stUserSetData.stNbiotCfg.EnTransMode = EN_TRANS_MODE_COAP;
        SaveUserSetData(stSysParaData.stUserSetData);
        printf("    Communication mode is %s!\r\n",cRcvBuf);
    }
    else if((strstr(cRcvBuf,"UDP") != NULL) && strlen(cRcvBuf) < 4)
    {
        stSysParaData.stUserSetData.stNbiotCfg.EnTransMode = EN_TRANS_MODE_UDP;
        printf("    Communication mode is %s!\r\n",cRcvBuf);
        SaveUserSetData(stSysParaData.stUserSetData);
    }
    else if((strstr(cRcvBuf,"TCP") != NULL) && strlen(cRcvBuf) < 4)
    {
        stSysParaData.stUserSetData.stNbiotCfg.EnTransMode = EN_TRANS_MODE_TCP;
        printf("    Communication mode is %s!\r\n",cRcvBuf);
        SaveUserSetData(stSysParaData.stUserSetData);
    }

    else
    {
        printf("    Communication mode error,please input again!\r\n");
    }
    return MENU_STATE_CONTINUE;
}
#endif

/** @brief 设置定时上报时间，报警上报时间，采样间隔.
*/
static int16_t com_SetHeartAlarmTime(void)
{
     uint16_t u16HeartBeatIntevaltemp = 0;
    uint16_t u16SampleInterval = 0;

    com_ClearBuffer();
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please input the timely report/sample interval->");
    scanf("%hu,%hu",&u16HeartBeatIntevaltemp,&u16SampleInterval);
    com_ClearStdin();

    if((u16HeartBeatIntevaltemp == 0) || (u16HeartBeatIntevaltemp > 1440))
    {
        printf("    Input error,please input again!\r\n");
        return MENU_STATE_CONTINUE;
    }

    if((u16SampleInterval == 0) || (u16SampleInterval > 1440))
    {
        printf("    Input error,please input again!\r\n");
        return MENU_STATE_CONTINUE;
    }
    
    stSysParaData.stUserSetData.u16HeartBeatInteval = u16HeartBeatIntevaltemp;
    stSysParaData.stUserSetData.u16SampleInteval = u16SampleInterval;

    printf("    Timely report interval is:%humin, sample interval is %humin\r\n",\
        stSysParaData.stUserSetData.u16HeartBeatInteval,stSysParaData.stUserSetData.u16SampleInteval);

    SaveUserSetData(stSysParaData.stUserSetData);
    
    return MENU_STATE_CONTINUE;
}

static int16_t com_SetAICH1(void)
{
    float fAICHlowlevel = 0.0;
    float fAICHHighlevel = 0.0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please set analog channel->");
    scanf("%f,%f",&fAICHlowlevel,&fAICHHighlevel);
    com_ClearStdin();

    //if(EN_AI_TYPE_VOL == IOChannel.GetAnalogType(EN_TYPE_AI1))
    if(EN_AI_TYPE_VOL == (stSysParaData.u8AnalogType & (1 << EN_TYPE_AI1)))
    {
        if((fAICHlowlevel > fAICHHighlevel) || (fAICHHighlevel > AICH_VOLTAGE_MAX))
        {
            printf("    Input error,please input again\r\n");
            return MENU_STATE_CONTINUE;
        }
    }
    else
    {
        if((fAICHlowlevel > fAICHHighlevel) || (fAICHHighlevel > AICH_CURRENT_MAX))
        {
            printf("    Input error,please input again\r\n");
            return MENU_STATE_CONTINUE;
        }
    }

    stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel =(uint16_t)(fAICHlowlevel * 1000);
    stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel =(uint16_t)(fAICHHighlevel * 1000);

    com_ShowIOChannelPara(EN_TYPE_AI1);

    SaveUserSetData(stSysParaData.stUserSetData);
    
    return MENU_STATE_CONTINUE;
}
static int16_t com_SetAICH2(void)
{
    float fAICHlowlevel = 0.0;
    float fAICHHighlevel = 0.0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please set analog channel->");
    scanf("%f,%f",&fAICHlowlevel,&fAICHHighlevel);
    com_ClearStdin();

    //if(EN_AI_TYPE_VOL == IOChannel.GetAnalogType(EN_TYPE_AI1))
    if(EN_AI_TYPE_VOL == (stSysParaData.u8AnalogType & (1 << EN_TYPE_AI2)))
    {
        if((fAICHlowlevel > fAICHHighlevel) || (fAICHHighlevel > AICH_VOLTAGE_MAX))
        {
            printf("    Input error,please input again\r\n");
            return MENU_STATE_CONTINUE;
        }
    }
    else
    {
        if((fAICHlowlevel > fAICHHighlevel) || (fAICHHighlevel > AICH_CURRENT_MAX))
        {
            printf("    Input error,please input again\r\n");
            return MENU_STATE_CONTINUE;
        }
    }

    stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel =(uint16_t)(fAICHlowlevel * 1000);
    stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel =(uint16_t)(fAICHHighlevel * 1000);
    
    com_ShowIOChannelPara(EN_TYPE_AI2);

    SaveUserSetData(stSysParaData.stUserSetData);
    
    return MENU_STATE_CONTINUE;
}

#if 0
static int16_t com_SetDICH1(void)
{
    uint8_t u8Status = 0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please set digital input channel 1->");
    scanf("%hhu",&u8Status);
    com_ClearStdin();
    
    if(0 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_DI1,CHANNEL_DISABLE);
    }
    else if(1 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_DI1,CHANNEL_ENABLE);
    }
    else
    {
        printf("Input error,please input again\r\n");
        return MENU_STATE_CONTINUE;
    }

    com_ShowIOChannelPara(EN_TYPE_DI1);
    
    SaveUserSetData(stSysParaData.stUserSetData);
    
    return MENU_STATE_CONTINUE;
}

static int16_t com_SetDICH2(void)
{
    uint8_t u8Status = 0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please set digital input channel 2->");
    scanf("%hhu",&u8Status);
    com_ClearStdin();
    
    if(0 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_DI2,CHANNEL_DISABLE);
    }
    else if(1 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_DI2,CHANNEL_ENABLE);
    }
    else
    {
        printf("Input error,please input again\r\n");
        return MENU_STATE_CONTINUE;
    }

    com_ShowIOChannelPara(EN_TYPE_DI2);
    
    SaveUserSetData(stSysParaData.stUserSetData);

    return MENU_STATE_CONTINUE;
}

static int16_t com_SetDOCH1(void)
{
    uint8_t u8Status = 0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please set digital output channel->");
    scanf("%hhu",&u8Status);
    com_ClearStdin();
    
    if(0 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_DO1,CHANNEL_DISABLE);
    }
    else if(1 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_DO1,CHANNEL_ENABLE);
    }
    else
    {
        printf("Input error,please input again\r\n");
        return MENU_STATE_CONTINUE;
    }

    com_ShowIOChannelPara(EN_TYPE_DO1);
    
    SaveUserSetData(stSysParaData.stUserSetData);
    return MENU_STATE_CONTINUE;
}

static int16_t com_SetRS485(void)
{
    uint8_t u8Status = 0;
    
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please set RS485 channel->");
    scanf("%hhu",&u8Status);
    com_ClearStdin();
    
    if(0 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_RS485,CHANNEL_DISABLE);
    }
    else if(1 == u8Status)
    {
        IOChannel.ChangeState(EN_TYPE_RS485,CHANNEL_ENABLE);
    }
    else
    {
        printf("Input error,please input again\r\n");
        return MENU_STATE_CONTINUE;
    }

    com_ShowIOChannelPara(EN_TYPE_RS485);
    
    SaveUserSetData(stSysParaData.stUserSetData);
    return MENU_STATE_CONTINUE;
}
#endif

static void com_ShowIOChannelPara(EnChannelType_t p_enChannelType)
{
    switch(p_enChannelType)
    {
        case EN_TYPE_AI1:
        {
            //if(EN_AI_TYPE_VOL == IOChannel.GetAnalogType(EN_TYPE_AI1))
            if(EN_AI_TYPE_VOL == (stSysParaData.u8AnalogType & (1 << EN_TYPE_AI1)))
            {
                printf("    Analog input channel 1 alarm threshold:%.2fV~%.2fV\r\n",\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel / 1000.0),\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel / 1000.0));
            }
            else
            {
                printf("    Analog input channel 1 alarm threshold:%.2fmA~%.2fmA\r\n",\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel / 1000.0),\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel / 1000.0));
            }
            break;
        }
        case EN_TYPE_AI2:
        {
            if(EN_AI_TYPE_VOL == (stSysParaData.u8AnalogType & (1 << EN_TYPE_AI2)))
            {
                printf("    Analog input channel 2 alarm threshold:%.2fV~%.2fV\r\n",\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel / 1000.0),\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel / 1000.0));
            }
            else
            {
                printf("    Analog input channel 2 alarm threshold:%.2fmA~%.2fmA\r\n",\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel / 1000.0),\
                (float)(stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel / 1000.0));
            }
            break;
        }
        default:{printf("Invalid IO channel!\r\n");break;}
    }
}


/** @brief 打印已设置的参数.
*/
static int16_t com_ShowSetPara(void)
{
    EnChannelType_t i;
#ifdef IOT_NET_TYPE_NBIOT
    UNU32U8_t IP_PORT;
#endif
    ReadUserSetData(&stSysParaData.stUserSetData);
    ReadFactorySetData(&stSysParaData.stFactorySetData);
    printf("    Hardware version is : %c\r\n",(char)stSysParaData.stFactorySetData.u8HWVersion);
    printf("    Device serial number(SN) is : %u\r\n",stSysParaData.stFactorySetData.u32DeviceSN);
    printf("    Device ID is : %hu\r\n",stSysParaData.stUserSetData.u16DeviceID);
#ifdef IOT_NET_TYPE_NBIOT
    IP_PORT.u32 = stSysParaData.stUserSetData.stNbiotCfg.u32IpAddr;
    printf("    IP and PORT is : %hhu.%hhu.%hhu.%hhu:%hu\r\n",\
        IP_PORT.u8[0],IP_PORT.u8[1],IP_PORT.u8[2],IP_PORT.u8[3],\
        stSysParaData.stUserSetData.stNbiotCfg.u16Port);

    if(stSysParaData.stUserSetData.stNbiotCfg.EnTransMode == EN_TRANS_MODE_COAP)
    {
        printf("    NBIOT communication mode is : COAP\r\n");
    }
    else if(stSysParaData.stUserSetData.stNbiotCfg.EnTransMode == EN_TRANS_MODE_UDP)
    {
        printf("    NBIOT communication mode is : UDP\r\n");
    }
    else if(stSysParaData.stUserSetData.stNbiotCfg.EnTransMode == EN_TRANS_MODE_TCP)
    {
        printf("    NBIOT communication mode is : TCP\r\n");
    }
    else
    {
        printf("    NBIOT communication mode err,pls set!\r\n");
    }

#endif
#ifdef IOT_NET_TYPE_LORAWAN
    if(stSysParaData.stUserSetData.stLoraWanCfg.EnTransMode == EN_TRANS_MODE_OTAA)
    {
        printf("    LoraWan work mode is : OTAA\r\n");
    }
    else if(stSysParaData.stUserSetData.stLoraWanCfg.EnTransMode == EN_TRANS_MODE_ABP)
    {
        printf("    LoraWan work mode is : ABP\r\n");
    }
#endif

    printf("    Timely report and sample interval is: %humin,%humin\r\n",\
    stSysParaData.stUserSetData.u16HeartBeatInteval,stSysParaData.stUserSetData.u16SampleInteval);
    
    com_ShowIOChannelPara(EN_TYPE_AI1);
    com_ShowIOChannelPara(EN_TYPE_AI2);
    
    return MENU_STATE_CONTINUE; 
}

/** @brief 恢复出厂设置.
*/
static int16_t com_ParaFactory(void)
{
    int16_t ret = -1;

    ret = RestorFactoryPara();
    if(0 != ret)
    {
        printf("Restore Factory para err,ret is %d!\r\n", ret);
        return MENU_STATE_CONTINUE;
    }

    printf("Restore Factory para ok,system is rebooting...\r\n");
    NVIC_SystemReset();

    return MENU_STATE_CONTINUE;
}

/** @brief 退出菜单.
*/
static int16_t com_EXITMenu(void)
{
    return MENU_STATE_EXIT;
}

/** @brief 透传模式.
*/
static int16_t com_SetSerialTrans(void)
{
    uint8_t u8eof[2] = {0x0D,0x0A};
    char *pu8_RcvBuffer = NULL;
    uint32_t i = 0;
    uint8_t ret = 0;
    uint32_t MuartState = 1;

#ifdef IOT_NET_TYPE_LORAWAN
    SrvIODevice_open(&stLORAWAN, EN_ID_LORAWAN, IO_READ | IO_WRITE);
    ret = SrvIODevice_ioctl(&stLORAWAN, EN_CMD_SET_MUART_STATE, (uint32_t)&MuartState);
    if (0 != ret)
    {
        printf("Pass-through mode enable moduler uart fail!\r\n");
    }
#endif
#ifdef IOT_NET_TYPE_NBIOT
    SrvIODevice_open(&stNBIOT, EN_ID_NBIOT, IO_READ | IO_WRITE);
    ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_SET_MUART_STATE, (uint32_t)&MuartState);
    if (0 != ret)
    {
        printf("Pass-through mode enable moduler uart fail!\r\n");
    }

    SrvIODevice_ioctl(&stNBIOT, EN_CMD_WAKEUP,0);

    SrvIODevice_ioctl(&stNBIOT, EN_CMD_DISABLE_PSM,0);
#endif

    com_ClearBuffer();
    printf("%s\r\n",m_MenuItem[m_stComItem.u8ComMenuIndex].pcHelp);
    printf("Please intput AT command->");
    SrvIODevice_ioctl(&stDebugUartDevice,EN_CMD_REGSERCALLBACK,(uint32_t)com_SerialTranscallback);
    scanf("%s",m_u8ComMenuBuf);
    com_ClearStdin();
    pu8_RcvBuffer = strstr(m_u8ComMenuBuf,"exit");
    while(pu8_RcvBuffer == NULL)
    {
        /* 发送数据 */
        MuartSend((uint8_t *)m_u8ComMenuBuf, strlen(m_u8ComMenuBuf));
        MuartSend(u8eof, sizeof(u8eof));
        com_ClearBuffer();
        scanf("%s",m_u8ComMenuBuf);
        com_ClearStdin();
        pu8_RcvBuffer = strstr(m_u8ComMenuBuf,EXIT);
    }

#ifdef IOT_NET_TYPE_LORAWAN
        SrvIODevice_open(&stLORAWAN, EN_ID_LORAWAN, IO_READ | IO_WRITE);
        ret = SrvIODevice_ioctl(&stLORAWAN, EN_CMD_ENTER_LOWPOWER, 0);
        if(0 != ret)
        {
            printf("Pass-through enable lorawan moduler lowpower mode err!\r\n");
        }
#endif
#ifdef IOT_NET_TYPE_NBIOT
    SrvIODevice_open(&stNBIOT, EN_ID_NBIOT, IO_READ | IO_WRITE);
    SrvIODevice_ioctl(&stNBIOT, EN_CMD_ENABLE_PSM,0);
#endif
    /* Open Module Device*/
    return MENU_STATE_CONTINUE;
}

/** @brief 人机交互菜单
*/
static int16_t com_MenuConfig(uint8_t p_u8MenuIndex)
{
  uint8_t i = 0;
  uint8_t u8return = 0;
  while(1)
  {
    com_ClearBuffer();
    m_stComItem.u8ComMenuIndex = EN_USERMENU_SETDEVICEID;
    for( i = m_stComItem.u8ComMenuIndex; i < p_u8MenuIndex; i++)
    {
      printf("%d:%s\r\n",m_MenuItem[i].s8MenuID,m_MenuItem[i].pcDes);
      printf("\r\n");
    }
    printf("Please select menu->");
    u8return = scanf("%hhu",&m_stComItem.u8ComMenuIndex);
    com_ClearStdin();
    if((u8return != 0) && (m_stComItem.u8ComMenuIndex < p_u8MenuIndex))
    {
        if(m_MenuItem[m_stComItem.u8ComMenuIndex].fpMenuIndex() == MENU_STATE_EXIT)
        {
            return MENU_STATE_EXIT;
        }
    }
    else
    {
        printf("    Error input,please input again\r\n");
        com_ClearStdin();
    }
  }
}

/** @brief 管理员配置
*/
static int16_t com_AdminConfig()
{
  uint8_t u8IndexMAX = EN_ADMINMENU_MAX;
    printf("Please input your password->");
    while(m_stComItem.u8ComConfigFlag)
    {
        com_ClearBuffer();
        scanf("%s",m_u8ComMenuBuf);
        com_ClearStdin();

        if((strstr(m_u8ComMenuBuf,ADMIN_KEY) != NULL) && (strlen(m_u8ComMenuBuf) < 7) )
        {
            if(com_MenuConfig(u8IndexMAX) == MENU_STATE_EXIT)
            {
                break;
            }
        }
        else if(strstr(m_u8ComMenuBuf,EXIT) != NULL)
        {
            break;
        }
        else
        {
          printf("Password error,please input again->");
        }
    }

    return MENU_STATE_EXIT;
}

/** @brief 用户配置
*/
static int16_t com_UserConfig(void)
{
    uint8_t u8IndexMAX = EN_USERMENU_MAX;
    if(com_MenuConfig(u8IndexMAX) == MENU_STATE_EXIT)
    {
        return MENU_STATE_EXIT;
    }
    return MENU_STATE_CONTINUE;
}

