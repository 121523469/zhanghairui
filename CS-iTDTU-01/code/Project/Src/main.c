/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "lptim.h"
#include "SrvErrorNo.h"
#include "ADCDriver.h"
#include "string.h"
#include "common_interface.h"
#include "low_power_ctl.h"
#include "nbiot.h"
#include "lorawan.h"
#include "duart.h"
#include "communication.h"
#include "SVNVersion.h"
#include "battery.h"
#include "led.h"
#include "IOChannelManage.h"
#include "gpio.h"
#define SYS_ADC_SAMLPE_CYCLE    (120)/*every 2min sample once*/
/* Private variables ---------------------------------------------------------*/

void SystemClock_Config(void);
#define STR1(R)    #R
#define CONVERT_TO_STR(R)     STR1(R)

uint32_t g_u32SysTimerCounter = 0;/*systimer unit second*/
#define MINUTE_COUNTER    (3)
#define NET_ABNORMAL_UPLINK_TIME      (60)
#define NET_ABNORMAL_UPLINK_COUNTER     (3 * NET_ABNORMAL_UPLINK_TIME)

void FeedDog(void)
{
    IWDG->KR = 0xAAAA;
}

void ResetMode()
{
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) == SET)
    {
        printf("Reset mode is power-on reset!\r\n");
    }
    else if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == SET)
    {
        printf("Reset mode is IWDG reset!\r\n");
    }
    else
    {
        printf("Reset mode is Software reset!\r\n");
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
}


void UpdateBatLeftPer(uint8_t u8Per)
{
    if (u8Per < stSysParaData.u8BatteryLeftPercent)
    {
        stSysParaData.u8BatteryLeftPercent = u8Per;
    }
}


int16_t SysParaInit(void)
{
    int16_t ret = -1;
    memset(&stSysParaData, 0, sizeof(stSysParaData));

    /* 读出用户配置参数到 User EEPROM Storage */
    ret = ReadUserSetData(&stSysParaData.stUserSetData);
    if(ERR_OK != ret)
    {
        printf("SysParaInit read User para fail,ret = %d!\r\n", ret);
        return ret;
    }

    /* 读出出厂配置参数到 Factory EEPROM Storage */

    ret = ReadFactorySetData(&stSysParaData.stFactorySetData);
    if(ERR_OK != ret)
    {
        printf("SysParaInit read factory para fail,ret = %d!\r\n", ret);
        return ret;
    }
    
    stSysParaData.u8DeviceType = DEVICE_TYPE;

    stSysParaData.u8AnalogType = IOChannel.GetAnalogType();
    
    if (0x0000 == stSysParaData.stUserSetData.u16HeartBeatInteval)
    {
        stSysParaData.stUserSetData.u16HeartBeatInteval = 1440;/*默认定时上报时间间隔60min*/
    }
    if (0x0000 == stSysParaData.stUserSetData.u16SampleInteval)
    {
        stSysParaData.stUserSetData.u16SampleInteval = 5;/*默认定时上报时间间隔60min*/
    }
    if (0x0000 == stSysParaData.stUserSetData.u16DeviceID)
    {
        stSysParaData.stUserSetData.u16DeviceID = 1;/*默认device id 1*/
    }
    if (0x00 == stSysParaData.stFactorySetData.u8HWVersion)
    {
        stSysParaData.stFactorySetData.u8HWVersion = 'A';/*默认device id 1*/
    }
#ifdef IOT_NET_TYPE_LORAWAN
    if(0x0000 == stSysParaData.stUserSetData.stLoraWanCfg.EnTransMode)
    {
        stSysParaData.stUserSetData.stLoraWanCfg.EnTransMode = EN_TRANS_MODE_ABP;
    }
#endif

#ifdef IOT_NET_TYPE_NBIOT
    if(0x0000 == stSysParaData.stUserSetData.stNbiotCfg.EnTransMode)
    {
        stSysParaData.stUserSetData.stNbiotCfg.EnTransMode = EN_TRANS_MODE_COAP;
    }
#endif
#if 0
    if(0x0000 == stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel)
    {
        stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel = 1500; /*1.5V*/
    }
    if(0x0000 == stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel)
    {
        stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel = 3500; /*3.5V*/
    }
    
    if(0x0000 == stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel)
    {
        stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel = 1500; /*1.5V*/
    }
    if(0x0000 == stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel)
    {
        stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel = 3500; /*3.5V*/
    }
#endif
    stSysParaData.u8BatteryLeftPercent = 100;
    SaveUserSetData(stSysParaData.stUserSetData);
    SaveFactorySetData(stSysParaData.stFactorySetData);
    return 0;
}


void IWDG_Init(uint8_t p_u8Prescaler, uint16_t p_u16ReloadValue)
{
    IWDG->KR = 0x5555;
    IWDG->PR = p_u8Prescaler;
    IWDG->RLR = p_u16ReloadValue;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
}

int16_t SysAdcInit(void)
{
    int16_t ret = -1;

    ret = SrvIODevice_open(&stADC, EN_ID_ADC, IO_READ | IO_WRITE);
    if ((ERR_OK != ret) && (ERR_BEYOND_MAX != ret))
    {
        printf("SrvIODevice_open open stADC fail,ret=%d!\r\n", ret);
    }
    return ret;
}

void TimelyReportProc(void)
{
    static int8_t FirstRun = 1;
    static uint32_t LastReportCounter = 0;
    static uint16_t IgnoreTimelyReport = 0;
    int16_t Ret = -1;
    uint32_t NetState = 0;

    if (1 == FirstRun)
    {
        printf("First Send Pro_SendRegularUpload!\r\n");
        Pro_SendRegularUpload();
        FirstRun = 0;
        return;
    }

    if(LastReportCounter == stSysParaData.u32TimeCounter)
    {
        return;
    }

    if(0 == stSysParaData.u32TimeCounter % \
        (stSysParaData.stUserSetData.u16HeartBeatInteval * MINUTE_COUNTER))
    {
        printf("Send regular report msg!\r\n");

        if(NET_ABNORMAL_UPLINK_TIME > stSysParaData.stUserSetData.u16HeartBeatInteval)
        {
#ifdef IOT_NET_TYPE_NBIOT
            Ret = SrvIODevice_open(&stNBIOT, EN_ID_NBIOT, IO_READ | IO_WRITE);
            if((0 != Ret) && (ERR_BEYOND_MAX != Ret))
            {
                printf("TimelyReportProc open stNBIOT fail,Ret=%d!\r\n", Ret);
                return;
            }

            Ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_GET_NBIOT_NET_STATE, (uint32_t)&NetState);
            if (0 != Ret)
            {
                printf("TimelyReportProc ioctl get net state fail,Ret=%d!\r\n", Ret);
                return;
            }
#endif
#ifdef IOT_NET_TYPE_LORAWAN
            Ret = SrvIODevice_open(&stLORAWAN, EN_ID_LORAWAN, IO_READ | IO_WRITE);
            if((0 != Ret) && (ERR_BEYOND_MAX != Ret))
            {
                printf("TimelyReportProc open stLORAWAN fail,Ret=%d!\r\n", Ret);
                return;
            }
            
            Ret = SrvIODevice_ioctl(&stLORAWAN, EN_CMD_GET_LORAWAN_NET_STATE, (uint32_t)&NetState);
            if (0 != Ret)
            {
                printf("TimelyReportProc ioctl get net state fail,Ret=%d!\r\n", Ret);
                return;
            }
#endif
            /*上次入网失败*/
            if(0 == NetState)
            {
                printf("TimelyReportProc last search network fail!\r\n");
                printf("stSysParaData.u32TimeCounter = %d,IgnoreTimelyReport = %d!\r\n",\
                    stSysParaData.u32TimeCounter, IgnoreTimelyReport);

                IgnoreTimelyReport++;
                if(IgnoreTimelyReport < NET_ABNORMAL_UPLINK_COUNTER\
                    / (stSysParaData.stUserSetData.u16HeartBeatInteval * MINUTE_COUNTER))
                {
                    printf("Current net signal abnormal,ignore timely report!\r\n");
                    LastReportCounter = stSysParaData.u32TimeCounter;
                    return;
                }
                else
                {
                    IgnoreTimelyReport = 0;
                }
            }
            else
            {
                IgnoreTimelyReport = 0;
            }
        }

        Pro_SendRegularUpload();
        LastReportCounter = stSysParaData.u32TimeCounter;
    }
}

void AlarmReportProc(void)
{
    //uint16_t Ret = -1;
    //uint16_t u16WaitCnt = 0;
}

void AnalogChannelSampleProc(void)
{
    int16_t Ret = 0;
    uint16_t u16AItemp = 0;

    Ret = IOChannel.Collector(&stAnalogInputCH1,EN_TYPE_AI1,(uint8_t *)&u16AItemp,2);
    if(ERR_OK != Ret)
    {
        printf("IOChannel.Collector EN_TYPE_AI1 fail!\r\n");
    }
    else
    {
        stSysParaData.u16AICH1Value = u16AItemp;
        printf("Analog input channel 1 value %d\r\n",stSysParaData.u16AICH1Value);
    }

    Ret = IOChannel.Collector(&stAnalogInputCH2,EN_TYPE_AI2,(uint8_t *)&u16AItemp,2);
    if(ERR_OK != Ret)
    {
        printf("IOChannel.Collector EN_TYPE_AI2 fail!\r\n");
    }
    else
    {
        stSysParaData.u16AICH2Value = u16AItemp;
        printf("Analog input channel 2 value %d\r\n",stSysParaData.u16AICH2Value);
    }
}

void DigitalChannelSampleProc(void)
{
    int16_t Ret = 0;
    uint8_t u8DigitalInputStatus = 0;

    Ret = IOChannel.Collector(&stDigitalInputCH1,EN_TYPE_DI1,&u8DigitalInputStatus,1);
    if(ERR_OK != Ret)
    {
        printf("IOChannel.Collector EN_TYPE_DI1 fail!\r\n");
    }
    else
    {
        stSysParaData.u8DICH1Status = u8DigitalInputStatus;
        printf("Digital input channel 1 value %d\r\n",stSysParaData.u8DICH1Status);
    }

    Ret = IOChannel.Collector(&stDigitalInputCH2,EN_TYPE_DI2,&u8DigitalInputStatus,1);
    if(ERR_OK != Ret)
    {
        printf("IOChannel.Collector EN_TYPE_DI2 fail!\r\n");
    }
    else
    {
        stSysParaData.u8DICH2Status = u8DigitalInputStatus;
        printf("Digital input channel 2 value %d\r\n",stSysParaData.u8DICH2Status);
    }
}

void SysSampleProc(void)
{
    uint32_t Ret = 0;
    uint32_t u32Vol = 0;  /*0.0001V*/
    int32_t i32Temp = 0;  /*0.1*/ 
    uint8_t u8BatLeftPer = 0;
    
    SysAdcInit();
    Ret = SrvIODevice_ioctl(&stADC, EN_ADC_CHN_VOL_BAT, (uint32_t)(&u32Vol));
    if (ERR_OK == Ret)
    {
        Ret = SrvIODevice_ioctl(&stADC, EN_ADC_TEMP, (uint32_t)(&i32Temp));
        if (ERR_OK == Ret)
        {
            u8BatLeftPer = BatParaCalculate(u32Vol, i32Temp);
        }
        UpdateBatLeftPer(u8BatLeftPer);
        if(5 == u8BatLeftPer)
        {
            SET_BATTERY_LOW_STATE();
        }
    }
    AnalogChannelSampleProc();
    DigitalChannelSampleProc();
    SrvIODevice_close(&stADC);
    if (ERR_OK != Ret)
    {
        printf("SysSampleProc get parking state err!\r\n");
    }
    printf("SysSampleProc excute!\r\n");
}
void SysTimelySampleProc(void)
{
    static uint32_t LastReportCounter = 0;

    if(LastReportCounter == stSysParaData.u32TimeCounter)
    {
        return;
    }

    if(0 == (stSysParaData.u32TimeCounter % \
        (stSysParaData.stUserSetData.u16SampleInteval * MINUTE_COUNTER)))
    {
        SysSampleProc();
        LastReportCounter = stSysParaData.u32TimeCounter;
    }
}

void DigitalInputStateChangeReport()
{

    if(0 != IOChannel.GetDIIrqStatus(EN_TYPE_DI1))
    {
        IOChannel.SetDIIrqFlag(EN_TYPE_DI1,0);
        SET_DI1_STATE(1);
    }
    
    if(0 != IOChannel.GetDIIrqStatus(EN_TYPE_DI2))
    {
        IOChannel.SetDIIrqFlag(EN_TYPE_DI2,0);
        SET_DI1_STATE(2);
    }

    if((0 != GET_DI1_STATE()) || (0 != GET_DI2_STATE()))
    {
        delay_ms(200);
        DigitalChannelSampleProc();
        printf("First! Digital input channel state change report!\r\n");
        Pro_SendRegularUpload();
        FeedDog();
        HAL_Delay(10000);
        if(EN_PRO_CMD_STATE_ACK_OK != ProGetCmdAckState(EN_PRO_FUNCCODE_TIMELY_REPORT))
        {
            printf("Second! Digital input channel state change report!\r\n");
            Pro_SendRegularUpload();
            FeedDog();
            HAL_Delay(5000);
        }
        SET_DI1_STATE(0);
        SET_DI1_STATE(0);
    }
}

void AnalogAlarmReportProc()
{
    static uint16_t u16LastDeviceState = 0;

    u16LastDeviceState = stSysParaData.u16DeviceState;
    
    if(0 == GET_AI1_STATE())
    {
        if((stSysParaData.u16AICH1Value > stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel) || \
            (stSysParaData.u16AICH1Value < stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel))
        {
            printf("Analog inout channel 1 alarm\r\n");
            SET_AI1_STATE(1);
        }
    }
    else
    {
        if((stSysParaData.u16AICH1Value < stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel) && \
            (stSysParaData.u16AICH1Value > stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel))
        {
            printf("Analog inout channel 1 alarm release\r\n");
            SET_AI1_STATE(0);
        }
    }

    if(0 == GET_AI2_STATE())
    {
        if((stSysParaData.u16AICH2Value > stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel) || \
            (stSysParaData.u16AICH2Value < stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel))
        {
            printf("Analog inout channel 2 alarm\r\n");
            SET_AI2_STATE(1);
        }
    }
    else
    {
        if((stSysParaData.u16AICH2Value < stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel) && \
            (stSysParaData.u16AICH2Value > stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel))
        {
            printf("Analog inout channel 2 alarm release\r\n");
            SET_AI2_STATE(0);
        }
    }

    if(u16LastDeviceState != stSysParaData.u16DeviceState)
    {
        printf("First! Analog input channel state change report!\r\n");
        Pro_SendRegularUpload();
        FeedDog();
        HAL_Delay(10000);
        if(EN_PRO_CMD_STATE_ACK_OK != ProGetCmdAckState(EN_PRO_FUNCCODE_TIMELY_REPORT))
        {
            printf("Second! Analog input channel state change report!\r\n");
            Pro_SendRegularUpload();
            FeedDog();
            HAL_Delay(5000);
        }
        u16LastDeviceState = stSysParaData.u16DeviceState;
    }
}


#ifdef IOT_NET_TYPE_NBIOT
int16_t GetNBIOTModulerPara(void)
{
    int16_t ret = -1;

    ret = SrvIODevice_open(&stNBIOT,EN_ID_NBIOT,IO_READ | IO_WRITE);
    if ((ERR_OK != ret) && (ERR_BEYOND_MAX != ret))
    {
        printf("GetNBIOTModulerPara open stNBIOT fail,ret = %d!\r\n", ret);
    }

    ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_WAKEUP,0);

    ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_GET_IMSI,\
        (uint32_t)(&stSysParaData.stFactorySetData.u8Imsi));
    if (ERR_OK != ret)
    {
        printf("GetNBIOTModulerPara read imsi fail,ret = %d!\r\n", ret);
    }

    ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_GET_IMEI,\
        (uint32_t)(&stSysParaData.stFactorySetData.u8Imei));
    if (ERR_OK != ret)
    {
        printf("GetNBIOTModulerPara read imei fail,ret = %d!\r\n", ret);
    }
    
    return ERR_OK;
}
#endif

#ifdef IOT_NET_TYPE_LORAWAN
int16_t GetLoraWanModulerPara(void)
{
    int16_t ret = -1;

    ret = SrvIODevice_open(&stLORAWAN,EN_ID_LORAWAN,IO_READ | IO_WRITE);
    if ((ERR_OK != ret) && (ERR_BEYOND_MAX != ret))
    {
        printf("GetLoraWanModulerPara open stLORAWAN fail,ret = %d!\r\n", ret);
        return ret;
    }
    ret = SrvIODevice_ioctl(&stLORAWAN, EN_CMD_GET_LORAWAN_MoudleVER,\
        (uint32_t)(&stSysParaData.stFactorySetData.u8LoraVer));
    if (ERR_OK != ret)
    {
        printf("GetLoraWanModulerPara read lorawan version fail,ret = %d!\r\n", ret);
        return ret;
    }
    SaveFactorySetData(stSysParaData.stFactorySetData);
    return ERR_OK;
}
#endif

void SysDriverInit(void)
{
    MX_GPIO_Init();
    printf("SysDriverInit ok!\r\n");
}

int main(void)
{
    uint32_t loop = 0;
    uint16_t u16waitnum = 0;
    int16_t ret = -1;
    
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    EEPROM_configure();
    ADC_configure();
    duart_configure();
    MX_LPTIM1_Init();
    led_configure();
    ResetMode();
    Led_SetWorkMode(WORK_LED,LED_MODE_LIGHTON,LED_ALWAYSALIVE,LED_BLK_FREQ_1,3000);
    printf("System start ok !\r\n");
    printf("Software ver is %d.%d.%d.%d!\r\n",\
        RESERVE_SW_VER, MAJOR_SW_VER, MINOR_SW_VER, RELEASE_SW_VER);
    printf("Svn ver is %s!\r\n", CONVERT_TO_STR(SVN_VERSION));
    printf("Device type is %d!\r\n", DEVICE_TYPE);
    printf("Device name is %s!\r\n", DEVICE_NAME);
    SysDriverInit();
    IOChannel.Init();
    SysParaInit();
#ifdef IOT_NET_TYPE_LORAWAN
    Lorawan_configure(stSysParaData.stUserSetData.stLoraWanCfg);
#endif

#ifdef IOT_NET_TYPE_NBIOT
    nbiot_configure(stSysParaData.stUserSetData.stNbiotCfg);
#endif
    Com_Init();
    SysSampleProc();
    printf("Please press the Enter key to enter the configuration mode!\r\n");
    while(u16waitnum < 100)
    {
        Com_process();
        u16waitnum++;
        delay_ms(100);
    }
    
#ifdef IOT_NET_TYPE_NBIOT
    GetNBIOTModulerPara();
#endif

#ifdef IOT_NET_TYPE_LORAWAN
    GetLoraWanModulerPara();
#endif
    SysSampleProc();
#if 1
    Pro_SendPowerOnReportFrame(EN_PRO_FUNCCODE_POWERON_REPORT, 0);
    HAL_Delay(10000);
    if(EN_PRO_CMD_STATE_ACK_OK != ProGetCmdAckState(EN_PRO_FUNCCODE_POWERON_REPORT))
    {
        printf("Power on report send second time!\r\n");
        Pro_SendPowerOnReportFrame(EN_PRO_FUNCCODE_POWERON_REPORT, 0);
    }
    HAL_Delay(10000);
#endif
    IWDG_Init(6,4095);
    while(1)
    {
        FeedDog();
        if(0 == (loop % 200000))
        {
            loop = 0;
            printf("System polling!\r\n");
            printf("Please press the Enter key to enter the configuration mode!\r\n");
        }
        loop++;
        SysTimelySampleProc();
        FeedDog();
        DigitalInputStateChangeReport();
        AnalogAlarmReportProc();
        /*Timely upload check*/
        TimelyReportProc();
        Com_process();
        FeedDog();
        Low_Power_Enter();
        Low_Power_Exit();
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
    PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
