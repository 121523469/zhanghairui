#include <string.h>
#include <stdio.h>
#include "SrvErrorNo.h"
#include "common_interface.h"
#include "stdlib.h"
#include "stm32l0xx_hal.h"

StSysPara_t stSysParaData;

void delay_us(uint32_t us)
{
    while(us--)
    {
        __NOP();
        __NOP();
    }
}

void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}

uint32_t htonl(uint32_t u32Src)
{
    uint8_t *p = (uint8_t *)&u32Src;
    
    return (uint32_t)(p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]);
}

uint16_t htons(uint16_t u16Src)
{
    uint8_t *p = (uint8_t *)&u16Src;
    
    return (uint16_t)(p[0] << 8 | p[1]);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}


int16_t ReadUserSetData(StUserSetData_t *p_UserData)
{
    int16_t ret = -1;

    /* 读出用户配置参数到 User EEPROM Storage */
    ret = SrvIODevice_open(&stEEPROMUserDevice, EN_ID_USERSTORAGE, IO_READ);
    if(ERR_OK != ret)
    {
        printf("ReadUserSetData open device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_seek(&stEEPROMUserDevice, 0);
    if(ERR_OK != ret)
    {
        printf("ReadUserSetData seek device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_read(&stEEPROMUserDevice, (uint32_t *)p_UserData, sizeof(StUserSetData_t));
    if(ERR_OK != ret)
    {
        printf("ReadUserSetData read device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_close(&stEEPROMUserDevice);
    if(ERR_OK != ret)
    {
        printf("ReadUserSetData close device err,ret = %d!\r\n", ret);
        return ret;
    }
    return ret;
}


int16_t SaveUserSetData(StUserSetData_t pUserData)
{
    int16_t ret = -1;

    ret = SrvIODevice_open(&stEEPROMUserDevice, EN_ID_USERSTORAGE, IO_WRITE);
    if(ERR_OK != ret)
    {
        printf("SaveUserSetData open device err,ret = %d!\r\n", ret);
        return ret;
    }

    SrvIODevice_seek(&stEEPROMUserDevice,0);
    if(ERR_OK != ret)
    {
        printf("SaveUserSetData seek device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_write(&stEEPROMUserDevice,(uint32_t *)&pUserData, sizeof(StUserSetData_t)/4);
    if(ERR_OK != ret)
    {
        printf("SaveUserSetData write device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_close(&stEEPROMUserDevice);
    if(ERR_OK != ret)
    {
        printf("SaveUserSetData close device err,ret = %d!\r\n", ret);
        return ret;
    }
    return ret;
}

int16_t ReadFactorySetData(StFactorySetData_t *p_FactoryData)
{
    int16_t ret = -1;

    /* 读出用户配置参数到 User EEPROM Storage */
    ret = SrvIODevice_open(&stEEPROMManuDevice, EN_ID_MANUSTORAGE, IO_READ);
    if(ERR_OK != ret)
    {
        printf("ReadFactorySetData open device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_seek(&stEEPROMManuDevice, 0);
    if(ERR_OK != ret)
    {
        printf("ReadFactorySetData seek device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_read(&stEEPROMManuDevice, (uint32_t *)p_FactoryData, sizeof(StFactorySetData_t));
    if(ERR_OK != ret)
    {
        printf("ReadFactorySetData read device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_close(&stEEPROMManuDevice);
    if(ERR_OK != ret)
    {
        printf("ReadFactorySetData close device err,ret = %d!\r\n", ret);
        return ret;
    }
    return ret;
}


int16_t SaveFactorySetData(StFactorySetData_t pFactoryData)
{
    int16_t ret = -1;

    ret = SrvIODevice_open(&stEEPROMManuDevice, EN_ID_MANUSTORAGE, IO_WRITE);
    if(ERR_OK != ret)
    {
        printf("SaveFactorySetData open device err,ret = %d!\r\n", ret);
        return ret;
    }

    SrvIODevice_seek(&stEEPROMManuDevice,0);
    if(ERR_OK != ret)
    {
        printf("SaveFactorySetData seek device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_write(&stEEPROMManuDevice,(uint32_t *)&pFactoryData, sizeof(StFactorySetData_t) / 4);
    if(ERR_OK != ret)
    {
        printf("SaveFactorySetData write device err,ret = %d!\r\n", ret);
        return ret;
    }

    ret = SrvIODevice_close(&stEEPROMManuDevice);
    if(ERR_OK != ret)
    {
        printf("SaveFactorySetData close device err,ret = %d!\r\n", ret);
        return ret;
    }

    return ret;
}

int16_t RestorFactoryPara(void)
{
    int16_t Ret = -1;
#ifdef IOT_NET_TYPE_NBIOT
    uint32_t IpAddr = stSysParaData.stUserSetData.stNbiotCfg.u32IpAddr;
    uint16_t Port = stSysParaData.stUserSetData.stNbiotCfg.u16Port;
    EnTransModeType_t EnTransMode = stSysParaData.stUserSetData.stNbiotCfg.EnTransMode;
#endif

    /* 读出用户配置参数到 User EEPROM Storage */
    memcpy(&(stSysParaData.stUserSetData),\
        &(stSysParaData.stFactorySetData.stDefaultUserData),\
        sizeof((stSysParaData.stUserSetData)));
#ifdef IOT_NET_TYPE_NBIOT
    /*Restore Factory para without ip、port*/
    stSysParaData.stUserSetData.stNbiotCfg.u32IpAddr = IpAddr;
    stSysParaData.stUserSetData.stNbiotCfg.u16Port = Port;
    stSysParaData.stUserSetData.stNbiotCfg.EnTransMode = EnTransMode;
#endif
    Ret = SaveUserSetData(stSysParaData.stUserSetData);
    if(ERR_OK != Ret)
    {
        printf("Pro_RestorFactoryPara save user para err,Ret=%d!\r\n", Ret);
        return Ret;
    }

    return Ret;
}

/*For protocol send response msg*/
int16_t ModulerSendResponseMsg(int8_t *SendData, uint16_t Len)
{
    int16_t Ret = -1;
    FeedDog();

#ifdef IOT_NET_TYPE_NBIOT
#if 1
    Ret = SrvIODevice_open(&stNBIOT,EN_ID_NBIOT,IO_READ | IO_WRITE);
    if ((0 != Ret) && (ERR_BEYOND_MAX != Ret))
    {
        printf("ModulerSendResponseMsg open stNBIOT fail,Ret=%d!\r\n", Ret);
        return Ret;
    }
#endif
    Ret = SrvIODevice_write(&stNBIOT, SendData, Len);
    if (0 != Ret)
    {
        printf("ModulerSendResponseMsg write stNBIOT fail,Ret=%d!\r\n", Ret);
        return Ret;
    }

    return Ret;
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    Ret = SrvIODevice_write(&stLORAWAN, SendData, Len);
    if (0 != Ret)
    {
        printf("ModulerSendResponseMsg write stLORAWAN fail,Ret=%d!\r\n", Ret);
        return Ret;
    }

    return Ret;
#endif
}

/*For protocol send msg*/
int16_t ModulerSendMsg(int8_t *SendData, uint16_t Len)
{
    int16_t Ret = -1;
    FeedDog();
#ifdef IOT_NET_TYPE_NBIOT
    Ret = SrvIODevice_open(&stNBIOT, EN_ID_NBIOT, IO_READ | IO_WRITE);
    if ((0 != Ret) && (ERR_BEYOND_MAX != Ret))
    {
        printf("ModulerSendseMsg open stNBIOT fail,Ret=%d!\r\n", Ret);
        goto EXIT;
    }
    Ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_SEARCH_NETWORK, 0);
    if (0 != Ret)
    {
        printf("ModulerSendseMsg ioctl search network fail,Ret=%d!\r\n", Ret);
        goto EXIT;
    }
    
    Led_SetWorkMode(NET_LED,LED_MODE_BLINKING,LED_ALWAYSALIVE,LED_BLK_FREQ_5,3000);

    Ret = SrvIODevice_write(&stNBIOT, SendData, Len);
    if (0 != Ret)
    {
        printf("ModulerSendseMsg write stNBIOT fail,Ret=%d!\r\n", Ret);
        goto EXIT;
    }
    
    FeedDog();
    delay_ms(2000);
    
    /*由于udp数据有可能缓存，每次发上行后，尝试读取下行数据*/
    Ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_DLMSG_PROC, 0);
    if (0 != Ret)
    {
        printf("ModulerSendResponseMsg read downlink msg fail,Ret=%d!\r\n", Ret);
        goto EXIT;
    }
#if 1
    Ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_GETRSSI, (uint32_t)(&stSysParaData.s32Rssi));
    if (0 != Ret)
    {
        printf("ModulerSendResponseMsg read Rssi fail,Ret=%d!\r\n", Ret);
    }
    Ret = SrvIODevice_ioctl(&stNBIOT, EN_CMD_GETECL, (uint32_t)(&stSysParaData.unSpecData.stNBSpecData));
    if (0 != Ret)
    {
        printf("ModulerSendResponseMsg read Ecl fail,Ret=%d!\r\n", Ret);
    }
#endif
EXIT:
    SrvIODevice_ioctl(&stNBIOT, EN_CMD_ENABLE_PSM,0);
    
    return Ret;
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    Ret = SrvIODevice_open(&stLORAWAN, EN_ID_LORAWAN, IO_READ | IO_WRITE);
    if ((0 != Ret) && (ERR_BEYOND_MAX != Ret))
    {
        printf("ModulerSendseMsg open stLORAWAN fail,Ret=%d!\r\n", Ret);
        return Ret;
    }
    
    Ret = SrvIODevice_ioctl(&stLORAWAN, EN_CMD_GET_RSSI, (uint32_t)(&stSysParaData.s32Rssi));
    if (0 != Ret)
    {
        printf("ModulerSendseMsg search LORAWAN net fail,Ret=%d!\r\n", Ret);
        return Ret;
    }
    
    Ret = SrvIODevice_ioctl(&stLORAWAN, EN_CMD_SEARCH_NETWORK, 0);
    if (0 != Ret)
    {
        printf("ModulerSendseMsg search LORAWAN net fail,Ret=%d!\r\n", Ret);
        return Ret;
    }

    Ret = SrvIODevice_write(&stLORAWAN, SendData, Len);
    if (0 != Ret)
    {
        printf("ModulerSendseMsg write stLORAWAN fail,Ret=%d!\r\n", Ret);
        return Ret;
    }

    return Ret;
#endif
}


int16_t Led_SetWorkMode(LedType_t LedType,LedWorkMode_t LedWorkMode,LedEnableState_t LedEnableState,
                                   LedBlinkFreq_t LedBlinkFreq,uint32_t u32Duration)
{
    int8_t ret = -1;
    StLedConfigPara_t stLedConfigPara;
    
    SrvIODevice_open(&stledDevice,EN_ID_LED,IO_READ | IO_WRITE);
    stLedConfigPara.LedType = LedType;
    stLedConfigPara.LedWorkMode = LedWorkMode;
    stLedConfigPara.u32Duration = u32Duration;
    stLedConfigPara.LedBlinkFreq = LedBlinkFreq;
    stLedConfigPara.LedEnableState = LedEnableState;
    
    ret = SrvIODevice_ioctl(&stledDevice, EN_LED_CMD_SET, (uint32_t)&stLedConfigPara);
    if (0 != ret)
    {
        printf("StledDevice open fail,ret=%d!\r\n", ret);
    }
    ret = SrvIODevice_close(&stledDevice);
    if (ERR_OK != ret)
    {
        printf("StledDevice close fail,ret=%d!\r\n", ret);
    }
    return ret;
}

