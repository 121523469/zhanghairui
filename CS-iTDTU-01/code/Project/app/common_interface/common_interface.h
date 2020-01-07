#ifndef _COMMON_INTERFACE_H_
#define _COMMON_INTERFACE_H_

#include "protocol.h"
#include "EEPROM.h"
#include "Flash.h"
#include "Led.h"
#define UPDATE_TIMER_COUNTER()      do \
{\
    stSysParaData.u32TimeCounter++;\
} while(0)


/** 无线终端系统数据结构体 */
typedef struct
{
    StUserSetData_t stUserSetData;         /**< 用户配置区域结构体 */
    StFactorySetData_t stFactorySetData;   /**< 出厂配置区域结构体*/
    StFWVersion_t stFWversion;             /**< Firmware version */
    UnSpec_t unSpecData;                   /**< 无线模块特殊字段联合体 不同的无线输出形式其定义不一样 但是长度一样 */
    uint16_t u16DeviceState;               /**< Device State */
    int32_t s32Rssi;                       /**< Signal strength */
    volatile uint32_t u32TimeCounter;
    uint8_t u8DeviceType;                  /**< Device type */
    uint8_t u8BatteryLeftPercent;          /**< Battery 电量百分比*/
    volatile uint16_t u16UplinkMsgSn;      /**< Uplink  msg sn */
    volatile uint16_t u16DownlinkMsgSn;    /**< Downlink  msg sn */
    uint32_t u32SysLowPowerState;          /*1-- low power state*/
    uint16_t u16AICH1Value;                /**< Analog  input channel 1 value */                      
    uint16_t u16AICH2Value;                /**< Analog  input channel 2 value */
    uint8_t u8DICH1Status;                 /**< Digital input channel 1 status */
    uint8_t u8DICH2Status;                 /**< Digital input channel 2 status */
    uint8_t u8AnalogType;                  /**< Analog  input channel type */
    uint8_t u8RS485Value[4];               /**< For rs485 reserve */
}StSysPara_t;

#pragma pack(1)
typedef struct
{
    uint32_t u32DeviceSN;
    StFWVersion_t stFWversion;
    StUserSetData_t stUserSetData;
}StNfcPara_t;

#pragma pack()




extern StSysPara_t stSysParaData;
extern StNfcPara_t StNfcPara;
extern StSrvIODevice_t stNBIOT;
extern StSrvIODevice_t stFLASH;
extern StSrvIODevice_t stADC;
extern int16_t ReadUserSetData(StUserSetData_t *p_UserData);
extern int16_t SaveUserSetData(StUserSetData_t pUserData);
extern int16_t ReadFactorySetData(StFactorySetData_t *p_FactoryData);
extern int16_t SaveFactorySetData(StFactorySetData_t pFactoryData);
extern int16_t RestorFactoryPara(void);
extern int16_t SaveUpgradeInfoData(void *p_SaveData,uint32_t u32Offset,uint16_t u16Size);
extern int16_t ReadUpgradeInfoData(void *p_ReadData,uint32_t u32Offset,uint16_t u16Size);
extern int16_t EraseUpgradeInfoData(uint32_t u32Offset,uint16_t u16Page);
extern int16_t Led_SetWorkMode(LedType_t LedType,LedWorkMode_t LedWorkMode,LedEnableState_t LedEnableState,
                                   LedBlinkFreq_t LedBlinkFreq,uint32_t u32Duration);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

extern int16_t ModulerSendResponseMsg(int8_t *SendData, uint16_t Len);
extern int16_t ModulerSendMsg(int8_t *SendData, uint16_t Len);
extern void FeedDog(void);
extern void delay_ms(uint32_t ms);
extern uint32_t htonl(uint32_t u32Src);
extern uint16_t htons(uint16_t u16Src);
#endif




