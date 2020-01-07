#include "product_config.h"
#ifndef _PRODUCT_PROTOCOL_H_
#define _PRODUCT_PROTOCOL_H_

#include "stdint.h"
#include "lorawan.h"
#include "nbiot.h"
#include "IOChannelManage.h"

#define DEVICE_STATE_BP    (0X0001) /*0-电池正常，1-电池欠压*/
#define DEVICE_STATE_AI1   (0X0002) /*0-云端应答正常，1-云端应答错误*/
#define DEVICE_STATE_AI2   (0X0004) /*0-无线模块状态正常，1-无线模块状态异常*/
#define DEVICE_STATE_DI1   (0X0008) /*0-设备布防，1-设备撤防*/
#define DEVICE_STATE_DI2   (0X0010) /*0-无车，1-有车*/

#define SET_BATTERY_LOW_STATE() do \
{\
    if(0 == stSysParaData.u16DeviceState | DEVICE_STATE_BP) \
    {\
        stSysParaData.u16DeviceState |= DEVICE_STATE_BP;\
    }\
}while(0)

/*0--ok,1--alarm*/
#define SET_AI1_STATE(state) do \
{\
    if(0 == (state))\
    {\
        stSysParaData.u16DeviceState &= ~DEVICE_STATE_AI1;\
    }\
    else if(1 == state)\
    {\
        stSysParaData.u16DeviceState |= DEVICE_STATE_AI1;\
    }\
}while(0)

/*0--ok,1--alarm*/
#define SET_AI2_STATE(state) do \
{\
    if(0 == (state))\
    {\
        stSysParaData.u16DeviceState &= ~DEVICE_STATE_AI2;\
    }\
    else if(1 == state)\
    {\
        stSysParaData.u16DeviceState |= DEVICE_STATE_AI2;\
    }\
}while(0)


/*0--ok,1--status change*/
#define SET_DI1_STATE(state) do \
{\
    if(0 == (state))\
    {\
        stSysParaData.u16DeviceState &= ~DEVICE_STATE_DI1;\
    }\
    else if(1 == state)\
    {\
        stSysParaData.u16DeviceState |= DEVICE_STATE_DI1;\
    }\
}while(0)

/*0--ok,1--status change*/
#define SET_DI2_STATE(state) do \
{\
    if(0 == (state))\
    {\
        stSysParaData.u16DeviceState &= ~DEVICE_STATE_DI2;\
    }\
    else if(1 == state)\
    {\
        stSysParaData.u16DeviceState |= DEVICE_STATE_DI2;\
    }\
}while(0)

#define GET_AI1_STATE()     (stSysParaData.u16DeviceState & DEVICE_STATE_AI1)
#define GET_AI2_STATE()     (stSysParaData.u16DeviceState & DEVICE_STATE_AI2)
#define GET_DI1_STATE()     (stSysParaData.u16DeviceState & DEVICE_STATE_DI1)
#define GET_DI2_STATE()     (stSysParaData.u16DeviceState & DEVICE_STATE_DI2)

typedef struct
{
    uint16_t u16AICH1LowLevel;
    uint16_t u16AICH1HighLevel;
    uint16_t u16AICH2LowLevel;
    uint16_t u16AICH2HighLevel;
}StConfigureCmdPayload_t;

/** 用户配置区域结构体 */
typedef struct
{
    uint16_t u16DeviceID;               /**< New Device ID */
    uint16_t u16HeartBeatInteval;       /**< Timely repeat internal， FFFF时表示保持当前设置不变 */
    uint16_t u16SampleInteval;
    StConfigureCmdPayload_t stCfgPayload;
#ifdef IOT_NET_TYPE_NBIOT
    stNbiotCfg_t stNbiotCfg;
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    stLoraWanCfg_t stLoraWanCfg;             /**< Lorawan cfg para */
    uint8_t u8Reserve[2];
#endif
}StUserSetData_t;


/** 出厂配置区域结构体 */
typedef struct
{
    uint32_t u32DeviceSN;          /**< Device serial number */
    uint8_t u8ModelPostfix[16];    /**< 终端型号后缀描述，表示何种无线输出形式以及运营商支持等 */
    uint8_t u8HWVersion;           /**< Hardware version */
#ifdef IOT_NET_TYPE_NBIOT
    uint8_t u8Imei[16];            /**< Device imei */
    uint8_t u8Imsi[16];            /**< Device imsi */
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    uint8_t u8LoraVer[LORAWAN_VERSION_MAX];
#endif
    EnTransModeType_t u8TransMode;       /*Transport    mode(Coap or Udp)*/
    uint8_t u8ResvBuf[3];
    StUserSetData_t stDefaultUserData;
}StFactorySetData_t;

#pragma pack(1)
/** NBIOT特殊字段结构体 */
typedef struct {
    uint8_t u8ECL;               /**< Signal coverage level */
    uint8_t u8SNR;                      /**< Signal noise ratio */
    uint16_t u16PCI;                     /**< Timely repeat inteval */
    uint32_t u32cellID;                  /**< housing estate ID */
}StNBSpec_t;
#pragma pack()

#pragma pack(1)
/** 特殊字段联合体 */
typedef union {
    StNBSpec_t stNBSpecData;             /**< Signal coverage level */
    uint8_t  u8SpecData[8];           	 /**< Signal noise ratio */
}UnSpec_t;
#pragma pack()

#pragma pack(1)
typedef struct {
#ifdef IOT_NET_TYPE_NBIOT
    uint8_t u8Imei[16];                    /**< Device imei */
    uint8_t u8Imsi[16];                    /**< Device imsi */
#endif
    StConfigureCmdPayload_t stCfgPayload;
    uint8_t u8AnalogType;
}StPoweronFramePayload_t;
#pragma pack()

#pragma pack(1)
typedef struct {
#ifdef IOT_NET_TYPE_NBIOT
    UnSpec_t unSpecData;        /**< 无线模块特殊字段联合体 不同的无线输出形式其定义不一样 但是长度一样 */
#endif
    uint16_t u16AI1SampleValue;
    uint16_t u16AI2SampleValue;
    uint8_t  u8DI1Level;
    uint8_t  u8DI2Level;
    uint8_t  Reserve[4];
}StRegularUploadPayload_t;
#pragma pack()

extern int16_t Pro_GetRegularPayload(StRegularUploadPayload_t *pstRegularPayload);
extern int16_t Pro_GetPowerOnPayload(StPoweronFramePayload_t *stPowerOnPayload);
extern int8_t Pro_UpdateCfgPayload(StConfigureCmdPayload_t stcfgPayload);
extern int16_t Pro_ResetAlarmState(void);
#endif
