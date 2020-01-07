#ifndef __LED_H
#define __LED_H

#include "stm32l0xx_hal.h"
#include "stdint.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"

typedef enum
{
    WORK_LED,
    NET_LED,
    BLUE_LED,
    MAX_LED,
}LedType_t;


typedef enum
{
    LED_ALIVE,
    LED_UNALIVE,
    LED_ALWAYSALIVE,
}LedEnableState_t;

/**< ioctl控制命令 */
typedef enum
{
    EN_LED_CMD_SET,            /**< Set led */
    EN_LED_CMD_READ,            /**< Read led state*/
}EnLedIoctlCmd_t;

typedef enum
{
    EN_ERR_LED_OK,
    EN_ERR_LED_NULL,
    EN_ERR_LED_TIMOUT,
    EN_ERR_LED_PARAM
}EnLedErrCode_t;

/**<led state struction def*/
typedef enum
{
    LED_MODE_LIGHTON,
    LED_MODE_LIGHTOFF,
    LED_MODE_BLINKING,
}LedWorkMode_t;

typedef enum
{
    LED_BLK_FREQ_1 = 0,
    LED_BLK_FREQ_2,
    LED_BLK_FREQ_5,
    LED_BLK_FREQ_10,
    LED_BLK_FREQ_BUTT,
}LedBlinkFreq_t;

/**< LED ctrl struct define */

#pragma pack(1)
typedef struct
{
    LedType_t LedType;
    volatile LedEnableState_t LedEnableState;
    LedWorkMode_t LedWorkMode;
    LedBlinkFreq_t LedBlinkFreq;     /* 闪烁频率 */
    uint32_t u32Duration;           /* 点亮或闪烁的时间 ms */
    volatile uint32_t u32LedCounter;
    volatile uint8_t u8TaskQry;
}StLedCtrl_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    LedType_t LedType;
    LedEnableState_t LedEnableState;
    LedWorkMode_t LedWorkMode;
    LedBlinkFreq_t LedBlinkFreq;     /* 闪烁频率 */
    uint32_t u32Duration;           /* 点亮或闪烁的时间 ms */
}StLedConfigPara_t;
#pragma pack()


extern StSrvIODevice_t stledDevice;
extern void led_configure(void);
extern void LedCtrlTimeoutCallback(void);

#endif

