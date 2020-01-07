#ifndef _IOCHANNEL_MANAGE_H_
#define _IOCHANNEL_MANAGE_H_

#include <stdint.h>

#define AICH_VOLTAGE_MIN    0.5         /**< 模拟电压输入 MIN*/
#define AICH_VOLTAGE_MAX    4.5         /**< 模拟电压输入 MAX*/

#define AICH_CURRENT_MIN    4           /**< 模拟电流输入 MIN*/
#define AICH_CURRENT_MAX    20          /**< 模拟电流输入 MAX*/

#define DIN_CH1_GPIO_PORT       GPIOC
#define DIN_CH1_GPIO_PIN        GPIO_PIN_3

#define DIN_CH2_GPIO_PORT       GPIOC
#define DIN_CH2_GPIO_PIN        GPIO_PIN_2

#define AIN_CH1_GPIO_PORT       GPIOC
#define AIN_CH1_GPIO_PIN        GPIO_PIN_4

#define AIN_CH2_GPIO_PORT       GPIOC
#define AIN_CH2_GPIO_PIN        GPIO_PIN_5

#define AIN_CH_POWER_GPIO       GPIOB
#define AIN_CH_POWER_PIN        GPIO_PIN_6

#define DO_CH1_GPIO_PORT        GPIOC
#define DO_CH1_GPIO_PIN         GPIO_PIN_1

#define AICH1_TYPE_GPIO_PORT    GPIOA
#define AICH1_TYPE_GPIO_PIN     GPIO_PIN_5

#define AICH2_TYPE_GPIO_PORT    GPIOA
#define AICH2_TYPE_GPIO_PIN     GPIO_PIN_6

#define SUPPLY_POWER_GPIO_PORT  GPIOB
#define SUPPLY_POWER_GPIO_PIN   GPIO_PIN_7

#define AICH1_STATE     0x0001
#define AICH2_STATE     0x0002
#define DICH1_STATE     0x0004
#define DICH2_STATE     0x0008
#define DOCH1_STATE     0x0010
#define RS485CH_STATE   0x0020

typedef enum
{
    EN_DO_SET,
    EN_DO_RESET,
}EnDOChannelStatus;

typedef enum
{
    EN_AI_TYPE_VOL = 0,
    EN_AI_TYPE_CUR,
}EnAnalogType_t;

typedef enum
{
    EN_TYPE_AI1 = 0,
    EN_TYPE_AI2,
    EN_TYPE_DI1,
    EN_TYPE_DI2,
    EN_TYPE_DO1,
    EN_TYPE_RS485,
    EN_TYPE_SUPPLY,
    EN_TYPE_MAX,
}EnChannelType_t;


typedef int16_t (* Collector)(EnChannelType_t p_enChannelType,uint8_t *p_u8Buff,uint8_t p_u8Size);

typedef enum
{
    CHANNEL_DISABLE = 0,
    CHANNEL_ENABLE = 1,
}EnChannelState_t;


#pragma pack(1)
typedef struct
{
    EnChannelType_t  enChannelType;             /**< Channel Type*/
    uint16_t u16Length;
    uint8_t u8Value[];
}StTLV;
#pragma pack()

typedef struct IOChannel_param_s
{
    EnChannelType_t  enChannelType;             /**< Channel Type*/
    EnChannelState_t EnChannelState;
    Collector fpCollector;     /**< Pointer to Device Item */
    struct IOChannel_param_s *next;
}IOChannel_t;

typedef struct
{
    void (*Init)(void);
    int8_t (*Collector)(IOChannel_t* const obj,EnChannelType_t enChannelType,uint8_t *pu8Buff,uint8_t p_u8Size);
    void (*SetDIIrqFlag)(EnChannelType_t enChannelType,uint8_t u8Status);
    uint8_t (*GetDIIrqStatus)(EnChannelType_t enChannelType);
    uint8_t (*GetAnalogType)(void);
    void (*DOChannelOutput)(EnDOChannelStatus EnDoStatus);
    uint8_t (*GetDIChannelLevel)(EnChannelType_t enChannelType);
}IOChannels_t;

struct StIOChannelistLink
{
    struct StIOChannelistLink *pstNext;
    struct StIOChannelistLink *pstPrev;
};
typedef struct StIOChannelistLink StIOChannelistLink_t;
typedef struct 
{
    StIOChannelistLink_t stHead;
    StIOChannelistLink_t *pstActualIter;
    StIOChannelistLink_t *pstTmp;
}StIOChanneList_t;

/** IO Channel Item struct */
typedef struct
{
    StIOChannelistLink_t stListLink;                    /**< Link list. Must be the first element */
    EnChannelState_t EnChannelState;
    EnChannelType_t  enChannelType;             /**< Channel Type*/
    Collector fpCollector;     /**< Pointer to Device Item */
}StIOChannelItem_t;

#pragma pack(1)
typedef struct
{
    volatile uint16_t u16ChannelStatus;
    volatile uint8_t u8AnalogType;
}StIOChannelPara;
#pragma pack()

extern IOChannel_t stAnalogInputCH1;
extern IOChannel_t stAnalogInputCH2;
extern IOChannel_t stDigitalInputCH1;
extern IOChannel_t stDigitalInputCH2;
extern IOChannel_t stDigitalOutputCH1;
extern IOChannel_t stRS485CH1;

extern const IOChannels_t IOChannel;
#endif