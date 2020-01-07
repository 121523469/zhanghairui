/**
******************************************************************************
* File Name          : LED.c
* Description        : This file provides code for the configuration
*                      of the USART instances.
******************************************************************************/
#include "led.h"
#include "tim.h"
#include "gpio.h"
#include <string.h>
#include <stdint.h>
#include "SrvErrorNo.h"
#include "stm32l0xx_hal_tim.h"
#include "common_interface.h"

/* User led gpio define*/
#define WORK_LED_PORT       GPIOC
#define WORK_LED_PIN        GPIO_PIN_10

#define NET_LED_PORT       GPIOC
#define NET_LED_PIN        GPIO_PIN_11


/* User led gpio control*/
#define WORK_LED_ON()       do{HAL_GPIO_WritePin(WORK_LED_PORT, WORK_LED_PIN, GPIO_PIN_RESET);}while(0)
#define WORK_LED_OFF()      do{HAL_GPIO_WritePin(WORK_LED_PORT, WORK_LED_PIN, GPIO_PIN_SET);}while(0)
#define WORK_LED_BLINK()    do{HAL_GPIO_TogglePin(WORK_LED_PORT,WORK_LED_PIN);}while(0)

/* User led gpio control*/
#define NET_LED_ON()       do{HAL_GPIO_WritePin(NET_LED_PORT, NET_LED_PIN, GPIO_PIN_RESET);}while(0)
#define NET_LED_OFF()      do{HAL_GPIO_WritePin(NET_LED_PORT, NET_LED_PIN, GPIO_PIN_SET);}while(0)
#define NET_LED_BLINK()    do{HAL_GPIO_TogglePin(NET_LED_PORT,NET_LED_PIN);}while(0)


static int16_t led_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t led_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t led_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem ,StSrvIODevice_t* p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);
static int8_t led_init(void);
static void led_Led_cb(void);
static int8_t led_SetWorkPara(StLedConfigPara_t *p_LedConfigPara);
static void led_Reset(uint8_t Index);
static void led_LightOffControl(uint8_t Index);
static void led_TaskListHandle(uint8_t Index);


StSrvIODevice_t stledDevice;
static StSrvIODeviceItem_t m_stledDeviceItem;
static StSrvIOOperations_t m_stledOperation = 
{
    .open = led_open,
    .close = led_close,
    .ioctl = led_ioctl,
};

/**
* @brief All led manage
*/
StLedCtrl_t stLedCtrlTable[MAX_LED]= 
{
    {WORK_LED,   LED_UNALIVE, LED_MODE_LIGHTOFF, LED_BLK_FREQ_BUTT, 3000, 0, 0},
    {NET_LED, LED_UNALIVE, LED_MODE_LIGHTOFF, LED_BLK_FREQ_BUTT, 3000, 0, 0},
    {BLUE_LED,  LED_UNALIVE, LED_MODE_LIGHTOFF, LED_BLK_FREQ_BUTT, 3000, 0, 0},
};

/* @brief Led闪烁周期
*/
const uint16_t LedFreqTab[LED_BLK_FREQ_BUTT] =
{
    500,  /*ms*/
    250,
    100,
    50,
};

/**
*  @brief 注册led Device
*/
void led_configure(void)
{
    SrvIODeviceItem_register(&m_stledDeviceItem, EN_ID_LED, &m_stledOperation, IO_READ | IO_WRITE);
    MX_TIM6_Init();
}


/** 
  @brief led GPIO初始化
*/
static int8_t led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin = WORK_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WORK_LED_PORT, &GPIO_InitStruct);
    /*Configure GPIO pin Output Level */
    led_LightOffControl(WORK_LED);

    GPIO_InitStruct.Pin = NET_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NET_LED_PORT, &GPIO_InitStruct);
    /*Configure GPIO pin Output Level */
    led_LightOffControl(NET_LED);

    return ERR_OK;
}

/** 
* @brief Open led Device
*/
static int16_t led_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODeviceItem == NULL)
    {
        return ERR_FAULT;
    }
    if(p_pstSrvIODevice == NULL)
    {
        return ERR_FAULT;
    }

    led_init();
    return ERR_OK;
}

/** 
* @brief Close led Device
*/
static int16_t led_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODeviceItem == NULL)
    {
        return ERR_FAULT;
    }
    if(p_pstSrvIODevice == NULL)
    {
        return ERR_FAULT;
    }

    return ERR_OK;
}

/** 
* @brief iocrl led Device
*/
static int16_t led_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem ,StSrvIODevice_t* p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    if((NULL == p_pstSrvIODeviceItem) || (NULL == p_pstSrvIODevice) || (NULL == p_u32Arg))
    {
        return ERR_FAULT;
    }

    switch(p_u32Command)
    {
        case EN_LED_CMD_SET:
        {
            return led_SetWorkPara((StLedConfigPara_t *)p_u32Arg);
        }
        default:
        {
            printf("Invalid command!\r\n");
            break;
        }
    }
    
    return ERR_OK;
}

/**
* @brief 参数初始化
*/
static void led_Reset(uint8_t Index)
{
    led_LightOffControl(Index);
    stLedCtrlTable[Index].LedEnableState = LED_UNALIVE;
    stLedCtrlTable[Index].u32LedCounter = 0;
}

/**
* @brief 点亮Led
*/
static void led_LightOnControl(uint8_t Index)
{
    if(Index > MAX_LED)
    {
        return;
    }

    switch(Index)
    {
        case WORK_LED:
        {
            WORK_LED_ON();
            break;
        }
        case NET_LED:
        {
            NET_LED_ON();
            break;
        }
        /*add user led*/
        default:break;
    }
}

/**
* @brief 熄灭Led
*/
static void led_LightOffControl(uint8_t Index)
{
    if(Index > MAX_LED)
    {
        return;
    }

    switch(Index)
    {
        case WORK_LED:
        {
            WORK_LED_OFF();
            break;
        }
        case NET_LED:
        {
            NET_LED_OFF();
            break;
        }
        /*add user led*/
        default:break;
    }
}

/**
* @brief LED闪烁控制
*/
static void led_BlinkControl(uint8_t Index)
{
    if(Index > MAX_LED)
    {
        return;
    }

    switch(Index)
    {
        case WORK_LED:
        {
            WORK_LED_BLINK();
            break;
        }
        case NET_LED:
        {
            NET_LED_BLINK();
            break;
        }
        /*add user led*/
        default:break;
    }
}

/**
  @brief 设置led工作模式
  @para  p_LedConifgPara 包括工作模式，使能状态，闪烁频率，持续时间
  @retval 
*/
static int8_t led_SetWorkPara(StLedConfigPara_t *p_LedConfigPara)
{
    if((NULL == p_LedConfigPara) || (p_LedConfigPara->LedType > MAX_LED))
    {
        return ERR_FAULT;
    }
#if 0    
    if(LED_UNALIVE != stLedCtrlTable[p_LedConfigPara->LedType].LedEnableState)
    {
        printf("Led is running,set mode fail!\r\n");
        return ERR_FAULT;
    }
#endif
    stLedCtrlTable[p_LedConfigPara->LedType].LedEnableState = p_LedConfigPara->LedEnableState;
    stLedCtrlTable[p_LedConfigPara->LedType].LedWorkMode = p_LedConfigPara->LedWorkMode;
    stLedCtrlTable[p_LedConfigPara->LedType].LedBlinkFreq = p_LedConfigPara->LedBlinkFreq;
    stLedCtrlTable[p_LedConfigPara->LedType].u32Duration = p_LedConfigPara->u32Duration;
    
    return ERR_OK;
}

/**
* @brief user led callback
*/
static void led_TaskListHandle(uint8_t Index)
{
    switch(stLedCtrlTable[Index].LedWorkMode)
    {
        case LED_MODE_LIGHTON:
        {
            led_LightOnControl(Index);
            break;
        }
        case LED_MODE_LIGHTOFF:
        {
            led_Reset(Index);
            return;
        }
        case LED_MODE_BLINKING:
        {
            if(0 == (stLedCtrlTable[Index].u32LedCounter % LedFreqTab[stLedCtrlTable[Index].LedBlinkFreq]))
            {
                led_BlinkControl(Index);
                if(LED_ALWAYSALIVE == stLedCtrlTable[Index].LedEnableState)
                {
                    stLedCtrlTable[Index].u32LedCounter = 0;
                }
            }
            break;
        }
        default:
        {
            led_Reset(Index);
            return;
        }
    }
    
    stLedCtrlTable[Index].u32LedCounter++;
    
    if((LED_ALIVE == stLedCtrlTable[Index].LedEnableState) && \
        (stLedCtrlTable[Index].u32LedCounter > stLedCtrlTable[Index].u32Duration))
    {
        led_Reset(Index);
    }
}

/**
* @brief All led callback
*/
static void led_Led_cb(void)
{
    uint8_t LedIndex = 0; /*loop*/

    for(LedIndex = 0; LedIndex < MAX_LED; LedIndex++)
    {
        if(LED_UNALIVE == stLedCtrlTable[LedIndex].LedEnableState)
        {
            return;
        }
        
        led_TaskListHandle(LedIndex);
    }
}

/**
* @brief Timer6 callback
*/
void LedCtrlTimeoutCallback(void)
{
    led_Led_cb();
}
