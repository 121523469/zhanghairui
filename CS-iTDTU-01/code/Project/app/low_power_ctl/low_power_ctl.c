#include "low_power_ctl.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "usart.h"
#include "lptim.h"
#include "nbiot.h"
#include "SrvIODevice.h"
#include "common_interface.h"
#include "duart.h"
#include "stm32l0xx_ll_lpuart.h"
#include "lorawan.h"
#include "SrvErrorNo.h"

/** moduler psm state Enum */
typedef enum
{
    EN_MODULER_PSM_STATE,
    EN_MODULER_NORMAL_STATE,
}EnModulerWorkMode_t;

EnModulerWorkMode_t GetModulerPsmState(void)
{
    int16_t ret = -1;
    uint32_t MuartState = 0;
#ifdef IOT_NET_TYPE_NBIOT
    ret = SrvIODevice_open(&stNBIOT,EN_ID_NBIOT,IO_READ | IO_WRITE);
    if ((ERR_OK != ret) && (ERR_BEYOND_MAX != ret))
    {
        printf("open stNBIOT fail,ret = %d!\r\n", ret);
    }

    SrvIODevice_ioctl(&stNBIOT, EN_CMD_PSM_STATE, (uint32_t)(&stSysParaData.u32SysLowPowerState));
    if (1 == stSysParaData.u32SysLowPowerState)
    {
        return EN_MODULER_NORMAL_STATE;
    }
    SrvIODevice_ioctl(&stNBIOT, EN_CMD_SET_MUART_STATE, (uint32_t)&MuartState);

    return EN_MODULER_PSM_STATE;
#endif

#ifdef IOT_NET_TYPE_LORAWAN
    SrvIODevice_open(&stLORAWAN,EN_ID_LORAWAN,IO_READ | IO_WRITE);
    SrvIODevice_ioctl(&stLORAWAN, EN_CMD_GET_LPSATAE, (uint32_t)(&stSysParaData.u32SysLowPowerState));
    if (1 == stSysParaData.u32SysLowPowerState)
    {
        return EN_MODULER_NORMAL_STATE;
    }
    SrvIODevice_ioctl(&stLORAWAN, EN_CMD_SET_MUART_STATE, (uint32_t)&MuartState);

    return EN_MODULER_PSM_STATE;
#endif
}

void Low_Power_Enter(void)
{
    if(EN_MODULER_NORMAL_STATE == GetModulerPsmState())
    {
        return;
    }
    
    printf("Enter low power mode!\r\n");
    SrvIODevice_close(&stADC);

    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();

    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp();

    /* Select HSI as system clock source after Wake Up from Stop mode */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

    HAL_ADC_DeInit(&hadc);
    __ADC1_CLK_DISABLE();
    __HAL_ADC_DISABLE(&hadc);

    __HAL_UART_DISABLE(&huart1);
    __USART1_CLK_DISABLE();

    GPIO_InitTypeDef GPIO_InitStructure;
    /* Enable GPIOs clock */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    GPIO_InitStructure.Pin = GPIO_PIN_All;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);


    /*debug io remain     on*/
    GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_13) & (~GPIO_PIN_14) & (~GPIO_PIN_5) & (~GPIO_PIN_6);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

#if 1
    GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_10) & (~GPIO_PIN_11) & (~GPIO_PIN_3) & (~GPIO_PIN_6);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif 

    GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_8) & (~GPIO_PIN_6) &(~GPIO_PIN_1) &(~GPIO_PIN_2) & (~GPIO_PIN_3);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    LL_LPUART_ClearFlag_WKUP(LPUART1);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void Low_Power_Exit(void)
{
    if(1 == stSysParaData.u32SysLowPowerState)
    {
        return;
    }
    SystemClock_Config();
    SysDriverInit();
    
    HAL_ADC_DeInit(&hadc);
    __HAL_ADC_ENABLE(&hadc);
    HAL_ADC_Init(&hadc);
    
    printf("Exit low power mode!\r\n");
    stSysParaData.u32SysLowPowerState = 1;
}
