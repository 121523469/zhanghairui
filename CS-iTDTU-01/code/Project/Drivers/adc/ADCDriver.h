#ifndef __ADC_DRIVER_H
#define __ADC_DRIVER_H

#include "SrvIODevice.h"
#include "stm32l0xx_ll_adc.h"
typedef enum
{
    EN_ADC_CHN_VOL_BAT,             /**< 采集电池电压 */
    EN_ADC_CHN_VOL_CH1,
    EN_ADC_CHN_VOL_CH2,
    EN_ADC_CHN_REF,                 /*ref chan 17*/
    EN_ADC_TEMP,                    /*温度采集*/
    EN_ADC_CHN_MAX
}EN_ADCCommand_t;

extern StSrvIODevice_t stADC;

extern StSrvIODevice_t stADCDevice;
extern void ADC_configure(void);
#endif
