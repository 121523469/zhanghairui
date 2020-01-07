#include "ADCDriver.h"
#include "adc.h"
#include "stm32l0xx_hal.h"
#include "SrvErrorNo.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"
#include <string.h>

#define ADC_COLLECT_TIMES    (10)    /**< ADC采样次数 */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   (EN_ADC_CHN_MAX * ADC_COLLECT_TIMES)     /**< ADC采样数据Buffer大小 */
StSrvIODevice_t stADC;

static uint32_t m_ADCConvertedData[ADC_COLLECT_TIMES * ADC_CONVERTED_DATA_BUFFER_SIZE];         /**< ADC采样数据Buffer */
StSrvIODevice_t stADCDevice;
/**************************************************************************************************
 * static function prototypes
 *************************************************************************************************/
static int16_t adc_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t adc_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t adc_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);
static uint32_t adc_AnalogVolChannel(EN_ADCCommand_t CHN_ID);
static uint32_t adc_TempSample(void);

/**************************************************************************************************
 * static variables
 *************************************************************************************************/
static StSrvIODeviceItem_t m_stADCDeviceItem;                   /**< 生产参数存储设备 */
static StSrvIOOperations_t m_stADCOperation = 
{
    .open = adc_open,                                            /**< 从链表中找到EEPROM设备 */
    .close = adc_close,                                          /**< 关闭已找到的EEPROM设备 */
    .ioctl = adc_ioctl,                                          /**< 读取数据 */
};

/** @brief 注册ADC
*/
void ADC_configure(void)
{
    /* 注册 ADC Device */
    SrvIODeviceItem_register(&m_stADCDeviceItem, EN_ID_ADC, &m_stADCOperation, IO_READ | IO_WRITE);
}

/** @brief Open ADC Device.
    @param[in] p_pstSrvIODeviceItem Device item
    @param[in] p_pstSrvIODevice     Device

    @retval _SRVERRORNO_H_
*/
int16_t adc_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{

    p_pstSrvIODevice->pvPrivateData = (void *)0;
    p_pstSrvIODevice->pstDeviceItem->u8Count++;

    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 1)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
        return ERR_BEYOND_MAX;
    }

    if (EN_ID_ADC == p_pstSrvIODeviceItem->enDeviceId)
    {
        p_pstSrvIODevice->pvPrivateData = (void *)0;
    }

    MX_DMA_Init();
    MX_ADC_Init();
    return ERR_OK;
}

/** @brief Close ADC Device

    @param[in] p_pstSrvIODeviceItem Device item
    @param[in] p_pstSrvIODevice     Device

    @retval _SRVERRORNO_H_
*/
static int16_t adc_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 0)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
    }
    p_pstSrvIODevice->pstDeviceItem = NULL;
    
    return ERR_OK;
}

/** @brief GPRS ioctl
    @param[in] p_pstSrvIODeviceItem 
    @param[in] p_pstSrvIODevice       GPRS Device
    @param[in] p_u32Command  --ADC CHNx
    @param[in] p_u32Arg--ADC value

    @retval _SRVERRORNO_H_
*/
#define DELAY_50_MS    (50)
int16_t adc_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    if(NULL == p_pstSrvIODeviceItem)
    {
        printf("adc_ioctl p_pstSrvIODeviceItem is NULL!\r\n");
        return ERR_NODEV;
    }

    if(NULL == p_pstSrvIODevice)
    {
        printf("adc_ioctl p_pstSrvIODevice is NULL!\r\n");
        return ERR_NODEV;
    }
    memset(m_ADCConvertedData, 0, sizeof(m_ADCConvertedData));
    HAL_ADC_Start_DMA(&hadc, m_ADCConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
    HAL_Delay(DELAY_50_MS);
    HAL_ADC_Stop_DMA(&hadc);
    if (EN_ADC_TEMP == (EN_ADCCommand_t)p_u32Command)
    {
        *(uint32_t *)p_u32Arg = adc_TempSample();
    }
    else
    {
        *(uint32_t *)p_u32Arg = adc_AnalogVolChannel((EN_ADCCommand_t)p_u32Command);
    }
    return ERR_OK;
}

/** @brief adc_AnalogVolChannel
    @retval return    CHNx ADC value
*/
static uint32_t adc_AnalogVolChannel(EN_ADCCommand_t CHN_ID)
{
    uint8_t i = 0;
    uint32_t ADCValue = 0;
    uint32_t ADCRefValue = 0;
    uint32_t Vref12 = (*VREFINT_CAL_ADDR);

    for( i = CHN_ID; i < ADC_CONVERTED_DATA_BUFFER_SIZE;)
    {
        ADCValue += m_ADCConvertedData[i];
        ADCRefValue += m_ADCConvertedData[i + EN_ADC_CHN_REF - CHN_ID];
        i += EN_ADC_CHN_MAX;
    }

    ADCValue =   ADCValue / ADC_COLLECT_TIMES;
    ADCRefValue = ADCRefValue / ADC_COLLECT_TIMES;

    ADCValue = (uint32_t)(30000.0 * Vref12 * ADCValue / ADCRefValue / 4095);

    return ADCValue;
}

/** @brief adc_TempSample
    @retval return    temperature 0.1℃
*/
static uint32_t adc_TempSample(void)
{
    uint8_t i = 0;
    int32_t ADCValue = 0;
    uint32_t ADCRefValue = 0;
    uint32_t Vref12 = (*VREFINT_CAL_ADDR);
    int32_t Temp = 0;
    int32_t TempCal1 = (*TEMPSENSOR_CAL1_ADDR);/*30度adc值*/
    int32_t TempCal2 = (*TEMPSENSOR_CAL2_ADDR);/*130度adc值*/

    for( i = EN_ADC_TEMP; i < ADC_CONVERTED_DATA_BUFFER_SIZE;)
    {
        ADCValue += m_ADCConvertedData[i];
        ADCRefValue += m_ADCConvertedData[i + EN_ADC_CHN_REF - EN_ADC_TEMP];
        i += EN_ADC_CHN_MAX;
    }

    ADCValue =   ADCValue / ADC_COLLECT_TIMES;
    ADCRefValue = ADCRefValue / ADC_COLLECT_TIMES;

    /*ADC修正*/
    ADCValue = (uint32_t)(Vref12 * ADCValue / ADCRefValue);
    //printf("ADCValue = %d, TempCal1 = %d, TempCal2 = %d!\r\n", ADCValue, TempCal1, TempCal2);

    Temp = 1000000 / (TempCal2 - TempCal1) * (ADCValue - TempCal1) + 300000;
    Temp /= 1000;
    printf("Cur temperature is %d!\r\n", Temp);
    return Temp;
}

