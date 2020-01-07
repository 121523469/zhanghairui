#include "protocol.h"
#include "product_protocol.h"
#include "string.h"
#include "stm32l0xx_hal.h"
#include "SrvIODevice.h"
#include "ADCDriver.h"
#include "EEPROM.h"
#include "SrvErrorNo.h"
#include "common_interface.h"
#include "NBIOT.h"
#include "IOChannelManage.h"

/** @brief Update cfg payload
*/
int8_t Pro_UpdateCfgPayload(StConfigureCmdPayload_t stcfgPayload)
{
    if((0xFFFF != stcfgPayload.u16AICH1LowLevel) && (0xFFFF != stcfgPayload.u16AICH1HighLevel))
    {
        if(EN_AI_TYPE_VOL == (stSysParaData.u8AnalogType & (1 << EN_TYPE_AI1)))
        {
            if((stcfgPayload.u16AICH1LowLevel < stcfgPayload.u16AICH1HighLevel) && (stcfgPayload.u16AICH1HighLevel < (AICH_VOLTAGE_MAX * 1000)))
            {
                stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel = stcfgPayload.u16AICH1LowLevel;
                stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel= stcfgPayload.u16AICH1HighLevel;
            }
            else
            {
                return PROTOCOL_ERR;
            }
        }
        else
        {
            if((stcfgPayload.u16AICH1LowLevel < stcfgPayload.u16AICH1HighLevel) && (stcfgPayload.u16AICH1HighLevel < (AICH_CURRENT_MAX * 1000)))
            {
                stSysParaData.stUserSetData.stCfgPayload.u16AICH1LowLevel = stcfgPayload.u16AICH1LowLevel;
                stSysParaData.stUserSetData.stCfgPayload.u16AICH1HighLevel= stcfgPayload.u16AICH1HighLevel;
            }
            else
            {
                return PROTOCOL_ERR;
            }
        }
    }

    if((0xFFFF != stcfgPayload.u16AICH2LowLevel) && (0xFFFF != stcfgPayload.u16AICH2HighLevel))
    {
        if(EN_AI_TYPE_VOL == (stSysParaData.u8AnalogType & (1 << EN_TYPE_AI2)))
        {
            if((stcfgPayload.u16AICH2LowLevel < stcfgPayload.u16AICH2HighLevel) && (stcfgPayload.u16AICH2HighLevel < (AICH_VOLTAGE_MAX * 1000)))
            {
                stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel = stcfgPayload.u16AICH2LowLevel;
                stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel= stcfgPayload.u16AICH2HighLevel;
            }
            else
            {
                return PROTOCOL_ERR;
            }
        }
        else
        {
            if((stcfgPayload.u16AICH2LowLevel < stcfgPayload.u16AICH2HighLevel) && (stcfgPayload.u16AICH2HighLevel < (AICH_CURRENT_MAX * 1000)))
            {
                stSysParaData.stUserSetData.stCfgPayload.u16AICH2LowLevel = stcfgPayload.u16AICH2LowLevel;
                stSysParaData.stUserSetData.stCfgPayload.u16AICH2HighLevel= stcfgPayload.u16AICH2HighLevel;
            }
            else
            {
                return PROTOCOL_ERR;
            }
        }
    }

    return PROTOCOL_OK;
}

/** @brief   Get Power On Payload.
*/
int16_t Pro_GetPowerOnPayload(StPoweronFramePayload_t *stPowerOnPayload)
{
#ifdef IOT_NET_TYPE_NBIOT
    memcpy(stPowerOnPayload->u8Imei, stSysParaData.stFactorySetData.u8Imei, sizeof(stPowerOnPayload->u8Imei));
    memcpy(stPowerOnPayload->u8Imsi, stSysParaData.stFactorySetData.u8Imsi, sizeof(stPowerOnPayload->u8Imsi));
#endif
    stPowerOnPayload->stCfgPayload = stSysParaData.stUserSetData.stCfgPayload;

    stPowerOnPayload->u8AnalogType = stSysParaData.u8AnalogType;
    return PROTOCOL_OK;
}

/** @brief Get regular report payload
*/
int16_t Pro_GetRegularPayload(StRegularUploadPayload_t *pstRegularPayload)
{
#ifdef IOT_NET_TYPE_NBIOT
    /* 无线模块特殊字段 */
    pstRegularPayload->unSpecData.stNBSpecData.u8ECL = \
        stSysParaData.unSpecData.stNBSpecData.u8ECL;
    pstRegularPayload->unSpecData.stNBSpecData.u8SNR = \
        stSysParaData.unSpecData.stNBSpecData.u8SNR;
    pstRegularPayload->unSpecData.stNBSpecData.u16PCI = \
        stSysParaData.unSpecData.stNBSpecData.u16PCI;
    pstRegularPayload->unSpecData.stNBSpecData.u32cellID = \
        stSysParaData.unSpecData.stNBSpecData.u32cellID;
#endif
    pstRegularPayload->u16AI1SampleValue = stSysParaData.u16AICH1Value;
    pstRegularPayload->u16AI2SampleValue = stSysParaData.u16AICH2Value;
    pstRegularPayload->u8DI1Level = stSysParaData.u8DICH1Status;
    pstRegularPayload->u8DI2Level = stSysParaData.u8DICH2Status;
    return PROTOCOL_OK;
}

