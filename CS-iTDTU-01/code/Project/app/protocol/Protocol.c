#include "protocol.h"
#include "string.h"
#include "stm32l0xx_hal.h"
#include "SrvIODevice.h"
#include "ADCDriver.h"
#include "EEPROM.h"
#include "SrvErrorNo.h"
#include "common_interface.h"
#include "NBIOT.h"
#include "low_power_ctl.h"
#include "Lorawan.h"

static uint16_t Pro_RcvReadPowerOnReportCmd(uint8_t* p_pu8RcvBuffer);
static uint16_t Pro_SendResponseMsg(uint8_t p_u8FuncCode, uint8_t p_u8ErrCode, uint16_t p_u16QryMsgSn);
static uint16_t Pro_RcvConfigCmd(uint8_t* RcvBuffer);
static uint16_t Pro_DOControlCmd(uint8_t* RcvBuffer);

static uint16_t Pro_RcvResetCmd(uint8_t* RcvBuffer);
//static uint16_t Pro_RcvDefendCmd(uint8_t* RcvBuffer);
//static uint16_t Pro_RcvCancelAlmCmd(uint8_t* RcvBuffer);
static uint16_t Pro_RcvACKMsg(uint8_t* RcvBuffer);
static uint16_t Pro_SleepModeSetting(uint8_t* p_pu8RcvBuffer);
static uint16_t Pro_GetBaseMagnit(uint8_t* p_pu8RcvBuffer);
static uint16_t Pro_RestoreFactorySetting(uint8_t* p_pu8RcvBuffer);
static int16_t UpgradeDataACK(uint8_t p_u8FuncCode, uint8_t p_u8ErrCode, uint16_t p_u16QryMsgSn, uint16_t p_u16PktNum);
static int16_t Pro_UpgradeCmdFun(uint8_t* p_pu8RcvBuffer);
static int16_t Pro_UpgradeDataHandle(uint8_t* p_pu8RcvBuffer);


typedef struct 
{
    EnProFuncCode_t EnFuncCode;
    EnProCmdState_t EnProCmdState;
    pfProCmdHeader fpHeader;
}StProCmdTab_t;

StProCmdTab_t stProCmdTab[PRO_CMD_MAX] = 
{
    {EN_PRO_FUNCCODE_NULL_0, EN_PRO_CMD_STATE_NULL, NULL},/*null*/
    {EN_PRO_FUNCCODE_POWERON_REPORT, EN_PRO_CMD_STATE_NULL, NULL},
    {EN_PRO_FUNCCODE_TIMELY_REPORT, EN_PRO_CMD_STATE_NULL, NULL},
    {EN_PRO_FUNCCODE_CONFIG, EN_PRO_CMD_STATE_NULL, (pfProCmdHeader)Pro_RcvConfigCmd},
    {EN_PRO_FUNCCODE_RESET, EN_PRO_CMD_STATE_NULL, (pfProCmdHeader)Pro_RcvResetCmd},
    {EN_PRO_FUNCCODE_REV_1, EN_PRO_CMD_STATE_NULL, NULL},
    {EN_PRO_FUNCCODE_CANCEL_ALM, EN_PRO_CMD_STATE_NULL, NULL},
    {EN_PRO_FUNCCODE_READ_POWERON, EN_PRO_CMD_STATE_NULL, (pfProCmdHeader)Pro_RcvReadPowerOnReportCmd},
    {EN_PRO_FUNCCODE_NULL_8, EN_PRO_CMD_STATE_NULL, NULL},
    {EN_PRO_FUNCCODE_RESTORE, EN_PRO_CMD_STATE_NULL, (pfProCmdHeader)Pro_RestoreFactorySetting},
    {EN_PRO_FUNCCODE_CONTROL, EN_PRO_CMD_STATE_NULL, (pfProCmdHeader)Pro_DOControlCmd},
    {EN_PRO_FUNCCODE_ACK, EN_PRO_CMD_STATE_NULL, (pfProCmdHeader)Pro_RcvACKMsg},
};

static void ProSetCmdAckState(EnProFuncCode_t FuncCode, EnProCmdState_t State)
{
    stProCmdTab[FuncCode].EnProCmdState = State;
}

EnProCmdState_t ProGetCmdAckState(EnProFuncCode_t FuncCode)
{
    return stProCmdTab[FuncCode].EnProCmdState;
}

static uint8_t ProGetCmdIndex(EnProFuncCode_t FuncCode)
{
    return FuncCode;
}

/* CMD长度和数据长度的差值(数据长度是指除了ID和FUNcode及CRC以外的所有数据长度) */
#define CMD_DATA_LEN_DIFF    (10)

#define SET_DEVICE_ID(p_DevId)      do \
{\
    if(p_DevId != 0xFFFF)\
    {\
        stSysParaData.stUserSetData.u16DeviceID = p_DevId;\
    }\
} while(0)

#define SET_HEART_BEAT_INTEVAL(p_Inteval)         do \
{\
    if((p_Inteval != 0xFFFF) &&((p_Inteval <= 1440) && (p_Inteval >= 1)))\
    {\
        stSysParaData.stUserSetData.u16HeartBeatInteval = p_Inteval;\
    }\
} while(0)

#define SET_SAMPLE_INTEVAL(p_Inteval)       do \
{\
    if((p_Inteval != 0xFFFF) &&((p_Inteval <= 1440) && (p_Inteval >= 1)))\
    {\
        stSysParaData.stUserSetData.u16SampleInteval = p_Inteval;\
    }\
}while(0)

#define SET_PORT_NUM(p_u16Port)       do \
{\
    if(p_u16Port != 0xFFFF)\
    {\
        stSysParaData.stUserSetData.stNbiotCfg.u16Port = (p_u16Port);\
    }\
}while(0)

#define SET_IP_ADDR(p_u32IpAddr)       do \
{\
    if(p_u32IpAddr != 0xFFFFFFFF)\
    {\
        stSysParaData.stUserSetData.stNbiotCfg.u32IpAddr = p_u32IpAddr;\
    }\
}while(0)


/*************************************************************************************************/
/** @brief   cmd process
*/
/*************************************************************************************************/
void Pro_CmdParseHandle(uint8_t* p_pu8RcvBuffer)
{
    uint8_t index = 0;
    int16_t ret = PROTOCOL_ERR;
    StFrameHeader_t *pstFrameHeader;
    pstFrameHeader = (StFrameHeader_t *)p_pu8RcvBuffer;
    
    for(index = 0; index < PRO_CMD_MAX; index++)
    {
        if(pstFrameHeader->u8FuncCode == stProCmdTab[index].EnFuncCode)
        {
            break;
        }
    }

    if ((PRO_CMD_MAX == index ) || (PRO_CMD_MAX < index ))
    {
        printf("Pro_CmdParseHandle FuncCode is invalid,u8FuncCode = %d!\r\n", pstFrameHeader->u8FuncCode);
        return;
    }

    if(EN_PRO_FUNCCODE_ACK != pstFrameHeader->u8FuncCode)
    {
        ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_WAIT_ACK);
    }

    if (NULL == stProCmdTab[index].fpHeader)
    {
        printf("Pro_CmdParseHandle fpHeader is null,u8FuncCode = %d!\r\n", pstFrameHeader->u8FuncCode);
        return;
    }
    
    ret = stProCmdTab[index].fpHeader(p_pu8RcvBuffer);
    if(PROTOCOL_ERR == ret)
    {
        ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_ACK_ERR);
    }

}

/** @brief  Device ID verify
*/
static int8_t Pro_DeviceIdVerify(uint16_t u16DevId)
{
    /*  通用ID直接返回ok */
    if (0 == u16DevId)
    {
        printf("Pro_DeviceIdVerify rev devid is 0!\r\n");
        return PROTOCOL_OK;
    }

    if (u16DevId != stSysParaData.stUserSetData.u16DeviceID)
    {
        printf("Pro_DeviceIdVerify, DeviceId verify err,rev DevId is %d, DeviceId is %d\r\n",\
            u16DevId, stSysParaData.stUserSetData.u16DeviceID);
        return PROTO_ERRCODE_PARAM;
    }

    return PROTOCOL_OK;
}

/** @brief   Updata Device ID
*/
int8_t Pro_UpdateDeviceId(uint16_t u16DevId)
{
    /*  如果收到通用ID不更新 */
    if (0 == u16DevId)
    {
        printf("Pro_UpdateDeviceId rev devid is 0\r\n");
        return PROTO_ERRCODE_PARAM;
    }

    if(0xFFFF == u16DevId)
    {
        //printf("Pro_UpdateDeviceId device id invalid,u16DevId = %d!\r\n", u16DevId);
        return PROTOCOL_OK;
    }

    if (u16DevId != stSysParaData.stUserSetData.u16DeviceID)
    {
        /* set Device ID*/
        SET_DEVICE_ID(u16DevId);
    }
    return PROTOCOL_OK;
}

/** @brief Update timely report interval
*/
int8_t Pro_UpdateHeartBeatInteval(uint16_t p_u16Inteval)
{
    /* Port number set，FFFF时表示保持当前设置不变 */
    if(p_u16Inteval != 0xFFFF)
    {
        if((p_u16Inteval < 1) || (p_u16Inteval > 1440))
        {
            printf("Pro_UpdateHeartBeatInteval u16Inteval err,p_u16Inteval = %d\r\n", p_u16Inteval);
            return PROTO_ERRCODE_PARAM;
        }
        else
        {
            SET_HEART_BEAT_INTEVAL(p_u16Inteval);
        }
    }

    return PROTOCOL_OK;
}

/** @brief Update timely report interval
*/
int8_t Pro_UpdateSampleInteval(uint16_t p_u16Inteval)
{
    /* Port number set，FFFF时表示保持当前设置不变 */
    if(p_u16Inteval != 0xFFFF)
    {
        if((p_u16Inteval < 1) || (p_u16Inteval > 1440))
        {
            printf("Pro_UpdateSampleInteval u16Inteval err,p_u16Inteval = %d\r\n", p_u16Inteval);
            return PROTO_ERRCODE_PARAM;
        }
        else
        {
            SET_SAMPLE_INTEVAL(p_u16Inteval);
        }
    }

    return PROTOCOL_OK;
}


#ifdef IOT_NET_TYPE_NBIOT
/** @brief Update port num
*/
int8_t Pro_UpdatePortNum(uint16_t p_u16Port)
{
    /* Port number set，FFFF时表示保持当前设置不变 */
    if(p_u16Port != 0xFFFF)
    {
        if(0 == p_u16Port)
        {
            printf("Pro_UpdatePortNum u16Port err,p_u16Port = %d\r\n", p_u16Port);
            return PROTO_ERRCODE_PARAM;
        }
        else
        {
            SET_PORT_NUM(p_u16Port);
        }
    }
    return PROTOCOL_OK;
}

/** @brief Update IP addr
*/
int8_t Pro_UpdateIpAddr(uint32_t p_u32IpAddr)
{
    /* Ip address setting， FFFFFFFF时表示保持当前设置不变 */
    if(0xFFFFFFFF != p_u32IpAddr)
    {
        if(0 == p_u32IpAddr)
        {
            printf("Pro_UpdateIpAddr u32IpAddr err,p_u32IpAddr = %d\r\n", p_u32IpAddr);
            return PROTO_ERRCODE_PARAM;
        }
        else
        {
            SET_IP_ADDR(p_u32IpAddr);
        }
    }

    return PROTOCOL_OK;
}
#endif

/** @brief Config cmd process
*/
static uint16_t Pro_RcvConfigCmd(uint8_t* p_pu8RcvBuffer)
{
    uint8_t u8ErrCode = PROTO_ERRCODE_INTERAL;
    StUserSetData_t p_stUserSetData;
    int16_t ret = -1;
    StConfigureCmd_t *pstConfigCmd;
    pstConfigCmd = (StConfigureCmd_t *)p_pu8RcvBuffer;
    uint8_t LenWithoutCrc = sizeof(StConfigureCmd_t) - 2;
    uint8_t DataLength = LenWithoutCrc - sizeof(StFrameHeader_t);
    
    memset(&p_stUserSetData, 0, sizeof(StUserSetData_t));
    memcpy((char*)&p_stUserSetData, (char*)&stSysParaData.stUserSetData, sizeof(StUserSetData_t));
    /* CRC校验 */
    if(pstConfigCmd->u16CRC != CRC16(p_pu8RcvBuffer, LenWithoutCrc))
    {
        u8ErrCode = PROTO_ERRCODE_CRC;
        //printf("Pro_RcvConfigCmd CRC check err!\r\n");
        goto EXIT;
    }

    if(DataLength != pstConfigCmd->stFrameHeader.u16DataLength)
    {
        u8ErrCode = PROTO_ERRCODE_PARAM;
        printf("Pro_RcvConfigCmd frame data lenght err,len is %d!\r\n", pstConfigCmd->stFrameHeader.u16DataLength);
        goto EXIT;
    }

    /* DeviceId Update */
    u8ErrCode = Pro_UpdateDeviceId(pstConfigCmd->u16NewDeviceId);
    if (PROTOCOL_OK != u8ErrCode)
    {
        printf("Pro_RcvConfigCmd device id verify err!\r\n");
        goto EXIT;
    }

    /* Timely repeat internal， FFFF时表示保持当前设置不变 */
    u8ErrCode = Pro_UpdateHeartBeatInteval(pstConfigCmd->u16HeartBeatInteval);
    if (PROTOCOL_OK != u8ErrCode)
    {
        printf("Pro_RcvConfigCmd u16HeartBeatInteval set err!\r\n");
        goto EXIT;
    }

    u8ErrCode = Pro_UpdateSampleInteval(pstConfigCmd->u16SampleInteval);
    if (PROTOCOL_OK != u8ErrCode)
    {
        printf("Pro_RcvConfigCmd u16SampleInteval set err!\r\n");
        goto EXIT;
    }

#ifdef IOT_NET_TYPE_NBIOT
    /* Port number set，FFFF时表示保持当前设置不变 */
    u8ErrCode = Pro_UpdatePortNum(pstConfigCmd->u16Port);
    if (PROTOCOL_OK != u8ErrCode)
    {
        printf("Pro_RcvConfigCmd u16AlarmInteval set err!\r\n");
        goto EXIT;
    }

    /* Ip address setting， FFFFFFFF时表示保持当前设置不变 */
    u8ErrCode = Pro_UpdateIpAddr(pstConfigCmd->u32IpAddr);
    if (PROTOCOL_OK != u8ErrCode)
    {
        printf("Pro_RcvConfigCmd u32IpAddr set err!\r\n");
        goto EXIT;
    }
#endif
    u8ErrCode = Pro_UpdateCfgPayload(pstConfigCmd->stPayload);
    if (PROTOCOL_OK != u8ErrCode)
    {
        printf("Pro_RcvConfigCmd Pro_UpdateCfgPayload err u8ErrCode = %d!\r\n", u8ErrCode);
        goto EXIT;
    }

    /* 写入用户配置参数到 User EEPROM Storage */
    ret = SaveUserSetData(stSysParaData.stUserSetData);
    if(0 != ret)
    {
        printf("Pro_RcvConfigCmd save user para fail,ret = %d!\r\n", ret);
        u8ErrCode = ret;
    }

EXIT:
    if(PROTOCOL_OK != u8ErrCode)
    {
        memcpy((char*)&stSysParaData.stUserSetData, (char*)&p_stUserSetData, sizeof(StUserSetData_t));
    }
    /* 回复云端 */
    Pro_SendResponseMsg(EN_PRO_FUNCCODE_CONFIG, u8ErrCode, pstConfigCmd->stFrameHeader.u16MsgSn);
    return u8ErrCode;
}

/** @brief Reset cmd process
*/
static uint16_t Pro_RcvResetCmd(uint8_t* p_pu8RcvBuffer)
{
    uint16_t ret = PROTOCOL_ERR;

    StResetCmd_t *pstResetCmd;
    pstResetCmd = (StResetCmd_t *)p_pu8RcvBuffer;
    uint8_t LenWithoutCrc = sizeof(StResetCmd_t) - 2;
    uint8_t DataLength = LenWithoutCrc - sizeof(StFrameHeader_t);

    stSysParaData.u16DownlinkMsgSn++;

    /* CRC校验 */
    if(pstResetCmd->u16CRC != CRC16(p_pu8RcvBuffer,  LenWithoutCrc))
    {
        printf("Pro_RcvResetCmd CRC check err!\r\n");
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET,\
            PROTO_ERRCODE_CRC, pstResetCmd->stFrameHeader.u16MsgSn);
        return PROTO_ERRCODE_CRC;
    }

    if(DataLength != pstResetCmd->stFrameHeader.u16DataLength)
    {
        printf("Pro_RcvResetCmd frame data lenght err,len is %d!\r\n", pstResetCmd->stFrameHeader.u16DataLength);
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET,\
            PROTO_ERRCODE_PARAM, pstResetCmd->stFrameHeader.u16MsgSn);

        return PROTO_ERRCODE_PARAM;
    }

    ret = Pro_DeviceIdVerify(pstResetCmd->stFrameHeader.u16DeviceId);
    if (PROTOCOL_OK != ret)
    {
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET, PROTO_ERRCODE_PARAM,\
            pstResetCmd->stFrameHeader.u16MsgSn);

        printf("Pro_RcvResetCmd device id check err!\r\n");
        return ret;
    }

    /* Reset MCU */
    Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET, ret,\
        pstResetCmd->stFrameHeader.u16MsgSn);
    printf("Rcv reset cmd,system is rebooting...\r\n");
    delay_ms(3000);
    NVIC_SystemReset();
    return ret;
}

static uint16_t Pro_DOControlCmd(uint8_t* RcvBuffer)
{
    uint16_t ret = PROTOCOL_ERR;

    StDOControlCmd_t *pstDOControlCmd;
    pstDOControlCmd = (StDOControlCmd_t *)RcvBuffer;
    
    uint8_t LenWithoutCrc = sizeof(StDOControlCmd_t) - 2;
    uint8_t DataLength = LenWithoutCrc - sizeof(StFrameHeader_t);

    stSysParaData.u16DownlinkMsgSn++;

    /* CRC校验 */
    if(pstDOControlCmd->u16CRC != CRC16(RcvBuffer,  LenWithoutCrc))
    {
        printf("Pro_DOControlCmd CRC check err!\r\n");
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET,\
            PROTO_ERRCODE_CRC, pstDOControlCmd->stFrameHeader.u16MsgSn);
        return PROTO_ERRCODE_CRC;
    }

    if(DataLength != pstDOControlCmd->stFrameHeader.u16DataLength)
    {
        printf("Pro_DOControlCmd frame data lenght err,len is %d!\r\n", pstDOControlCmd->stFrameHeader.u16DataLength);
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET,\
            PROTO_ERRCODE_PARAM, pstDOControlCmd->stFrameHeader.u16MsgSn);

        return PROTO_ERRCODE_PARAM;
    }

    ret = Pro_DeviceIdVerify(pstDOControlCmd->stFrameHeader.u16DeviceId);
    if (PROTOCOL_OK != ret)
    {
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET, PROTO_ERRCODE_PARAM,\
            pstDOControlCmd->stFrameHeader.u16MsgSn);

        printf("Pro_DOControlCmd device id check err!\r\n");
        return ret;
    }
    
    if(0 == pstDOControlCmd->u8ControlStatus)
    {
        IOChannel.DOChannelOutput(EN_DO_RESET);
    }
    else if(1 == pstDOControlCmd->u8ControlStatus)
    {
        IOChannel.DOChannelOutput(EN_DO_SET);
    }
    else
    {
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET, PROTO_ERRCODE_PARAM,\
            pstDOControlCmd->stFrameHeader.u16MsgSn);
        printf("Invalid digital output status, pstDOControlCmd->u8ControlStatus is %d\r\n", pstDOControlCmd->u8ControlStatus);
        return PROTO_ERRCODE_PARAM;
    }

    /* Reset MCU */
    Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESET, ret,\
        pstDOControlCmd->stFrameHeader.u16MsgSn);
    
    return ret;
}


/** @brief   restore factory setting.
*/
static uint16_t Pro_RestoreFactorySetting(uint8_t* p_pu8RcvBuffer)
{
    uint16_t  Ret = PROTOCOL_ERR;
    StRestoreCmd_t *pstRestoreCmd;
    pstRestoreCmd = (StRestoreCmd_t *)p_pu8RcvBuffer;
    uint8_t LenWithoutCrc = sizeof(StRestoreCmd_t) - 2;
    uint8_t DataLength = LenWithoutCrc - sizeof(StFrameHeader_t);
    //stSysParaData.u16DownlinkMsgSn++;

    /* CRC校验 */
    if(pstRestoreCmd->u16CRC != CRC16(p_pu8RcvBuffer, LenWithoutCrc))
    {
        /* 回复云端 */
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESTORE, PROTO_ERRCODE_CRC,\
            pstRestoreCmd->stFrameHeader.u16MsgSn);
        printf("Pro_RestoreFactorySetting CRC check error.\r\n");
        return PROTO_ERRCODE_CRC;
    }

    if(DataLength != pstRestoreCmd->stFrameHeader.u16DataLength)
    {
        printf("Pro_RestoreFactorySetting frame data lenght err,len is %d!\r\n", pstRestoreCmd->stFrameHeader.u16DataLength);
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESTORE,\
            PROTO_ERRCODE_PARAM, pstRestoreCmd->stFrameHeader.u16MsgSn);

        return PROTO_ERRCODE_PARAM;
    }

    Ret = Pro_DeviceIdVerify(pstRestoreCmd->stFrameHeader.u16DeviceId);
    if (PROTOCOL_OK != Ret)
    {
        printf("Pro_RestoreFactorySetting devid check err!\r\n");
        /* 回复云端 */
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESTORE, PROTO_ERRCODE_PARAM,\
            pstRestoreCmd->stFrameHeader.u16MsgSn);
        return Ret;
    }

    Ret = RestorFactoryPara();
    if(ERR_OK != Ret)
    {
        printf("Pro_RestoreFactorySetting restore err,Ret=%d!\r\n", Ret);
    }

    /* 回复云端 */
    Pro_SendResponseMsg(EN_PRO_FUNCCODE_RESTORE, Ret,\
        pstRestoreCmd->stFrameHeader.u16MsgSn);

    printf("Protocol Restore Factory para ok,system is rebooting...\r\n");
    delay_ms(3000);
    NVIC_SystemReset();

    return Ret;
}

static uint16_t Pro_RcvReadPowerOnReportCmd(uint8_t* p_pu8RcvBuffer)
{
    int16_t ret = PROTOCOL_ERR;
    StReadPowerOnReportCmd_t *pstRdPowerOnReportCmd;
    pstRdPowerOnReportCmd = (StReadPowerOnReportCmd_t *)p_pu8RcvBuffer;
    uint8_t LenWithoutCrc = sizeof(StReadPowerOnReportCmd_t) - 2;
    uint8_t DataLength = LenWithoutCrc - sizeof(StFrameHeader_t);


    /* CRC校验 */
    if(pstRdPowerOnReportCmd->u16CRC != CRC16(p_pu8RcvBuffer, LenWithoutCrc))
    { 
        ret = PROTO_ERRCODE_CRC;
        printf("Pro_RcvReadPowerOnReportCmd CRC check error.\r\n");
        /* 回复 */
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_READ_POWERON, PROTO_ERRCODE_CRC,\
            pstRdPowerOnReportCmd->stFrameHeader.u16MsgSn);
        return ret;
    }

    if(DataLength != pstRdPowerOnReportCmd->stFrameHeader.u16DataLength)
    {
        printf("Pro_RcvReadPowerOnReportCmd frame data lenght err,len is %d!\r\n", pstRdPowerOnReportCmd->stFrameHeader.u16DataLength);
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_READ_POWERON,\
            PROTO_ERRCODE_PARAM, pstRdPowerOnReportCmd->stFrameHeader.u16MsgSn);

        return PROTO_ERRCODE_PARAM;
    }

    ret = Pro_DeviceIdVerify(pstRdPowerOnReportCmd->stFrameHeader.u16DeviceId);
    if (PROTOCOL_OK != ret)
    {
        printf("Pro_RcvReadPowerOnReportCmd devid check err!\r\n");
        /* 回复云端 */
        Pro_SendResponseMsg(EN_PRO_FUNCCODE_READ_POWERON, PROTO_ERRCODE_PARAM,\
            pstRdPowerOnReportCmd->stFrameHeader.u16MsgSn);
        return ret;
    }

    /* 回复 */
    ret = Pro_SendPowerOnReportFrame(EN_PRO_FUNCCODE_READ_POWERON,\
        pstRdPowerOnReportCmd->stFrameHeader.u16MsgSn);

    return ret;
}

/** @brief ACK msg process
*/
static uint16_t Pro_RcvACKMsg(uint8_t* p_pu8RcvBuffer)
{
    StACKFrame_t *pstACKCmd;
    uint8_t index = 0;
    uint8_t LenWithoutCrc = sizeof(StACKFrame_t) - 2;
    uint8_t DataLength = LenWithoutCrc - sizeof(StFrameHeader_t);
    pstACKCmd = (StACKFrame_t *)p_pu8RcvBuffer;
    /* CRC校验 */
    if(pstACKCmd->u16CRC != CRC16(p_pu8RcvBuffer, LenWithoutCrc))
    { 
        printf("Pro_RcvACKMsg CRC check error.\r\n");
        return PROTO_ERRCODE_CRC;
    }

    if(DataLength != pstACKCmd->stFrameHeader.u16DataLength)
    {
        printf("Pro_RcvACKMsg frame data lenght err,len is %d!\r\n", pstACKCmd->stFrameHeader.u16DataLength);
        return PROTO_ERRCODE_PARAM;
    }

    index = ProGetCmdIndex((EnProFuncCode_t)pstACKCmd->u8ResponseFuncCode);

    ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_ACK_OK);
    return PROTOCOL_OK;
}

/** @brief Response msg to cloud
    @param[in]p_u8ErrCode
*/
static uint16_t Pro_SendResponseMsg(uint8_t p_u8FuncCode, uint8_t p_u8ErrCode, uint16_t p_u16QryMsgSn)
{
    StACKFrame_t ACKFrame_t;
    int8_t index = PRO_CMD_MAX;
    int16_t Ret = -1;
    memset(&ACKFrame_t, 0, sizeof(ACKFrame_t));
    ACKFrame_t.stFrameHeader.u16MsgSn = p_u16QryMsgSn;
    index = ProGetCmdIndex((EnProFuncCode_t)p_u8FuncCode);
    ACKFrame_t.stFrameHeader.u8ProtocolVer = PROTOCO_VER;

    /* Device ID */
    ACKFrame_t.stFrameHeader.u16DeviceId = stSysParaData.stUserSetData.u16DeviceID;

    /* Function Code */
    ACKFrame_t.stFrameHeader.u8FuncCode  = EN_PRO_FUNCCODE_ACK;

    /* Data Length */
    ACKFrame_t.stFrameHeader.u16DataLength = sizeof(ACKFrame_t) - CMD_DATA_LEN_DIFF;

    /* Error code */
    ACKFrame_t.u8ErrCode = p_u8ErrCode;
    ACKFrame_t.u8ResponseFuncCode = p_u8FuncCode;

    /* CRC Check */
    ACKFrame_t.u16CRC = CRC16((uint8_t *)&ACKFrame_t,sizeof(ACKFrame_t) - 2);

    /* 回复云端 */
    Ret = ModulerSendResponseMsg((int8_t *)&ACKFrame_t, sizeof(ACKFrame_t));
    if(0 != Ret)
    {
        ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_ACK_ERR);
    }

    ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_ACK_OK);
    return Ret;
}

/** @brief Power on report process
*/
uint16_t Pro_SendPowerOnReportFrame(EnProFuncCode_t enFuncCode, uint16_t u16MsgSn)
{
    StPoweronFrame_t stPoweronFrame;
    int16_t Ret = -1;
    int8_t index = ProGetCmdIndex(enFuncCode);

    memset(&stPoweronFrame, 0, sizeof(stPoweronFrame));

    stPoweronFrame.stFrameHeader.u8ProtocolVer = PROTOCO_VER;
    /* 设备ID */
    stPoweronFrame.stFrameHeader.u16DeviceId = stSysParaData.stUserSetData.u16DeviceID;

    if(EN_PRO_FUNCCODE_POWERON_REPORT ==enFuncCode)
    {
        stPoweronFrame.stFrameHeader.u16MsgSn = stSysParaData.u16UplinkMsgSn;
        stSysParaData.u16UplinkMsgSn++;
    }
    else
    {
        stPoweronFrame.stFrameHeader.u16MsgSn = u16MsgSn;
    }

    stPoweronFrame.stFrameHeader.u8FuncCode = EN_PRO_FUNCCODE_POWERON_REPORT;

    /* 数据长度是指除了ID和FUNcode及CRC以外的所有数据长度 */
    stPoweronFrame.stFrameHeader.u16DataLength = sizeof(stPoweronFrame) - CMD_DATA_LEN_DIFF;

    /* 产品SN */
    stPoweronFrame.u32DeviceSn = stSysParaData.stFactorySetData.u32DeviceSN;

    /* 设备类型根据不同的设备选择 */
    stPoweronFrame.u8DeviceType = DEVICE_TYPE;

    /* 硬件版本 */
    stPoweronFrame.u8HWVersion = stSysParaData.stFactorySetData.u8HWVersion;

    /* 软件版本 */
    stPoweronFrame.stFWversion.u8Release = RELEASE_SW_VER;
    stPoweronFrame.stFWversion.u8MinorVersion = MINOR_SW_VER;
    stPoweronFrame.stFWversion.u8MajorVerison = MAJOR_SW_VER;
    stPoweronFrame.stFWversion.u8Reserve = RESERVE_SW_VER;

    /* 定时上报时间间隔 */
    stPoweronFrame.u16HeartBeatInteval = stSysParaData.stUserSetData.u16HeartBeatInteval;

    stPoweronFrame.u16SampleInteval = stSysParaData.stUserSetData.u16SampleInteval;

    Pro_GetPowerOnPayload(&stPoweronFrame.stPayload);

    /* CRC校验 */
    stPoweronFrame.u16CRC = CRC16((uint8_t *)&stPoweronFrame,(sizeof(stPoweronFrame) - 2));

    /* 云端发送 */
    if(EN_PRO_FUNCCODE_POWERON_REPORT == enFuncCode)
    {
        printf("PowerOn report\r\n");
        ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_WAIT_ACK);
        Ret = ModulerSendMsg((int8_t *)&stPoweronFrame, sizeof(stPoweronFrame));
        if(0 != Ret)
        {
            printf("Pro_SendPowerOnReportFrame send msg fail, Ret=%d!\r\n", Ret);
            return Ret;
        }
    }
    else if(EN_PRO_FUNCCODE_READ_POWERON == enFuncCode)
    {
        printf("Read PowerOn report\r\n");
        ProSetCmdAckState(EN_PRO_FUNCCODE_POWERON_REPORT, EN_PRO_CMD_STATE_WAIT_ACK);
        Ret = ModulerSendResponseMsg((int8_t *)&stPoweronFrame, sizeof(stPoweronFrame));
        if(0 != Ret)
        {
            printf("Pro_SendPowerOnReportFrame send msg fail, Ret=%d!\r\n", Ret);
            ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_ACK_ERR);
            return Ret;
        }
        ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_ACK_OK);
    }

    return Ret;
}

/** @brief Timely report process
*/
int16_t Pro_SendRegularUpload(void)
{
    StRegularUpload_t stRegularUpload_t;
    int16_t Ret = -1;
    int8_t index = PRO_CMD_MAX;

    index = ProGetCmdIndex(EN_PRO_FUNCCODE_TIMELY_REPORT);

    memset(&stRegularUpload_t, 0, sizeof(stRegularUpload_t));

    stRegularUpload_t.stFrameHeader.u8ProtocolVer = PROTOCO_VER;

    /* 功能码 */
    stRegularUpload_t.stFrameHeader.u8FuncCode    = EN_PRO_FUNCCODE_TIMELY_REPORT;

    /* 设备ID */
    stRegularUpload_t.stFrameHeader.u16DeviceId = stSysParaData.stUserSetData.u16DeviceID;

    stRegularUpload_t.stFrameHeader.u16MsgSn         = stSysParaData.u16UplinkMsgSn;
    stSysParaData.u16UplinkMsgSn++;

    /* 数据长度是指除了ID和FUNcode及CRC以外的所有数据长度 */
    stRegularUpload_t.stFrameHeader.u16DataLength   = sizeof(stRegularUpload_t) - CMD_DATA_LEN_DIFF;

    stRegularUpload_t.u32DeviceSn = stSysParaData.stFactorySetData.u32DeviceSN;

    stRegularUpload_t.u16DeviceState = stSysParaData.u16DeviceState;

    /* 电池剩余容量 */
    stRegularUpload_t.u8BatteryLeftVol = stSysParaData.u8BatteryLeftPercent;

    /* 信号强度 */
    stRegularUpload_t.s32Rssi = stSysParaData.s32Rssi;

    Pro_GetRegularPayload(&stRegularUpload_t.stPayload);

    /* CRC校验 */
    stRegularUpload_t.u16CRC = CRC16((uint8_t *)&stRegularUpload_t, sizeof(stRegularUpload_t) - 2);

    /* 云端发送 */
    ProSetCmdAckState((EnProFuncCode_t)index, EN_PRO_CMD_STATE_WAIT_ACK);
    Ret = ModulerSendMsg((int8_t *)&stRegularUpload_t, sizeof(stRegularUpload_t));
    if(0 != Ret)
    {
        printf("Pro_SendRegularUpload send msg fail, Ret=%d!\r\n", Ret);
        return Ret;
    }

    return Ret;
}

#if 1
uint16_t CRC16(uint8_t *dataIn,uint32_t length)
{
    uint32_t IX = 0;
    uint16_t IY = 0;
    uint16_t CRC_Result = 0xFFFF;
    
    if (length <= 0) 
    {
        CRC_Result = 0;
    }
    else 
    {
        length--;
        for (IX=0;IX<=length;IX++)
        {
            CRC_Result=CRC_Result^(uint16_t)(dataIn[IX]);
            
            for(IY=0;IY<=7;IY++)
            {
                if((CRC_Result&1)!=0) 
                {
                    CRC_Result=(CRC_Result>>1)^(0xA001);
                }
                else
                {
                    CRC_Result=CRC_Result>>1;
                }
            }             
        }
    }
    
    return CRC_Result;
}
#endif
#if 0
static uint16_t const CRC16Table[256] = {   
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t CRC16(volatile uint8_t *puchMsg, uint16_t usDataLen)
{
	uint16_t uCRC16 = 0xffff;
	uint16_t uIndex = 0;
	uint16_t i = 0;
	for(i = 0;i < usDataLen;i++)
	{
		uIndex = ((uCRC16 & 0xff)^(puchMsg[i] & 0xff));
		uCRC16 = ((uCRC16>>8) & 0xff)^CRC16Table[uIndex];
	}
	return (uCRC16>>8) | (uCRC16<<8);
}
#endif

