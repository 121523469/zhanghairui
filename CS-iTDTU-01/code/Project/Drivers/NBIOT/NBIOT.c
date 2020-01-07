#include "nbiot.h"

#ifdef IOT_NET_TYPE_NBIOT

#include "stm32l0xx_hal.h"
#include "SrvErrorNo.h"
#include "SrvIODeviceItem.h"
#include <string.h>
#include "protocol.h"
#include "usart.h"
#include "stdlib.h"
#include "character_convert.h"
#include "protocol.h"
#include "common_interface.h"
#include "tim.h"
#include "musart.h"
#include "stdio.h"

/**************************************************************************************************
 * static function prototypes
 *************************************************************************************************/
static int16_t nbiot_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t nbiot_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t nbiot_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t nbiot_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t nbiot_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);

static void NbiotClearRcvBuffer(void);
static void NbiotCopyDataToRcvBuff(void *p_pvParamSrc, void *p_pvParamLen);

void NbiotAtCommandSend(enNbiotAtCmd_t command_num);
static EnNBIOTNetStatus_t NbiotNetHandle(void);
static int16_t NbiotSendData(void *p_pvBuffer, uint16_t p_u16BufferSize);
static EnNBIOTNetStatus_t SearchNetWorkProc(void);
static int16_t NbiotSearchNetWork(void);
static void* NbiotGetRcvBuffer(void);

static int16_t NbiotUdpInitSocket(void);
static int16_t NbiotTcpInitSocket(void);
static void NbiotSetCmdAckState(enNbiotAtCmd_t Index, EnAtState_t State);
static uint8_t NbiotGetCmdAckState(enNbiotAtCmd_t Index);
static uint8_t NbiotWaitAtAck(enNbiotAtCmd_t Index);
static int8_t NbiotAtiResponse(char *p_pu8Buf);
static void NbiotQryCSQResponse(char *p_pu8Buf);
static uint32_t NbiotReadRssi(void);
static void NbiotTuestatsResponse(char *p_pu8Buf);
static void NbiotCgsnResponse(char *p_pu8Buf);

static int16_t NbiotReadImei(void);
static int16_t NbiotReadImsi(void);

static void NbiotCheckUartState(void);
static int16_t NbiotTeleComDev(void);
static int16_t NbiotTCPMode(void);
static int16_t NbiotUDPMode(void);
static int16_t NbiotNCDPOpen(void);
static int16_t NbiotConnectPlatform(void);
static int16_t NbiotCreatePPPLink(void);
static int16_t NbiotTcpInitSocket(void);
static void NbiotSetRecvMode(void);
static void NbiotCloseTcpSocket(void);
static void NbiotSetTcpLink(void);
static void NbiotQryTcpLink(void);
static void NbiotCheckIPStatus(void);
static void NbiotGPIOCtrlInit(void);
static void NbiotWakeUp(void);
static int16_t NbiotSetPSMState(void);
static void nbiotCloseEcho(void);
static void nbiotEventConfig(void);
static void ModuleWakeUp(void);
static void NbiotCheckCsq(void);
static void NbiotEnablePSM(void);
static void NbiotDisablePSM(void);




static StNBReadECL_t* NbiotReadEcl(void);
void NbiotCloseWindow(void);
#if 0
static void NbiotSwResetPortInit(void);
static void NbiotSwReset(void);
#endif
static void NbiotHwReset(void);
static void NbiotPWK(void);


/**************************************************************************************************
 * Private define
 *************************************************************************************************/
#define true    (1)
#define false   (0)
#define NBIOT_BUFF_SIZE     (500)                                                                          /**< NBIOT rcv buffer */   

#define NBIOT_GET_RCV_FALG() m_stNBIOTItem.u8RcvUartFlag
#define  NbiotWindowTimerDisable Tim21Disable

StSrvIODevice_t stNBIOT;

#define NBIOT_SET_RCV_FALG(flag) do \
{\
    m_stNBIOTItem.u8RcvUartFlag = flag;\
}while(0)



StAtCmdTab_t stAtCmdTab[EN_CMD_MAX] =
{
    {EN_CMD_AT, "AT\r\n", EN_AT_STATE_NULL},
    {EN_CMD_ATE0, "ATE0\r\n", EN_AT_STATE_NULL},
    {EN_CMD_NEONBIOTCFG, "AT+NEONBIOTCFG=0,0,1,0\r\n", EN_AT_STATE_NULL},
    {EN_CMD_CGATT_QRY, "AT+CGATT?\r\n", EN_AT_STATE_NULL},
    {EN_CMD_CEREG_QRY, "AT+CEREG?\r\n", EN_AT_STATE_NULL},
    {EN_CMD_CSQ, "AT+CSQ\r\n", EN_AT_STATE_NULL},
    {EN_CMD_TUESTATS_SET, "AT+TUESTATS=\"RADIO\"\r\n", EN_AT_STATE_NULL},
    {EN_CMD_NCDP_SET, "AT+NCDP=", EN_AT_STATE_NULL},/*Configure and Query CDP Server Settings*/
    {EN_CMD_NCDPOPEN, "AT+NCDPOPEN=", EN_AT_STATE_NULL},
    {EN_CMD_NNMI_SET, "AT+NNMI=1\r\n", EN_AT_STATE_NULL},
    {EN_CMD_SET_CGATT, "AT+CGATT=1\r\n", EN_AT_STATE_NULL},
    {EN_CMD_SET_CGATT_DETACH, "AT+CGATT=0\r\n", EN_AT_STATE_NULL},
    {EN_CMD_QRY_CFUN, "AT+CFUN?\r\n", EN_AT_STATE_NULL},
    {EN_CMD_SET_CFUN, "AT+CFUN=1\r\n", EN_AT_STATE_NULL},
    {EN_CMD_CIMI, "AT+CIMI\r\n", EN_AT_STATE_NULL},
    {EN_CMD_NMGS, "AT+NMGS=\r\n", EN_AT_STATE_NULL},
    {EN_CMD_QRY_IMEI, "AT+CGSN\r\n", EN_AT_STATE_NULL},
    {EN_CMD_NVSETPM_NORMAL, "AT+NVSETPM=0\r\n",EN_AT_STATE_NULL},
    {EN_CMD_NVSETPM_LP, "AT+NVSETPM=2\r\n",EN_AT_STATE_NULL},
    {EN_CMD_PSM_SET, "AT+CPSMS=1\r\n",EN_AT_STATE_NULL},
    {EN_CMD_EDRX_SET,"AT+CEDRXS=0\r\n",EN_AT_STATE_NULL},
    {EN_CMD_XIIC_SET, "AT+XIIC=1\r\n", EN_AT_STATE_NULL},/*you fang setup ppp*/
    {EN_CMD_XIIC_QRY, "AT+XIIC?\r\n", EN_AT_STATE_NULL},/*you fang qry ppp*/
    {EN_CMD_RECVMODE, "AT+RECVMODE=1\r\n",EN_AT_STATE_NULL},
    {EN_CMD_UDP_CLOSE, "AT+UDPCLOSE=0\r\n", EN_AT_STATE_NULL},/*you fang UDP close*/
    {EN_CMD_UDP_SETUP, "AT+UDPSETUP=0,", EN_AT_STATE_NULL},/*you fang UDP close*/
    {EN_CMD_UDP_SEND, "AT+UDPSEND=0,", EN_AT_STATE_NULL},/*you fang udp send*/
    {EN_CMD_TCPCLOSE,"AT+TCPCLOSE=0\r\n",EN_AT_STATE_NULL},
    {EN_CMD_TCP_SETUP,"AT+TCPSETUP=0,",EN_AT_STATE_NULL},
    {EN_CMD_TCP_SEND,"AT+TCPSEND=0,\r\n",EN_AT_STATE_NULL},
    {EN_CMD_IPSTATUS,"AT+IPSTATUS=0\r\n",EN_AT_STATE_NULL},
};

StNBIOTItem_t m_stNBIOTItem =
{
    .u8RcvUartFlag = 0,
    .enBand = EN_NBIOT_NBAND_CTCC,
    .enNetState = CHECK_SIM_STATE,
    .u8SocketState = EN_SOCKET_DISABLE,
    .p_UartRingBufferCallback = NULL,
    .u8MuartState = 0,
    .u8NetState = 1,
    .u8NcdOopenState = 0,
    .u8PsmState = 0,
};

static uint8_t m_u8NBIOTRcvBuffer[NBIOT_BUFF_SIZE];          /**< NBIOT driver rcv buff*/
static uint32_t m_u8NBIOTRcvLen = 0;
/**************************************************************************************************
 * static variables
 *************************************************************************************************/
static StSrvIODeviceItem_t m_stNBIOTDeviceItem;
static StSrvIOOperations_t m_stNBIOTOperation = 
{
    .open = nbiot_open,
    .close = nbiot_close,
    .read = nbiot_read,
    .write = nbiot_write,
    .ioctl = nbiot_ioctl
};

void NbiotWindowTimerReset(void)
{
    m_stNBIOTItem.StNbiotPsmState = 1;
    Tim21Reset();
}

static uint8_t getNcdpOpenState()
{
    return m_stNBIOTItem.u8NcdOopenState;
}

uint8_t NbiotGetPsmState(void)
{
    return m_stNBIOTItem.StNbiotPsmState;
}

static void ModuleWakeUp(void)
{
    if(0 == m_stNBIOTItem.u8PsmState)
    {
        NbiotWakeUp();
        m_stNBIOTItem.u8PsmState = 1;
    }
}

static void ModulerUartEnable(void)
{
    if(0 == m_stNBIOTItem.u8MuartState)
    {
        MuartInit();
        m_stNBIOTItem.u8MuartState = 1;
    }
}

static void ModulerUartDisable(void)
{
    if(1 == m_stNBIOTItem.u8MuartState)
    {
        MuartDeinit();
        m_stNBIOTItem.u8MuartState = 0;
    }
}

static int32_t CongifModulerUart(uint32_t p_Enable)
{
    if(1 == p_Enable)
    {
        ModulerUartEnable();
        return 0;
    }
    if(0 == p_Enable)
    {
        ModulerUartDisable();
        return 0;
    }
    else
    {
        return -1;
    }
}

/*************************************************************************************************/
/** @brief register nbiot device
*/
/*************************************************************************************************/
void nbiot_configure(stNbiotCfg_t p_stNbiotCfg)
{
    uint8_t *u8IpBuf = (uint8_t *)&m_stNBIOTItem.stNbiotCfgPara.u32IpAddr;
    memset(&m_stNBIOTItem, 0, sizeof(m_stNBIOTItem));

    /* reg nbiot Device */
    SrvIODeviceItem_register(&m_stNBIOTDeviceItem, EN_ID_NBIOT, &m_stNBIOTOperation, IO_READ | IO_WRITE);
    m_stNBIOTItem.stNbiotCfgPara = p_stNbiotCfg;

    sprintf((char*)m_stNBIOTItem.u8IpInfoStr,\
        "%d.%d.%d.%d,%d",\
        u8IpBuf[0], u8IpBuf[1], u8IpBuf[2], u8IpBuf[3],\
        m_stNBIOTItem.stNbiotCfgPara.u16Port);
    m_stNBIOTItem.u8NetState = 1;
    m_stNBIOTItem.u8NcdOopenState = 0;


    ModulerUartEnable();
    SWI_GPIO_Init();
    MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, NbiotCopyDataToRcvBuff);
    NbiotGPIOCtrlInit();
    Tim21RegisterCb((Tim21ItCallback)NbiotCloseWindow);
    MX_TIM21_Init();
    MX_TIM22_Init();
    Led_SetWorkMode(NET_LED,LED_MODE_BLINKING,LED_ALWAYSALIVE,LED_BLK_FREQ_1,3000);
    NbiotDisablePSM();
}

int16_t NbGetSocketState(void)
{
    return m_stNBIOTItem.u8SocketState;
}

/** @brief open nbiot device

    @param[in] p_pstSrvIODeviceItem Device item
    @param[in] p_pstSrvIODevice     Device

    @retval _SRVERRORNO_H_
*/
int16_t nbiot_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    p_pstSrvIODevice->pvPrivateData = (void *)0;
    p_pstSrvIODevice->pstDeviceItem->u8Count++;

    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 1)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
        //printf("nbiot_device is already opened!\r\n");
        //NbiotWakeUp();
        /* register nbiot callback func */
        MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, NbiotCopyDataToRcvBuff);
        return ERR_BEYOND_MAX;
    }

    if (EN_ID_NBIOT == p_pstSrvIODeviceItem->enDeviceId)
    {
        p_pstSrvIODevice->pvPrivateData = (void *)0;
    }

    /* register nbiot callback func */
    MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, NbiotCopyDataToRcvBuff);

    //printf("nbiot_open success!\r\n");
    return ERR_OK;
}

/** @brief close nbiot device

    @param[in] p_pstSrvIODeviceItem Device item
    @param[in] p_pstSrvIODevice     Device

    @retval _SRVERRORNO_H_
*/
static int16_t nbiot_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 0)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
    }
    p_pstSrvIODevice->pstDeviceItem = NULL;

    return ERR_OK;
}


int16_t NbiotDetachModule(void)
{
    int8_t DetachCounter = 0;
    int16_t ret = -1;
    for (DetachCounter = 0; DetachCounter < 6; DetachCounter++)
    {
        FeedDog();
        NbiotAtCommandSend(EN_CMD_SET_CGATT);
        ret = NbiotWaitAtAck(EN_CMD_SET_CGATT);
        if ((EN_AT_STATE_ACK_OK != ret) && (DetachCounter < 3))
        {
            printf("NbiotDetachModule attach fail!\r\n");
            continue;
        }

        NbiotAtCommandSend(EN_CMD_SET_CGATT_DETACH);
        ret = NbiotWaitAtAck(EN_CMD_SET_CGATT_DETACH);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            printf("NbiotDetachModule ok!\r\n");
            return 0;
        }
    }
    printf("NbiotDetachModule fail!\r\n");
    return -1;
}

void NbiotCloseWindow(void)
{
    NbiotWindowTimerDisable();
    m_stNBIOTItem.StNbiotPsmState = 0;
    m_stNBIOTItem.enNetState = CHECK_UART;
    m_stNBIOTItem.u8SocketState = EN_SOCKET_DISABLE;
    m_stNBIOTItem.u8PsmState = 0;
    Led_SetWorkMode(NET_LED,LED_MODE_BLINKING,LED_ALWAYSALIVE,LED_BLK_FREQ_1,3000);
    if(EN_TRANS_MODE_UDP == m_stNBIOTItem.stNbiotCfgPara.EnTransMode)
    {
        if((EN_AT_STATE_ACK_OK != NbiotGetCmdAckState(EN_CMD_UDP_SEND))\
            && (EN_AT_STATE_NULL != NbiotGetCmdAckState(EN_CMD_UDP_SEND))\
            && (m_stNBIOTItem.s32Rssi < (-105)))
        {
            printf("Nbiot send msg err,close moduler function.");
            NbiotSetCmdAckState(EN_CMD_UDP_SEND, EN_AT_STATE_NULL);
            NbiotDetachModule();
        }
    }
    else if(EN_TRANS_MODE_TCP == m_stNBIOTItem.stNbiotCfgPara.EnTransMode)
    {
        if((EN_AT_STATE_ACK_OK != NbiotGetCmdAckState(EN_CMD_TCP_SEND))\
            && (EN_AT_STATE_NULL != NbiotGetCmdAckState(EN_CMD_TCP_SEND))\
            && (m_stNBIOTItem.s32Rssi < (-105)))
        {
            printf("Nbiot send msg err,close moduler function.");
            NbiotSetCmdAckState(EN_CMD_TCP_SEND, EN_AT_STATE_NULL);
            NbiotDetachModule();
        }
    }
    else if(EN_TRANS_MODE_COAP == m_stNBIOTItem.stNbiotCfgPara.EnTransMode)
    {
        if((EN_AT_STATE_ACK_OK != NbiotGetCmdAckState(EN_CMD_NMGS))\
            && (EN_AT_STATE_NULL != NbiotGetCmdAckState(EN_CMD_NMGS))\
            && (m_stNBIOTItem.s32Rssi < (-105)))
        {
            printf("Nbiot send msg err,close moduler function.");
            NbiotSetCmdAckState(EN_CMD_NMGS, EN_AT_STATE_NULL);
            NbiotDetachModule();
        }
    }
    else
    {
        /*nothing*/
    }
}

static int16_t NbiotSearchNetWork(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    EnNBIOTNetStatus_t enStatus = STATUS_BUTT;
    uint16_t SearchCounter = 0;
    EnTransModeType_t EnTransMode = 0;

    if(EN_SOCKET_ENABLE == m_stNBIOTItem.u8SocketState)
    {
        return ERR_OK;
    }

    
    while(2 > SearchCounter)
    {
        /* nbiot device first connect net */
        enStatus = NbiotNetHandle();
        if((REGISTED == enStatus))
        {
            /*registe net fail*/
            printf("nbiot_net handle finished,enStatus=%d!\r\n", enStatus);
            break;
        }
        /*入网1次失败后，软重启*/
        SearchCounter++;
        if(1 == SearchCounter)
        {
            printf("Attach fail,Reboot module\r\n");
            FeedDog();
            /*Hardware reboot*/
            NbiotHwReset();
            m_stNBIOTItem.u8NcdOopenState = 0;
            NbiotPWK();
        }
    }
    
    if(REGISTED != enStatus)
    {
        printf("nbiot_net handle fail,enStatus=%d!\r\n", enStatus);
        /*sim card ok,search net fail,set cfun=0*/
        if(STATUS_BUTT != enStatus)
        {
            NbiotDetachModule();
            m_stNBIOTItem.u8NetState = 0;
        }
        else/*sim card abnormal*/
        {
            /*do nothing*/
        }
        return ERR_FAULT;
    }
    
    m_stNBIOTItem.u8NetState = 1;

    ret = NbiotSetPSMState();
    if(ERR_OK != ret)
    {
        printf("NbiotSetPSMState fail!\r\n");
        return ret;
    }
    printf("NbiotSetPSMState success!\r\n");
    
    ret = NbiotCreatePPPLink();
    if(ERR_OK != ret)
    {
        printf("NbiotCreatePPPLink fail\r\n");
        return ret;
    }
    printf("NbiotCreatePPPLink success!\r\n");

    ret = NbiotConnectPlatform();
    if(ERR_OK != ret)
    {
        printf("NbioComModeProc fail\r\n");
        return ret;
    }
    
    return ERR_OK;
}


static int16_t NbiotSetPSMState(void)
{
    uint8_t loop = 0;
    uint8_t ret = EN_AT_STATE_BUT;

    NbiotAtCommandSend(EN_CMD_PSM_SET);
    ret = NbiotWaitAtAck(EN_CMD_PSM_SET);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        return ERR_FAULT;
    }
    
    NbiotAtCommandSend(EN_CMD_EDRX_SET);
    ret = NbiotWaitAtAck(EN_CMD_EDRX_SET);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        return ERR_FAULT;
    }
    
    return ERR_OK;
}


static int16_t NbiotCreatePPPLink(void)
{
    uint8_t loop = 0;
    uint8_t ret = EN_AT_STATE_BUT;

    /* set NNMI*/
    NbiotAtCommandSend(EN_CMD_XIIC_SET);
    ret = NbiotWaitAtAck(EN_CMD_XIIC_SET);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        return ERR_FAULT;
    }
    
    for(loop = 0; loop < 5; loop++)
    {
        FeedDog();
        NbiotAtCommandSend(EN_CMD_XIIC_QRY);
        ret = NbiotWaitAtAck(EN_CMD_XIIC_QRY);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            return ERR_OK;
        }
    }
    return ERR_FAULT;
}

static int16_t NbiotConnectPlatform(void)
{
    m_stNBIOTItem.u8SocketState = EN_SOCKET_DISABLE;
    
    switch(m_stNBIOTItem.stNbiotCfgPara.EnTransMode)
    {
        case EN_TRANS_MODE_COAP:
        {
            return NbiotTeleComDev();
        }
        case EN_TRANS_MODE_UDP:
        {
            return NbiotUDPMode();
        }
        case EN_TRANS_MODE_TCP:
        {
            return NbiotTCPMode();
        }
        case EN_TRANS_MODE_MQTT:
        {
            return ERR_FAULT;
            /*reserve*/
            break;
        }
        case EN_TRANS_MODE_ONENET:
        {
            return ERR_FAULT;
            /*reserve*/
            break;
        }
        default:return ERR_FAULT;break;
    }
}

static int16_t NbiotTeleComDev(void)
{
    uint8_t ret = EN_AT_STATE_BUT;

    /* set NNMI*/
    NbiotAtCommandSend(EN_CMD_NNMI_SET);
    ret = NbiotWaitAtAck(EN_CMD_NNMI_SET);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("nbiot_open set nnmi err!\r\n");
        return ERR_FAULT;
    }
    
    ret = NbiotNCDPOpen();
    if(ERR_OK != ret)
    {
        return ERR_FAULT;
    }

    m_stNBIOTItem.u8SocketState = EN_SOCKET_ENABLE;
    return ERR_OK;
}

static int16_t NbiotNCDPOpen(void)
{
    int16_t ret = ERR_OK;
    uint8_t loop = 0;
    uint8_t *u8IpBuf = (uint8_t*)&m_stNBIOTItem.stNbiotCfgPara.u32IpAddr;

    if(1 == getNcdpOpenState())
    {
        return ERR_OK;
    }
    
    sprintf((char*)m_stNBIOTItem.u8IpInfoStr,\
        "\"%d.%d.%d.%d\",%d",\
        u8IpBuf[0], u8IpBuf[1], u8IpBuf[2], u8IpBuf[3],\
        m_stNBIOTItem.stNbiotCfgPara.u16Port);
    
    for(loop = 0; loop < 3; loop++)
    {
        FeedDog();
        NbiotAtCommandSend(EN_CMD_NCDPOPEN);
        ret = NbiotWaitAtAck(EN_CMD_NCDPOPEN);
        if(EN_AT_STATE_ACK_OK == ret)
        {
            m_stNBIOTItem.u8NcdOopenState = 1;
            return ERR_OK;
        }
    }
    printf("nbiot set ip fail,ret = %d!\r\n",ret);
    return ERR_FAULT;
}


static int16_t NbiotTCPMode(void)
{
    uint8_t loop = 0;
    EnNBIOTTCPStatus_t EnTcpStatus = STATUS_BUTT;
    m_stNBIOTItem.EnTCPStatus = SET_RECVMODE;
    for(loop = 0; loop < 20; loop++)
    {
        FeedDog();
        EnTcpStatus = NbiotTcpInitSocket();
        if(TCP_SUCCESS == EnTcpStatus)
        {
            m_stNBIOTItem.u8SocketState = EN_SOCKET_ENABLE;
            return ERR_OK;
        }
        delay_ms(1000);
    }
    return ERR_FAULT;
}

static int16_t NbiotUDPMode(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    
    NbiotAtCommandSend(EN_CMD_RECVMODE);
    ret = NbiotWaitAtAck(EN_CMD_RECVMODE);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("udp set recv mode fail!\r\n");
        return ERR_FAULT;
    }

    NbiotAtCommandSend(EN_CMD_UDP_CLOSE);
    ret = NbiotWaitAtAck(EN_CMD_UDP_CLOSE);
 
    
    NbiotAtCommandSend(EN_CMD_UDP_SETUP);
    ret = NbiotWaitAtAck(EN_CMD_UDP_SETUP);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("udp set up fail!\r\n");
        return ERR_FAULT;
    }
    
    m_stNBIOTItem.u8SocketState = EN_SOCKET_ENABLE;
    return ERR_OK;
}


/** @brief write data to nbiot device

    @param[in] p_pstSrvIODevice   nbiot Device
    @param[in] p_pvBuffer         to be write Buffer
    @param[in] p_u16BufferSize    to bo write data lenght

    @retval _SRVERRORNO_H_
*/
static int16_t nbiot_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    int16_t s16Result = 0;

    if(p_pstSrvIODevice->u32Position + p_u16BufferSize > NBIOT_BUFF_SIZE)
    {
        return ERR_OUT_OF_RANGE;
    }
    if (EN_SOCKET_ENABLE == NbGetSocketState())
    {
        s16Result = NbiotSendData(p_pvBuffer, p_u16BufferSize);
    }
    else
    {
        s16Result = NbiotSearchNetWork();
        if(ERR_OK != s16Result)
        {
            printf("nbiot_write search network fail,s16Result=%d!\r\n", s16Result);
            return s16Result;
        }
        if (EN_SOCKET_ENABLE == NbGetSocketState())
        {
            s16Result = NbiotSendData(p_pvBuffer, p_u16BufferSize);
        }
    }
    /*Reset window timer */
    NbiotWindowTimerReset();
    return s16Result;
}

/** @brief read data from nbiot device
    @param[in] p_pstSrvIODevice   nbiot Device
    @param[in] p_pvBuffer         des buffer poiter
    @param[in] p_u16BufferSize    read data lenght

    @retval _SRVERRORNO_H_
*/
static int16_t nbiot_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= 500 )
    {
        return ERR_OUT_OF_RANGE;
    }

    return ERR_OK;
}

static int16_t nbiot_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    int16_t ret = -1;
    switch(p_u32Command)
    {
        case EN_CMD_REGRECVCALLBACK:
        {
            m_stNBIOTItem.p_UartRingBufferCallback = (UartRingBufferCallback)p_u32Arg;
        }break;
        case EN_CMD_GETRSSI:
        {
            *(uint32_t *)p_u32Arg = NbiotReadRssi();
        }break;
        case EN_CMD_GETECL:
        {
            StNBReadECL_t *u8temp = NULL;
            ModulerUartEnable();
            u8temp = (StNBReadECL_t *)NbiotReadEcl();
            if (NULL == u8temp)
            {
                return -1;
            }
            memcpy((char *)p_u32Arg,(char *)u8temp, sizeof(StNBReadECL_t));
        }break;
        case EN_CMD_PSM_STATE:
        {
            *(uint32_t *)p_u32Arg = NbiotGetPsmState();
            break;
        }
        case EN_CMD_SEARCH_NETWORK:
        {
            ModulerUartEnable();
            return NbiotSearchNetWork();
        }
        case EN_CMD_GET_IMEI:
        {
            ModulerUartEnable();
            ret = NbiotReadImei();
            if (ERR_OK != ret)
            {
                return -1;
            }
            memcpy((char *)p_u32Arg, (char *)m_stNBIOTItem.u8Imei, IMEI_LEN_MAX);
            break;
        }
        case EN_CMD_GET_IMSI:
        {
            ModulerUartEnable();
            ret = NbiotReadImsi();
            if (ERR_OK != ret)
            {
                return -1;
            }
            memcpy((char *)p_u32Arg, (char *)m_stNBIOTItem.u8Imsi, IMEI_LEN_MAX);
            break;
        }
        case EN_CMD_SET_MUART_STATE:
        {
            return CongifModulerUart(*(uint32_t *)p_u32Arg);
        }
        case EN_CMD_GET_NBIOT_NET_STATE:
        {
            *(uint32_t *)p_u32Arg = m_stNBIOTItem.u8NetState;
            break;
        }
        case EN_CMD_WAKEUP:
        {
            NbiotWakeUp();
            break;
        }
        case EN_CMD_ENABLE_PSM:
        {
            NbiotEnablePSM();
            break;
        }
        case EN_CMD_DISABLE_PSM:
        {
            NbiotDisablePSM();
            break;
        }
        default:break;
    }

    //printf("nbiot_ioctl success!\r\n");

    return 0;
}

/** @brief NbiotCopyDataToRcvBuff
*/
static void NbiotCopyDataToRcvBuff(void *p_pvParamSrc, void *p_pvParamLen)
{
    /* copy uart buffer data to NBIOT driver rcv buff */
    memcpy(m_u8NBIOTRcvBuffer, (uint8_t*)p_pvParamSrc, *(uint32_t *)p_pvParamLen);
    m_u8NBIOTRcvLen = *(uint32_t *)p_pvParamLen;
    #if 0
    memset(ModuleCmdProcessBuf, 0, sizeof(ModuleCmdProcessBuf));
    
    memcpy(ModuleCmdProcessBuf, NbiotGetRcvBuffer(), NbiotGetRcvBufferLen());

    NbiotClearRcvBuffer();
    #endif
    /* Flag=1 indecated one frame rev finished */
    NBIOT_SET_RCV_FALG(1);
}

/** @brief Clear nbiot Uart Buffer.
*/
static void NbiotClearRcvBuffer(void)
{
    memset(m_u8NBIOTRcvBuffer, 0, sizeof(m_u8NBIOTRcvBuffer));
    NBIOT_SET_RCV_FALG(0);
}

/** @brief Get nbiot Uart Buffer.
*/
static void* NbiotGetRcvBuffer(void)
{
    return m_u8NBIOTRcvBuffer;
}

static uint32_t NbiotGetRcvBufferLen(void)
{
    return m_u8NBIOTRcvLen;
}

/** @brief NBIOT connect to net.in 40s,if can not connect net,return error.
    @ retval int8_t 
*/
static EnNBIOTNetStatus_t NbiotNetHandle(void)
{
    uint16_t i;

    EnNBIOTNetStatus_t enNetState = STATUS_BUTT;
    NbiotClearRcvBuffer();
    NbiotWakeUp();
    NbiotDisablePSM();
    m_stNBIOTItem.enNetState = CHECK_UART;
    for(i = 0; i < 40; i++)
    {
        printf("Searching network...!\r\n");
        FeedDog();
        enNetState = SearchNetWorkProc();
        if((REGISTED == enNetState) || (STATUS_BUTT == enNetState))
        {
            break;
        }
        delay_ms(500);
    }

    return(enNetState);
}

static void NbiotModulerEnable(void)
{
    uint8_t ret = EN_AT_STATE_BUT;

    NbiotAtCommandSend(EN_CMD_QRY_CFUN);
    ret = NbiotWaitAtAck(EN_CMD_QRY_CFUN);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        return;
    }

    /*moduler is enabled*/
    if (1 == m_stNBIOTItem.u8CfunState)
    {
        //m_stNBIOTItem.enNetState = CHECK_CSQ;
        m_stNBIOTItem.enNetState = ATTACH;
        printf("NbiotModulerEnable ok,convert to attach status!\r\n");
        return;
    }

    /*send "AT+CFUN=1\r\n" enable moduler*/
    NbiotAtCommandSend(EN_CMD_SET_CFUN);
    ret = NbiotWaitAtAck(EN_CMD_SET_CFUN);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        /*do nothing*/
    }
}


static void NbiotCheckCsq(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    
    NbiotAtCommandSend(EN_CMD_CSQ);
    ret = NbiotWaitAtAck(EN_CMD_CSQ);

    if (EN_AT_STATE_ACK_OK == ret)
    {
        m_stNBIOTItem.enNetState = ATTACH;
        return;
    }
    
    m_stNBIOTItem.enNetState = CHECK_CSQ;
}

static void NbiotCheckUartState(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    uint8_t loop = 0;

    for(loop = 0; loop < 4; loop++)
    {
        FeedDog();
        NbiotWakeUp();
        NbiotAtCommandSend(EN_CMD_AT);
        ret = NbiotWaitAtAck(EN_CMD_AT);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            m_stNBIOTItem.enNetState = CLOSE_ECHO;
            return;
        }
    }
    m_stNBIOTItem.enNetState = STATUS_BUTT;
    printf("NbiotCheckUartState fail!\r\n");
}



static void nbiotEventConfig(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    uint8_t loop = 0;

    for(loop = 0; loop < 3; loop++)
    {
        NbiotAtCommandSend(EN_CMD_NEONBIOTCFG);
        ret = NbiotWaitAtAck(EN_CMD_NEONBIOTCFG);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            return;
        }
    }
    printf("nbiotEventConfig fail!\r\n");
}


static void nbiotCloseEcho(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    uint8_t loop = 0;

    for(loop = 0; loop < 3; loop++)
    {
        FeedDog();
        NbiotAtCommandSend(EN_CMD_ATE0);
        ret = NbiotWaitAtAck(EN_CMD_ATE0);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            m_stNBIOTItem.enNetState = CHECK_SIM_STATE;
            return;
        }
    }
    m_stNBIOTItem.enNetState = CHECK_SIM_STATE;
    printf("nbiotCloseEcho fail!\r\n");
}
static void NbiotCheckSimState(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    uint8_t loop = 0;

    for(loop = 0; loop < 4; loop++)
    {
        FeedDog();
        NbiotAtCommandSend(EN_CMD_CIMI);
        ret = NbiotWaitAtAck(EN_CMD_CIMI);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            m_stNBIOTItem.enNetState = MODULER_ENABLE;
            printf("NbiotCheckSimState ok,convert to moduler enable status!\r\n");
            return;
        }
    }
    
    m_stNBIOTItem.enNetState = STATUS_BUTT;
    printf("NbiotCheckSimState sim card identify fail,pls check card!\r\n");
}


/** @brief if nbiot net not active,send "AT+CGATT=1\r\n" to active net.
*/
static void NbiotNetAttach(void)
{
    uint8_t ret = EN_AT_STATE_BUT;

    NbiotAtCommandSend(EN_CMD_CGATT_QRY);
    ret = NbiotWaitAtAck(EN_CMD_CGATT_QRY);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        return;
    }

    /*moduler is enabled*/
    if (1 == m_stNBIOTItem.u8CgattState)
    {
        m_stNBIOTItem.enNetState = NET_REG;
        printf("NbiotNetAttach ok,convert to NET_REG status!\r\n");
        return;
    }

    /*send "AT+CGATT=1\r\n" to active net*/
    NbiotAtCommandSend(EN_CMD_SET_CGATT);
    ret = NbiotWaitAtAck(EN_CMD_SET_CGATT);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("NbiotNetAttach cgatt set fail!\r\n");
    }
}

/** @brief nbiot net attach.
*/
static void NbiotNetReg(void)
{
    uint8_t ret = EN_AT_STATE_BUT;

    NbiotAtCommandSend(EN_CMD_CEREG_QRY);
    ret = NbiotWaitAtAck(EN_CMD_CEREG_QRY);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        m_stNBIOTItem.enNetState = NET_REG;
        printf("NbiotNetReg cereg qry err,convert to ATTACH status!\r\n");
        return;
    }

    /*moduler is enabled*/
    if (1 == m_stNBIOTItem.u8CeregState)
    {
        m_stNBIOTItem.enNetState = REGISTED;
        printf("NbiotNetReg ok,convert to REGISTED status!\r\n");
    }
    else
    {
        m_stNBIOTItem.enNetState = NET_REG;
    }
}

/** @brief nbiot net search,status convet from 
    QRY_BAND->DETACH->ATTACH->REGISTED
*/
static EnNBIOTNetStatus_t SearchNetWorkProc(void)
{
    switch(m_stNBIOTItem.enNetState)
    {
        case CHECK_UART:
        {
            NbiotCheckUartState();
            break;
        }
        case CLOSE_ECHO:
        {
            nbiotCloseEcho();
            break;
        }
        case CHECK_SIM_STATE:
        {
            NbiotCheckSimState();
            break;
        }
        case MODULER_ENABLE:
        {
            NbiotModulerEnable();
            break;
        }
        case CHECK_CSQ:
        {
            NbiotCheckCsq();
            break;
        }
        case ATTACH:
        {
            NbiotNetAttach();
            break;
        }
        case NET_REG:
        {
            NbiotNetReg();
            break;
        }
        case REGISTED:
        {
            break;
        }
        default:break;
    }

    return m_stNBIOTItem.enNetState;
}

/** @brief nbiot send at cmd
*/
#define NBIOT_SEND_AT_LEN_MAX    (64)
void NbiotAtCommandSend(enNbiotAtCmd_t command_num)
{ 
    int8_t u8temp[NBIOT_SEND_AT_LEN_MAX] = {0};
    //uint8_t *u8IpBuf = (uint8_t*)&m_stNBIOTItem.stNbiotCfgPara.u32IpAddr;
    memset(u8temp, 0, sizeof(u8temp));
    switch(command_num)
    {
        case EN_CMD_NCDP_SET:
        {
            sprintf((char*)u8temp,"%s%s\r\n", stAtCmdTab[command_num].CmdStr, m_stNBIOTItem.u8IpInfoStr);
            MuartSend((uint8_t*)u8temp, strlen((char*)u8temp));
            break;
        }
        case EN_CMD_NCDPOPEN:
        {
            sprintf((char*)u8temp,"%s%s\r\n", stAtCmdTab[command_num].CmdStr, m_stNBIOTItem.u8IpInfoStr);
            MuartSend((uint8_t*)u8temp, strlen((char*)u8temp));
            break;
        }
        case EN_CMD_UDP_SETUP:
        {
            sprintf((char*)u8temp,"%s%s\r\n", stAtCmdTab[command_num].CmdStr, m_stNBIOTItem.u8IpInfoStr);
            MuartSend((uint8_t*)u8temp, strlen((char*)u8temp));
            break;
        }
        case EN_CMD_TCP_SETUP:
        {
            sprintf((char*)u8temp,"%s%s\r\n", stAtCmdTab[command_num].CmdStr, m_stNBIOTItem.u8IpInfoStr);
            MuartSend((uint8_t*)u8temp, strlen((char*)u8temp));
            break;
        }
        default:
        {
            strncpy((char*)u8temp, (char*)stAtCmdTab[command_num].CmdStr, strlen((char*)stAtCmdTab[command_num].CmdStr));
            MuartSend((uint8_t*)u8temp, strlen((char*)u8temp));
            break;
        }
    }
    stAtCmdTab[command_num].EnCmdState = EN_AT_STATE_WAIT_ACK;
}

static int16_t NbiotUdpInitSocket(void)
{
    uint8_t ret = EN_AT_STATE_BUT;

    NbiotAtCommandSend(EN_CMD_XIIC_SET);
    ret = NbiotWaitAtAck(EN_CMD_XIIC_SET);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("nbiot_open set up ppp fail!\r\n");
        return -1;
    }

    NbiotAtCommandSend(EN_CMD_XIIC_QRY);
    ret = NbiotWaitAtAck((EN_CMD_XIIC_QRY));
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("nbiot_open qry ppp connect fail!\r\n");
        return -1;
    }

    NbiotAtCommandSend(EN_CMD_UDP_SETUP);
    ret = NbiotWaitAtAck(EN_CMD_UDP_SETUP);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("nbiot_open udp set up fail!\r\n");
        return -1;
    }

    return 0;
}

static int16_t NbiotTcpInitSocket(void)
{
    switch(m_stNBIOTItem.EnTCPStatus)
    {
        case SET_RECVMODE:
        {
            NbiotSetRecvMode();
            break;
        }
        case TCP_CLOSE:
        {
            NbiotCloseTcpSocket();
            break;
        }
        case TCP_SETUP:
        {
            NbiotSetTcpLink();
            break;
        }
        case CHECK_IPSTATUS:
        {
            NbiotCheckIPStatus();
            break;
        }
    }
    return m_stNBIOTItem.EnTCPStatus;
}

static void NbiotSetRecvMode(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    
    NbiotAtCommandSend(EN_CMD_RECVMODE);
    ret = NbiotWaitAtAck(EN_CMD_RECVMODE);
    if (EN_AT_STATE_ACK_OK == ret)
    {
        m_stNBIOTItem.EnTCPStatus = TCP_CLOSE;
    }
    else
    {
        m_stNBIOTItem.EnTCPStatus = SET_RECVMODE;
    }
}

static void NbiotCloseTcpSocket(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    
    NbiotAtCommandSend(EN_CMD_TCPCLOSE);
    ret = NbiotWaitAtAck(EN_CMD_TCPCLOSE);
    if (EN_AT_STATE_ACK_OK == ret)
    {
        m_stNBIOTItem.EnTCPStatus = TCP_SETUP;
    }
    else
    {
        m_stNBIOTItem.EnTCPStatus = TCP_CLOSE;
    }
}

static void NbiotSetTcpLink(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    
    NbiotAtCommandSend(EN_CMD_TCP_SETUP);
    ret = NbiotWaitAtAck(EN_CMD_TCP_SETUP);
    if (EN_AT_STATE_ACK_OK == ret)
    {
        m_stNBIOTItem.EnTCPStatus = CHECK_IPSTATUS;
    }
    else
    {
        m_stNBIOTItem.EnTCPStatus = TCP_SETUP;
    }
}

static void NbiotCheckIPStatus(void)
{
    uint8_t ret = EN_AT_STATE_BUT;

    printf("Check ip status\r\n");
    NbiotAtCommandSend(EN_CMD_IPSTATUS);
    ret = NbiotWaitAtAck(EN_CMD_IPSTATUS);
    if (EN_AT_STATE_ACK_OK == ret)
    {
        m_stNBIOTItem.EnTCPStatus = TCP_SUCCESS;
    }
    else
    {
        m_stNBIOTItem.EnTCPStatus = CHECK_IPSTATUS;
    }
}

/** @brief write data to nbiot device

    @param[in] p_pvBuffer         to be Buffer poiter
    @param[in] p_u16BufferSize    to be write data lenght

    @retval EnNBIOTErrCode_t
*/
#define NBIOT_SEND_DATA_LEN_MAX    (256)
static int8_t NBtextBuf[NBIOT_SEND_DATA_LEN_MAX] = {0};
static int8_t SendAtBuffer[2 * NBIOT_SEND_DATA_LEN_MAX] = {0};
int16_t NbiotSendData(void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    uint8_t *u8IpBuf = (uint8_t *)&m_stNBIOTItem.stNbiotCfgPara.u32IpAddr;//xjh
    memset(SendAtBuffer, 0, sizeof(SendAtBuffer));
    memset(NBtextBuf, 0, sizeof(NBtextBuf));
    if(NULL == p_pvBuffer)
    {
        printf("NbiotSendData p_pvBuffer is NULL!\r\n");
        return -1;
    }

    if(p_u16BufferSize > NBIOT_SEND_DATA_LEN_MAX)
    {
        printf("NbiotSendData p_u16BufferSize is invalid,p_u16BufferSize = %d!\r\n", p_u16BufferSize);
        return -1;
    }

    if(EN_SOCKET_DISABLE == m_stNBIOTItem.u8SocketState)
    {
        printf("NbiotSendData socket not init!\r\n");
        return -1;
    }
    FeedDog();
    Hex2Char(p_pvBuffer, NBtextBuf, p_u16BufferSize);
    
    switch(m_stNBIOTItem.stNbiotCfgPara.EnTransMode)
    {
        case EN_TRANS_MODE_UDP:
        {
            sprintf((char*)SendAtBuffer,"AT+UDPSEND=0,%d\r\n", p_u16BufferSize);
            MuartSend((uint8_t *)SendAtBuffer, strlen((char*)SendAtBuffer));
            delay_ms(2000);
            printf("nbiot send msg:len,%d,%s", p_u16BufferSize, NBtextBuf);
            NbiotSetCmdAckState(EN_CMD_UDP_SEND, EN_AT_STATE_WAIT_ACK);
            MuartSend((uint8_t *)p_pvBuffer, p_u16BufferSize);
            break;
        }
        case EN_TRANS_MODE_TCP:
        {
            sprintf((char*)SendAtBuffer,"AT+TCPSEND=0,%d\r\n", p_u16BufferSize);
            MuartSend((uint8_t *)SendAtBuffer, strlen((char*)SendAtBuffer));
            delay_ms(2000);
            printf("nbiot send msg:len:%d,%s\r\n", p_u16BufferSize, (char *)NBtextBuf);
            NbiotSetCmdAckState(EN_CMD_TCP_SEND, EN_AT_STATE_WAIT_ACK);
            MuartSend((uint8_t *)p_pvBuffer, p_u16BufferSize);
            break;
        }
        case EN_TRANS_MODE_COAP:
        {
            sprintf((char*)SendAtBuffer,"AT+NMGS=%d,%s\r\n",strlen((char*)NBtextBuf)/2,NBtextBuf);
            NbiotSetCmdAckState(EN_CMD_NMGS, EN_AT_STATE_WAIT_ACK);
            MuartSend((uint8_t *)SendAtBuffer,strlen((char*)SendAtBuffer));
            printf("nbiot send msg:%s", SendAtBuffer);
            break;
        }
        case EN_TRANS_MODE_MQTT:
        {
            /*reserve*/
            break;
        }
        case EN_TRANS_MODE_ONENET:
        {
            /*reserve*/
            break;
        }
        default:break;
    }
    return 0;
}

#define NBIOT_ACK_LEN_MAX    (128)

int8_t u8TempHexBuf[NBIOT_ACK_LEN_MAX / 2] = {0};
int8_t u8TempBuffer[NBIOT_ACK_LEN_MAX] = {0};

/** @brief Read nbiot signal.
*/
static uint32_t NbiotReadRssi(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    
    NbiotAtCommandSend(EN_CMD_CSQ);
    ret = NbiotWaitAtAck(EN_CMD_CSQ);
    
    printf("Rssi:-%d\r\n",m_stNBIOTItem.s32Rssi);
    return m_stNBIOTItem.s32Rssi;
}

static void NbiotQryCSQResponse(char *p_pu8Buf)
{
    int32_t Ret = -1;

    if(strstr((char*)p_pu8Buf, "+CSQ: 99") != NULL)
    {
        NbiotSetCmdAckState(EN_CMD_CSQ, EN_AT_STATE_ACK_ERR);
    }
    else if(strstr((char*)p_pu8Buf, "OK") != NULL)
    {
        Ret = GetStrVale((int8_t*)p_pu8Buf, "+CSQ: ", ",",      strlen("+CSQ:"));
        if(-1 == Ret)
        {
            NbiotSetCmdAckState(EN_CMD_CSQ, EN_AT_STATE_ACK_ERR);
            return;
        }
        m_stNBIOTItem.s32Rssi = 113 - (Ret * 2);
        NbiotSetCmdAckState(EN_CMD_CSQ, EN_AT_STATE_ACK_OK);
    }
    else if(strstr((char*)p_pu8Buf, "ERROR") != NULL)
    {
        NbiotSetCmdAckState(EN_CMD_CSQ, EN_AT_STATE_ACK_ERR);
    }
    else
    {
        /*do nothing*/
    }
}

static StNBReadECL_t* NbiotReadEcl(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    NbiotAtCommandSend(EN_CMD_TUESTATS_SET);
    ret = NbiotWaitAtAck(EN_CMD_TUESTATS_SET);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("NbiotRead Ecl fail!\r\n");
        return NULL;
    }

    return (StNBReadECL_t *)&m_stNBIOTItem.stNbiotEcl;
}

static int16_t NbiotReadImei(void)
{
    uint8_t ret = EN_AT_STATE_BUT;
    if (0 != strlen((char*)m_stNBIOTItem.u8Imei))
    {
        return ERR_OK;
    }

    NbiotAtCommandSend(EN_CMD_QRY_IMEI);
    ret = NbiotWaitAtAck(EN_CMD_QRY_IMEI);
    if (EN_AT_STATE_ACK_OK != ret)
    {
        printf("NbiotReadImei read imei fail!\r\n");
        return ERR_FAULT;
    }

    return ERR_OK;
}


static int16_t NbiotReadImsi(void)
{
    uint8_t loop = 0;
    uint8_t ret = EN_AT_STATE_BUT;
    if (0 != strlen((char *)m_stNBIOTItem.u8Imsi))
    {
        return ERR_OK;
    }
    for(loop = 0; loop < 2; loop++)
    {
        NbiotAtCommandSend(EN_CMD_CIMI);
        ret = NbiotWaitAtAck(EN_CMD_CIMI);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            return ERR_OK;
        }
    }
    
    return ERR_FAULT;
}

static void NbiotTuestatsResponse(char *p_pu8Buf)
{
    int32_t Ret = -1;

    if(strstr((char*)p_pu8Buf, "OK") != NULL)
    {

        Ret = GetStrVale((int8_t*)p_pu8Buf, "Signal power,", "\r\n",      strlen("Signal power,"));
        if(-1 == Ret)
        {
            NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_ERR);
            return;
        }

        Ret = GetStrVale((int8_t*)p_pu8Buf, "Cell ID,", "\r\n",      strlen("Cell ID,"));
        if(-1 == Ret)
        {
            NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_ERR);
            return;
        }

        m_stNBIOTItem.stNbiotEcl.u32CellID = Ret;

        Ret = GetStrVale((int8_t*)p_pu8Buf, "ECL,", "\r\n",      strlen("ECL,"));
        if(-1 == Ret)
        {
            NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_ERR);
            return;
        }

        m_stNBIOTItem.stNbiotEcl.u8ECL = Ret;
        Ret = GetStrVale((int8_t*)p_pu8Buf, "SNR,", "\r\n",      strlen("SNR,"));
        if(-1 == Ret)
        {
            NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_ERR);
            return;
        }
        m_stNBIOTItem.stNbiotEcl.s8SNR = Ret;

        Ret = GetStrVale((int8_t*)p_pu8Buf, "PCI,", "\r\n",      strlen("PCI,"));
        if(-1 == Ret)
        {
            NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_ERR);
            return;
        }
        m_stNBIOTItem.stNbiotEcl.u16PCI = Ret;

        NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_OK);

        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
    }
    else if(strstr((char*)p_pu8Buf, "ERROR") != NULL)
    {
        NbiotSetCmdAckState(EN_CMD_TUESTATS_SET, EN_AT_STATE_ACK_ERR);
    }
    else
    {
        /*do nothing*/
    }
}

static void NbiotSetCmdAckState(enNbiotAtCmd_t Index, EnAtState_t State)
{
    stAtCmdTab[Index].EnCmdState = State;
}

static uint8_t NbiotGetCmdAckState(enNbiotAtCmd_t Index)
{
    return stAtCmdTab[Index].EnCmdState;
}

static uint8_t NbiotWaitAtAck(enNbiotAtCmd_t Index)
{
    uint16_t u16waitnum = 0;
    /*if received ack or timeout,exit loop*/
    while(!((u16waitnum > 1000) ||
        (EN_AT_STATE_WAIT_ACK != NbiotGetCmdAckState(Index))))
    {
         u16waitnum++;
         delay_ms(4);
    }

    if (u16waitnum > 1000)
    {
        NbiotSetCmdAckState(Index, EN_AT_STATE_TIMEOUT);
    }
    return NbiotGetCmdAckState(Index);
}

static void NbiotQryBandResponse(char *p_pu8Buf)
{
}

static void NbiotCfunQryResponse(char *p_pu8Buf)
{
    if(strstr((char*)p_pu8Buf, "OK") != NULL)
    {
        /*moduler is enabled*/
        if(strstr((char*)p_pu8Buf, "+CFUN: 1") != NULL)
        {
            m_stNBIOTItem.u8CfunState = 1;
            NbiotSetCmdAckState(EN_CMD_QRY_CFUN, EN_AT_STATE_ACK_OK);
        }
        else if(strstr((char*)p_pu8Buf, "+CFUN: 0") != NULL)
        {
            m_stNBIOTItem.u8CfunState = 0;
            NbiotSetCmdAckState(EN_CMD_QRY_CFUN, EN_AT_STATE_ACK_OK);
        }
        else
        {
            NbiotSetCmdAckState(EN_CMD_QRY_CFUN, EN_AT_STATE_ACK_ERR);
        }
        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");

    }
    else if(strstr((char*)p_pu8Buf, "ERROR") != NULL)
    {
        NbiotSetCmdAckState(EN_CMD_QRY_CFUN, EN_AT_STATE_ACK_ERR);
    }
    else
    {
        /*do nothing*/
    }
}


static void NbiotQryCgattResponse(char *p_pu8Buf)
{
    if(strstr((char*)p_pu8Buf, "OK") != NULL)
    {
        /*moduler is enabled*/
        if(strstr((char*)p_pu8Buf, "+CGATT: 1") != NULL)
        {
            m_stNBIOTItem.u8CgattState = 1;
            NbiotSetCmdAckState(EN_CMD_CGATT_QRY, EN_AT_STATE_ACK_OK);
        }
        else if(strstr((char*)p_pu8Buf, "+CGATT: 0") != NULL)
        {
            m_stNBIOTItem.u8CgattState = 0;
            NbiotSetCmdAckState(EN_CMD_CGATT_QRY, EN_AT_STATE_ACK_OK);
        }
        else
        {
            NbiotSetCmdAckState(EN_CMD_CGATT_QRY, EN_AT_STATE_ACK_ERR);
        }
        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
    }
    else if(strstr((char*)p_pu8Buf, "ERROR") != NULL)
    {
        NbiotSetCmdAckState(EN_CMD_CGATT_QRY, EN_AT_STATE_ACK_ERR);
    }
    else
    {
        /*do nothing*/
    }
}

static void NbiotQryCeregResponse(char *p_pu8Buf)
{
    if(strstr((char*)p_pu8Buf, "OK") != NULL)
    {
        /*moduler is enabled*/
        if(strstr((char*)p_pu8Buf, "+CEREG: 0,1") != NULL)
        {
            m_stNBIOTItem.u8CeregState = 1;
            NbiotSetCmdAckState(EN_CMD_CEREG_QRY, EN_AT_STATE_ACK_OK);
        }
        else
        {
            m_stNBIOTItem.u8CeregState = 0;
            NbiotSetCmdAckState(EN_CMD_CEREG_QRY, EN_AT_STATE_ACK_OK);
        }
        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
    }
    else if(strstr((char*)p_pu8Buf, "ERROR") != NULL)
    {
        m_stNBIOTItem.u8CeregState = 0;
        NbiotSetCmdAckState(EN_CMD_CEREG_QRY, EN_AT_STATE_ACK_ERR);
    }
    else
    {
        /*nothing*/
    }
}


static void NbiotSetNsorfResponse(char *p_pu8Buf)
{
    uint8_t u8Tempdatalen = 0;

    int8_t *TempStrStart = NULL;
    int8_t *TempStrEnd = NULL;
    //int8_t IpAddr[] = "120.26.207.183,8005";
    printf("rcv msg:%s!\r\n", p_pu8Buf);

    /*收到下行数据后，发生空数据，设置模块进入idle状�?/
    //NbiotAtCommandSend(EN_CMD_SET_NSOSTF_NULL);

    /*check NSONMI response ok*/
    if(strstr((char*)p_pu8Buf, "OK") != NULL)
    {
        /*Response frame
        <socket>,<ip_addr>,<port>,<length>,<data>,<remaining_
        length> 
        OK
        */

        /*find the <data> poiter*/
        TempStrStart = (int8_t*)strstr((char*)p_pu8Buf,\
            (char*)m_stNBIOTItem.u8IpInfoStr);
        if (NULL == TempStrStart)
        {
            printf("NSORF response err,no IP addr!\r\n");
        }

        for(int i = 0; i < 3; i++)
        {
            TempStrStart = (int8_t*)strstr((char*)TempStrStart,",");
            if(NULL == TempStrStart)
            {
                printf("TempStrStart is NULL,i = %d\r\n", i);
                return;
            }
            TempStrStart++;
            printf("TempStrStart is %s\r\n", TempStrStart);
        }
    
        if (NULL == TempStrStart)
        {
            printf("NSONMI response frame format TempStrStart err!\r\n");
            return;
        }
        printf("TempStrStart is %s\r\n", TempStrStart);
        TempStrEnd = (int8_t*)strstr((char*)TempStrStart,",");
        if (NULL == TempStrEnd)
        {
            printf("NSONMI response frame format TempStrEnd err!\r\n");
            return;
        }

        u8Tempdatalen = TempStrEnd - TempStrStart;
        memset(u8TempBuffer,0, sizeof(u8TempBuffer));
        memcpy((char *)u8TempBuffer,(char *)TempStrStart, u8Tempdatalen);
    
        /*convert data to hex*/
        Char2Hex((uint8_t*)u8TempBuffer, (uint8_t*)u8TempHexBuf, u8Tempdatalen / 2);
        Pro_CmdParseHandle((uint8_t*)u8TempHexBuf);
        //NbiotSetCmdAckState(EN_CMD_NSORF_SET, EN_AT_STATE_ACK_OK);

        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
    }
    else if(strstr((char*)p_pu8Buf, "ERROR") != NULL)
    {
        printf("NSORF response err,respose no OK!\r\n");
        //NbiotSetCmdAckState(EN_CMD_NSORF_SET, EN_AT_STATE_ACK_ERR);
    }
    else
    {
        /*do nothing*/
    }
}

static void NbiotRcvOkResponse(char *p_pu8Buf)
{
    uint8_t Loop = 0;
    int32_t ret = -1;
    int8_t ImsiBuf[IMSI_LEN_MAX] = {0};

    for(Loop = 0; Loop < EN_CMD_MAX; Loop++)
    {
        if(EN_AT_STATE_WAIT_ACK == stAtCmdTab[Loop].EnCmdState)
        {
            if (EN_CMD_CIMI == Loop)
            {
                ret = GetInsideStr((int8_t*)p_pu8Buf, ImsiBuf, "+CIMI: ", "\r\n", sizeof(ImsiBuf));
                if(ret != IMSI_LEN_MAX -1)
                {
                    printf("QRY IMSI err,ret = %d!\r\n", ret);
                    NbiotSetCmdAckState(EN_CMD_CIMI, EN_AT_STATE_ACK_ERR);
                    return;
                }
                memcpy(m_stNBIOTItem.u8Imsi, ImsiBuf, IMSI_LEN_MAX);
            }
            NbiotSetCmdAckState((enNbiotAtCmd_t)Loop, EN_AT_STATE_ACK_OK);
            break;
        }
    }
}

static void NbiotRcvErrorResponse(char *p_pu8Buf)
{
    uint8_t Loop = 0;
    for(Loop = 0; Loop < EN_CMD_MAX; Loop++)
    {
        if(EN_AT_STATE_WAIT_ACK == stAtCmdTab[Loop].EnCmdState)
        {
            NbiotSetCmdAckState((enNbiotAtCmd_t)Loop, EN_AT_STATE_ACK_ERR);
            break;
        }
    }
}

static void NbiotNnmigResponse(char *p_pu8Buf)
{
    int32_t RevLen = -1;
    memset(u8TempBuffer, 0, sizeof(u8TempBuffer));
    memset(u8TempHexBuf, 0, sizeof(u8TempHexBuf));
    int32_t Ret = -1;
    /*Reset window timer */
    NbiotWindowTimerReset();
    
    /*send null,moduler enter       idle state*/
    //NbiotAtCommandSend(EN_CMD_QLWULDATAEX_NULL);
    printf("rcv msg:%s", p_pu8Buf);
    p_pu8Buf = strstr((char*)p_pu8Buf, "+NNMI:");
    if(p_pu8Buf != NULL)
    {
        RevLen = GetStrVale((int8_t*)p_pu8Buf, "+NNMI:", ",",       strlen("+NNMI:"));
        if(-1 == RevLen)
        {
            printf("NbiotNnmigResponse err,RevLen = %d!\r\n", RevLen);
            return;
        }
        /*Get report data*/
        Ret = GetInsideStr((int8_t*)p_pu8Buf, u8TempBuffer, ",", "\r\n", sizeof(u8TempBuffer));
        if ((2 * RevLen) != Ret)
        {
            printf("NbiotNnmigResponse err,RevLen = %d, Ret = %d!\r\n", RevLen, Ret);
            return;
        }

        Char2Hex((uint8_t*)u8TempBuffer, (uint8_t*)u8TempHexBuf, RevLen);
        Pro_CmdParseHandle((uint8_t*)u8TempHexBuf);
    }
    else
    {
        /*do nothing*/
    }
}

static void NbiotUdprecvResponse(char *p_pu8Buf)
{
    int32_t RevLen = -1;
    int8_t *pStrEnd = NULL;
    memset(u8TempBuffer, 0, sizeof(u8TempBuffer));
    memset(u8TempHexBuf, 0, sizeof(u8TempHexBuf));
    int32_t Ret = -1;
    /*Reset window timer */
    NbiotWindowTimerReset();

    printf("rcv msg:%s!\r\n", p_pu8Buf);
    p_pu8Buf = strstr((char*)p_pu8Buf, "+UDPRECV: ");
    if(p_pu8Buf != NULL)
    {
        RevLen = GetStrVale((int8_t*)p_pu8Buf, "+UDPRECV: 0,", ",",       strlen("+UDPRECV: 0,"));
        if(-1 == RevLen)
        {
            printf("NbiotUdprecvgResponse received msg invalid!\r\n");
            return;
        }
        /*Get report data*/
        pStrEnd = (int8_t*)strstr((char*)p_pu8Buf + strlen("+UDPRECV: 0,"), ",");
        if(NULL == pStrEnd)
        {
            printf("NbiotNnmigResponse err,RevLen = %d, Ret = %d!\r\n", RevLen, Ret);
        }

        memcpy((uint8_t*)u8TempBuffer, pStrEnd + 1, RevLen);
        Pro_CmdParseHandle((uint8_t*)u8TempBuffer);
    }
    else
    {
        /*do nothing*/
    }
}

static void NbiotTcprecvResponse(char *p_pu8Buf)
{
    int32_t RevLen = -1;
    int8_t *pStrEnd = NULL;
    
    memset(u8TempBuffer, 0, sizeof(u8TempBuffer));
    memset(u8TempHexBuf, 0, sizeof(u8TempHexBuf));
    int32_t Ret = -1;
    /*Reset window timer */
    NbiotWindowTimerReset();

    p_pu8Buf = strstr((char*)p_pu8Buf, "+TCPRECV: ");
    if(p_pu8Buf != NULL)
    {
        RevLen = GetStrVale((int8_t*)p_pu8Buf, "+TCPRECV: 0,", ",",       strlen("+TCPRECV: 0,"));
        if(-1 == RevLen)
        {
            printf("NbiotTcprecvResponse received msg invalid!\r\n");
            return;
        }
        /*Get report data*/
        pStrEnd = (int8_t*)strstr((char*)p_pu8Buf + strlen("+TCPRECV: 0,"), ",");
        if(NULL == pStrEnd)
        {
            printf("NbiotTcprecvResponse err,RevLen = %d, Ret = %d!\r\n", RevLen, Ret);
        }
        memcpy((uint8_t*)u8TempHexBuf, pStrEnd + 1, RevLen);
        Hex2Char(u8TempHexBuf,u8TempBuffer,RevLen);
        printf("rcv msg:%s!\r\n", u8TempBuffer);
        Pro_CmdParseHandle((uint8_t*)u8TempHexBuf);
        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
    }
    else
    {
        /*do nothing*/
    }
}

static void NbiotCgsnResponse(char *p_pu8Buf)
{
    int32_t ret = -1;
    int8_t ImeiBuf[IMEI_LEN_MAX] = {0};

    if(strstr((char*)p_pu8Buf, "+CGSN: ") != NULL)
    {
        /*moduler is enabled*/
        ret = GetInsideStr((int8_t*)p_pu8Buf, ImeiBuf, "+CGSN: ", "\r\n",sizeof(ImeiBuf));
        if(ret != IMEI_LEN_MAX - 1)
        {
            printf("NbiotCgsnResponse err,ret = %d!\r\n", ret);
            NbiotSetCmdAckState(EN_CMD_QRY_IMEI, EN_AT_STATE_ACK_ERR);
            return;
        }

        memcpy(m_stNBIOTItem.u8Imei, ImeiBuf, IMEI_LEN_MAX);

        NbiotSetCmdAckState(EN_CMD_QRY_IMEI, EN_AT_STATE_ACK_OK);
        p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
    }
    else
    {
        NbiotSetCmdAckState(EN_CMD_CEREG_QRY, EN_AT_STATE_ACK_ERR);
    }
}

static void NbiotCimiResponse(char *p_pu8Buf)
{
    int8_t ImsiBuf[IMSI_LEN_MAX] = {0};
    int32_t ret = -1;
    ret = GetInsideStr((int8_t*)p_pu8Buf, ImsiBuf, "+CIMI: ", "\r\n", sizeof(ImsiBuf));
    if(ret != IMSI_LEN_MAX -1)
    {
        printf("QRY IMSI err,ret = %d!\r\n", ret);
        NbiotSetCmdAckState(EN_CMD_CIMI, EN_AT_STATE_ACK_ERR);
        return;
    }

    memcpy(m_stNBIOTItem.u8Imsi, ImsiBuf, IMSI_LEN_MAX);
    NbiotSetCmdAckState(EN_CMD_CIMI, EN_AT_STATE_ACK_OK);
    p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
}

static void NbiotXIICResponse(char *p_pu8Buf)
{
    int32_t ret = -1;
    if(NULL != strstr(p_pu8Buf,"+XIIC:    1"))
    {
        NbiotSetCmdAckState(EN_CMD_XIIC_QRY, EN_AT_STATE_ACK_OK);
    }
    else
    {
        NbiotSetCmdAckState(EN_CMD_XIIC_QRY, EN_AT_STATE_ACK_ERR);
    }
    p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
}

static void NbiotTcpCloseResponse(char *p_pu8Buf)
{
    int32_t ret = -1;
    if(NULL != strstr(p_pu8Buf,"+TCPCLOSE:"))
    {
        NbiotSetCmdAckState(EN_CMD_TCPCLOSE, EN_AT_STATE_ACK_OK);
    }
    p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
}

static void NbiotTcpSetUpResponse(char *p_pu8Buf)
{
    int32_t ret = -1;
    if(NULL != strstr(p_pu8Buf,"+TCPSETUP:0,OK"))
    {
        NbiotSetCmdAckState(EN_CMD_TCP_SETUP, EN_AT_STATE_ACK_OK);
    }
    else
    {
        NbiotSetCmdAckState(EN_CMD_TCP_SETUP, EN_AT_STATE_ACK_ERR);
    }
    p_pu8Buf = strstr((char*)p_pu8Buf, "OK");
}

static void NbiotIPStatusResponse(char *p_pu8Buf)
{
    int32_t ret = -1;
    uint16_t BufferMax = 0;
    if(NULL != strstr(p_pu8Buf,"+IPSTATUS:"))
    {
        if(NULL != strstr(p_pu8Buf,",CONNECT,"))
        {
            NbiotSetCmdAckState(EN_CMD_IPSTATUS, EN_AT_STATE_ACK_OK);
            return;
        }
        else
        {
            NbiotSetCmdAckState(EN_CMD_IPSTATUS, EN_AT_STATE_ACK_ERR);
        }
    }
    else
    {
        NbiotSetCmdAckState(EN_CMD_IPSTATUS, EN_AT_STATE_ACK_ERR);
    }
}




#define MODULE_BUFF_SIZE    (500)
uint8_t ModuleCmdProcessBuf[MODULE_BUFF_SIZE] = {0};
void ModuleDownLinkMsgProc(void)
{
    uint8_t *pu8_RevBuffer = NULL;

    if(0 == NBIOT_GET_RCV_FALG())
    {
        return;
    }

    /*copy buffer to process*/
    memset(ModuleCmdProcessBuf, 0, sizeof(ModuleCmdProcessBuf));
    
    memcpy(ModuleCmdProcessBuf, NbiotGetRcvBuffer(), NbiotGetRcvBufferLen());

    NbiotClearRcvBuffer();
    //printf("%s",ModuleCmdProcessBuf);
    
    pu8_RevBuffer = (uint8_t*)strstr((char*)ModuleCmdProcessBuf, "\r\n");
    while(NULL != pu8_RevBuffer)
    {
        if(strstr((char*)pu8_RevBuffer, "+CFUN: ")!=NULL)
        {
            NbiotCfunQryResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+CGATT: ") != NULL)
        {
            NbiotQryCgattResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+CSQ") != NULL)
        {
            NbiotQryCSQResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+CEREG: ") != NULL)
        {
            NbiotQryCeregResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, (char*)(char*)m_stNBIOTItem.u8IpInfoStr)!=NULL)
        {
            NbiotSetNsorfResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "TUESTATS:") != NULL)
        {
            NbiotTuestatsResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+NNMI:") != NULL)
        {
            NbiotNnmigResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+UDPRECV:") != NULL)
        {
            NbiotUdprecvResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+TCPRECV:") != NULL)
        {
            NbiotTcprecvResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+CGSN: ") != NULL)
        {
            NbiotCgsnResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+XIIC:") != NULL)
        {
            NbiotXIICResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+TCPCLOSE:") != NULL)
        {
            NbiotTcpCloseResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+TCPSETUP:") != NULL)
        {
            NbiotTcpSetUpResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "+IPSTATUS:") != NULL)
        {
            NbiotIPStatusResponse((char*)pu8_RevBuffer);
        }
        else if(strstr((char*)pu8_RevBuffer, "ENTER PSM") != NULL)
        {
            m_stNBIOTItem.u8PsmState = 0;
        }
        else if(strstr((char*)pu8_RevBuffer, "OK")!= NULL)
        {
            NbiotRcvOkResponse((char*)pu8_RevBuffer);
            pu8_RevBuffer = (uint8_t*)strstr((char*)pu8_RevBuffer, "OK");
        }
        else if(strstr((char*)pu8_RevBuffer, "+PBREADY")!= NULL)
        {
            NbiotSetCmdAckState(EN_CMD_AT, EN_AT_STATE_ACK_OK);
        }
        else if(strstr((char*)pu8_RevBuffer, "ERROR")!= NULL)
        {
            NbiotRcvErrorResponse((char*)pu8_RevBuffer);
            pu8_RevBuffer = (uint8_t*)strstr((char*)pu8_RevBuffer, "ERROR");
        }
        pu8_RevBuffer = (uint8_t*)strstr((char*)pu8_RevBuffer, "\r\n:");
    }
}

static void NbiotGPIOCtrlInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = NBIOT_POWER_CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NBIOT_POWER_CTRL_PORT, &GPIO_InitStruct);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(NBIOT_POWER_CTRL_PORT, NBIOT_POWER_CTRL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = NBIOT_WAKEUP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NBIOT_WAKEUP_PORT, &GPIO_InitStruct);
    NbiotWakeUp();
#if 1
    GPIO_InitStruct.Pin = NBIOT_PWKEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NBIOT_PWKEN_PORT, &GPIO_InitStruct);
    NbiotPWK();
#endif
}

static void NbiotHwReset(void)
{
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(NBIOT_POWER_CTRL_PORT, NBIOT_POWER_CTRL_PIN, GPIO_PIN_RESET);
    delay_ms(3000);
    HAL_GPIO_WritePin(NBIOT_POWER_CTRL_PORT, NBIOT_POWER_CTRL_PIN, GPIO_PIN_SET);
}

static void NbiotWakeUp(void)
{
    HAL_GPIO_WritePin(NBIOT_WAKEUP_PORT, NBIOT_WAKEUP_PIN, GPIO_PIN_SET);
    delay_ms(1200);
    HAL_GPIO_WritePin(NBIOT_WAKEUP_PORT, NBIOT_WAKEUP_PIN, GPIO_PIN_RESET);
}

static void NbiotEnablePSM(void)
{
    uint8_t loop = 0;
    uint8_t ret = EN_AT_STATE_BUT;
     /**/
    for(loop = 0; loop < 3; loop++)
    {
        NbiotAtCommandSend(EN_CMD_NVSETPM_LP);
        ret = NbiotWaitAtAck(EN_CMD_NVSETPM_LP);
        if(EN_AT_STATE_ACK_OK == ret)
        {
            return;
        }
    }
    printf("NbiotEnablePSM fail\r\n");
}

static void NbiotDisablePSM(void)
{
    uint8_t loop = 0;
    uint8_t ret = EN_AT_STATE_BUT;
     /**/
    for(loop = 0; loop < 3; loop++)
    {
        FeedDog();
        NbiotAtCommandSend(EN_CMD_NVSETPM_NORMAL);
        ret = NbiotWaitAtAck(EN_CMD_NVSETPM_NORMAL);
        if (EN_AT_STATE_ACK_OK == ret)
        {
            return;
        }
    }
    printf("NbiotDisablePSM fail\r\n");
}

static void NbiotPWK(void)
{
    HAL_GPIO_WritePin(NBIOT_PWKEN_PORT, NBIOT_PWKEN_PIN, GPIO_PIN_SET);
    delay_ms(2500);
    HAL_GPIO_WritePin(NBIOT_PWKEN_PORT, NBIOT_PWKEN_PIN, GPIO_PIN_RESET);
}
#endif

