#include "stdint.h"
#include "SrvErrorNo.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"
#include "character_convert.h"
#include "tim.h"
#include "protocol.h"
#include "lorawan.h"
#include <string.h>
#include "character_convert.h"
#include "common_interface.h"
#include "gpio.h"

#ifdef IOT_NET_TYPE_LORAWAN
static int16_t lorawan_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t lorawan_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t lorawan_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t lorawan_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t lorawan_seek(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset);
static int16_t lorawan_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);

static void LoraWanCopyDataToRcvBuff(void *p_pvParamSrc, void *p_pvParamLen);
static void LoraWanClearBuff(void);
static void* LoraWanGetRcvBuffer(void);
static uint32_t LoraWanRecBuffLen(void);
static int16_t LoraWanLowPower(void);
static int16_t LoraMoudleReset(void);
static uint8_t LoraWanGetCmdAckState(EnAtCMD_t pEnatCmd);
static void LoraWanSetAtCmdState(EnAtCMD_t pEnatCmd,EnLWState_t pEnLWState);
static void LoraWanAtCommandSend(EnAtCMD_t pEnatCmd);
static uint8_t LoraWanWaitAtState(EnAtCMD_t pEnatCmd);
static int16_t LoraWanReadModuleVersion(void);
static int16_t JoinDataUnpack(uint8_t* pBuff);
static int16_t SendDataunpack(uint8_t* pBuff);
static int16_t LowpowerUnpack(uint8_t* pBuff);
static int16_t ReceiveDataUnpack(uint8_t* pBuff);
static int16_t LoraWanPowerUnpack(uint8_t* pBuff);
static int16_t LoraWanResetUnpack(uint8_t* pBuff);
static int16_t LoraWanReceiveACK(uint8_t* pBuff);
static int16_t ReadVerInformationUnpack(uint8_t* pBuff);
static int16_t LoraWanSendData(void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t LoraWanSearchNetWork(void);
void LoraWanCloseWindow(void);
static void loraWanWindowTimerReset(void);
static EnLoraWanState_t SearchNetWorkProc(void);
static void LoraWanHWReset(void);
static void LoraWanPowerCtrlInit(void);
static void ModuleUartEnable(void);
static void ModuleUartDisable(void);
static int32_t ConfigModuleUart(uint32_t p_u32Enable);

#define LORAWAN_BUFF_SIZE (256)
#define  LoraWanWindowTimerDisable Tim21Disable
#define LORAWAN_GET_RCV_FALG() m_stLoraWanItem.u8RcvDataFlag

#define LoraWan_SET_RCV_FALG(flag) do \
{\
    m_stLoraWanItem.u8RcvDataFlag = flag;\
}while(0)

static uint8_t m_u8LoraWanRcvBuffer[LORAWAN_BUFF_SIZE] = {'\0'};          /**< LORAWAN driver rcv buff*/
static uint32_t m_u8LoraWanRcvLen = 0;

StSrvIODevice_t stLORAWAN;
m_stLoraWanItem_t m_stLoraWanItem = 
{
    .u8RcvDataFlag = 0,
    .u8NetState = 0,
    .u8MuartState = 0,
};

static StSrvIODeviceItem_t m_stLorawanDeviceItem;
static StSrvIOOperations_t m_stLorawanOperation = 
{
    .open = lorawan_open,
    .close = lorawan_close,
    .read = lorawan_read,
    .write = lorawan_write,
    .seek = lorawan_seek,
    .ioctl = lorawan_ioctl,
};

stLoraWanNetwork_t stLoraWanNetWork[] = 
{
    {EN_CMD_VER,"AT+VER\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_JOIN,"AT+JOIN\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_LOWPOWER,"AT+LOWPOWER\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_RCVDATA,"AT+CMSGHEX=\"12345678\"\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_POWER,"AT+POWER\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_RESET,"AT+RESET\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_AT,"AT\r\n",EN_AT_LW_STATE_NULL},
    {EN_CMD_ACK,"AT\r\n",EN_AT_LW_STATE_NULL},
};

static void loraWanWindowTimerReset(void)
{
    m_stLoraWanItem.u8LoraWanPsmState = 1;
    Tim21Reset();
}


/** @brief  LoraWan驱动初始化

    @param stLoraWanCfg_t LoarWanPara loraWan参数，入网方式
*/
void Lorawan_configure(stLoraWanCfg_t LoarWanPara)
{
    /* LoraWan入网方式 */
    m_stLoraWanItem.stLoraWanPara = LoarWanPara;
    SrvIODeviceItem_register(&m_stLorawanDeviceItem, EN_ID_LORAWAN, &m_stLorawanOperation, IO_READ | IO_WRITE);
    printf("LoraWan Init!\r\n");
    /* 软中断初始化 */
    SWI_GPIO_Init();
    /* 20s窗口定时器初始化 */
    MX_TIM21_Init();
    /* 串口接收定时器初始化 */
    MX_TIM22_Init();
    /* 初始化 MCU-Module串口 */
    ModuleUartEnable();
    /* 串口回调函数初始化 */
    MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, LoraWanCopyDataToRcvBuff);
    /* LoraWan模组电源管理IO初始化 */
    LoraWanPowerCtrlInit();
    /* 20s窗口期回调函数初始化 */
    Tim21RegisterCb((Tim21ItCallback)LoraWanCloseWindow);
    /* 初始化入网状态为 0，未入网 */
    m_stLoraWanItem.u8NetState = 0;
}

/** @brief  LoraWan-Uart1回调函数

    @param p_pvParamSrc Uart1接收buffer
    @param p_pvParamLen Uart1收到的数据长度
*/
static void LoraWanCopyDataToRcvBuff(void *p_pvParamSrc, void *p_pvParamLen)
{
    /* copy uart buffer data to LOWANRAN driver rcv buff */
    memcpy(m_u8LoraWanRcvBuffer, (uint8_t*)p_pvParamSrc, *(uint32_t *)p_pvParamLen);
    m_u8LoraWanRcvLen = *(uint32_t *)p_pvParamLen;

    /* Flag=1 indecated one frame rev finished */
    LoraWan_SET_RCV_FALG(1);
}

/** @brief  LoraWan receiver buffer
*/
static void* LoraWanGetRcvBuffer(void)
{
    return m_u8LoraWanRcvBuffer;
}

/** @brief  LoraWan receiver data length
*/
static uint32_t LoraWanRecBuffLen(void)
{
    return m_u8LoraWanRcvLen;
}

/** @brief  LoraWan模组电源管理IO口初始化
    @brief  GPIOB、PIN1，高电平使能（供电）
*/
static void LoraWanPowerCtrlInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    
    GPIO_InitStruct.Pin = LORAWAN_POWER_CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORAWAN_POWER_CTRL_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LORAWAN_POWER_CTRL_PORT, LORAWAN_POWER_CTRL_PIN, GPIO_PIN_SET);
    delay_ms(8000);
    HAL_GPIO_WritePin(LORAWAN_POWER_CTRL_PORT, LORAWAN_POWER_CTRL_PIN, GPIO_PIN_RESET);
}

static void LoraWanHWReset(void)
{
    HAL_GPIO_WritePin(LORAWAN_POWER_CTRL_PORT, LORAWAN_POWER_CTRL_PIN, GPIO_PIN_SET);
    delay_ms(3000);
    HAL_GPIO_WritePin(LORAWAN_POWER_CTRL_PORT, LORAWAN_POWER_CTRL_PIN, GPIO_PIN_RESET);
}
static int16_t lorawan_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    p_pstSrvIODevice->pstDeviceItem = &m_stLorawanDeviceItem;
    p_pstSrvIODevice->pvPrivateData = (void *)0;
    p_pstSrvIODevice->pstDeviceItem->u8Count++;
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 1)
    {
        MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, LoraWanCopyDataToRcvBuff);
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
        return ERR_BEYOND_MAX;
    }

    /* register lorawan callback func */
    MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, LoraWanCopyDataToRcvBuff);
    return ERR_OK;
}

/** @brief  close lorawan driver
*/
static int16_t lorawan_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 0)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
    }
    
    p_pstSrvIODevice->pstDeviceItem = NULL;
    
    LoraWan_SET_RCV_FALG(0);
    
    printf("LoraWan have alreadly closed!\r\n");
    return ERR_OK;
}

/** @brief read from lorawan driver
*/
static int16_t lorawan_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= 500 )
    {
        return ERR_OUT_OF_RANGE;
    }
    return ERR_OK;
}

/** @brief 初始化 MCU-module，Uart1
*/
static void ModuleUartEnable(void)
{
    /* 未初始化Uart时执行 */
    if(m_stLoraWanItem.u8MuartState == 0)
    {
        MuartInit();
        m_stLoraWanItem.u8MuartState = 1;
    }
}

/** @brief Disable MCU-module，Uart1
*/
static void ModuleUartDisable(void)
{
    if(m_stLoraWanItem.u8MuartState == 1)
    {
        MuartDeinit();
        m_stLoraWanItem.u8MuartState = 0;
    }
}

static int32_t ConfigModuleUart(uint32_t p_u32Enable)
{
    if(p_u32Enable == 1)
    {
        ModuleUartEnable();
        return ERR_OK;
    }
    else if(p_u32Enable == 0)
    {
        ModuleUartDisable();
        return ERR_OK;
    }
    else
    {
        return ERR_FAULT;
    }
}

static uint8_t LoraWanGetCmdAckState(EnAtCMD_t pEnatCmd)
{
    return stLoraWanNetWork[pEnatCmd].EnLWState;
}

static void LoraWanSetAtCmdState(EnAtCMD_t pEnatCmd,EnLWState_t pEnLWState)
{
    stLoraWanNetWork[pEnatCmd].EnLWState = pEnLWState;
}

static void LoraWanAtCommandSend(EnAtCMD_t pEnatCmd)
{
    UartSend(&huart1,(uint8_t*)stLoraWanNetWork[pEnatCmd].LoraWanAtCmd,strlen((char*)stLoraWanNetWork[pEnatCmd].LoraWanAtCmd));

    stLoraWanNetWork[pEnatCmd].EnLWState = EN_AT_LW_STATE_WAIT_ACK;
}

static uint8_t LoraWanWaitAtState(EnAtCMD_t pEnatCmd)
{
    uint16_t i = 0;
    
    while(!((i > 4) ||
         (EN_AT_LW_STATE_WAIT_ACK != LoraWanGetCmdAckState(pEnatCmd))))
    {
         i++;
         delay_ms(1000);
    }
    if(i > 4)
    {
        LoraWanSetAtCmdState(pEnatCmd,EN_AT_LW_STATE_TIMEOUT);
    }
    return LoraWanGetCmdAckState(pEnatCmd);
}

void LoraWanCloseWindow(void)
{
    static uint8_t u8FailCount = 0;
    
    if(LoraWanGetCmdAckState(EN_CMD_ACK) != EN_AT_LW_STATE_ACK_OK)
    {
        printf("LoraWan send data fail,u8FailCount = %d!\r\n",u8FailCount);
        u8FailCount++;
        if(u8FailCount >= 3)
        {
            u8FailCount= 0;
            m_stLoraWanItem.u8NetState = 0;
            LoraMoudleReset();
        }
    }
    else
    {
        m_stLoraWanItem.u8NetState = 1;
        u8FailCount = 0;
        printf("LoraWan send data success,u8FailCount = %d!\r\n",u8FailCount);
    }
    
    LoraWanLowPower();
    LoraWanWindowTimerDisable();
    m_stLoraWanItem.u8LoraWanPsmState = 0;
}


static int16_t lorawan_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    int16_t Ret = 0;

    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= LORAWAN_BUFF_SIZE)
    {
        return ERR_OUT_OF_RANGE;
    }
    
    Ret = LoraWanSendData(p_pvBuffer,p_u16BufferSize);
    
    if(ERR_OK == Ret)
    {
        loraWanWindowTimerReset();
    }
    else
    {
        printf("LoraWan senddata fail,Ret = %d!\r\n",Ret);
    }
    return Ret;
}

static int16_t lorawan_seek(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset)
{
    if(p_u32Offset >= 200)
    {
        return ERR_OUT_OF_RANGE;
    }
    
    p_pstSrvIODevice->u32Position = p_u32Offset;
    
    return ERR_OK;
}

static int16_t LoraWanSearchNetWork(void)
{
    EnLoraWanState_t State = EN_LORAWAN_BUTT;
    uint8_t i =0;

    if(m_stLoraWanItem.u8NetState == 1)
    {
        LoraWanAtCommandSend(EN_CMD_AT);
        delay_ms(3000);
        printf("LoraWan wake!\r\n");
        return ERR_OK;
    }
    
    if(EN_TRANS_MODE_ABP == m_stLoraWanItem.stLoraWanPara.EnTransMode)
    {
        printf("LoraWan type is ABP!\r\n");
        return ERR_OK;
    }
    
    
    else if(EN_TRANS_MODE_OTAA == m_stLoraWanItem.stLoraWanPara.EnTransMode)
    {
        for(i = 0; i < 3; i++)
        {
            FeedDog();
            printf("LoraWan type is OTAA!\r\n");
            State = SearchNetWorkProc();
            if(EN_LORAWAN_SUCCESS == State)
            {
                m_stLoraWanItem.u8NetState = 1;
                printf("LoraWan is working!\r\n");
                return ERR_OK;
            }
            else
            {
                if( i == 0)
                {
                    LoraMoudleReset();
                    printf("LoraWan join fail,State= %d!!!\r\n", State);
                }
                else if( i == 1)
                {
                    LoraWanHWReset();
                    printf("LoraWan module has been reset by hardware!\r\n");
                }
            }
        }
        if( i == 3)
        {
            LoraWanLowPower();
            m_stLoraWanItem.u8NetState = 0;
            printf("LoraWanSearchNetWork fail!\r\n");
            return ERR_FAULT;
        }
    }
    else
	{
	    /**/
		printf("Lorawan EnTransMode err,%d!\r\n", m_stLoraWanItem.stLoraWanPara.EnTransMode);
	}
    return ERR_OK;
}

static EnLoraWanState_t SearchNetWorkProc(void)
{
    uint8_t i = 0;

    EnLWState_t Ret = EN_AT_LW_STATE_BUT;

    for(i = 0; i < 6; i++)
    {
        FeedDog();
        LoraWanAtCommandSend(EN_CMD_JOIN);
        Ret = (EnLWState_t)LoraWanWaitAtState(EN_CMD_JOIN);
        delay_ms(3000);
        if(Ret != EN_AT_LW_STATE_ACK_OK)
        {
            printf("LoraWan join network fail value is %d\r\n",Ret);
        }
        
        if(EN_AT_LW_STATE_ACK_OK == LoraWanGetCmdAckState(EN_CMD_JOIN))
        {
            printf("LoraWan have already joined network!\r\n");
            return EN_LORAWAN_SUCCESS;
        }
        else if(EN_AT_LW_STATE_BUSY == LoraWanGetCmdAckState(EN_CMD_JOIN))
        {
            printf("LoraWan is busying!\r\n");
            continue;
        }
        else if(EN_AT_LW_STATE_ACK_ERR == LoraWanGetCmdAckState(EN_CMD_JOIN))
        {
            printf("LoraWan joined nedtwork fail!\r\n");
            return EN_LORAWAN_FAIL;
        }
        else if(EN_AT_LW_STATE_BUT == LoraWanGetCmdAckState(EN_CMD_JOIN))
        {
            printf("LoraWan is not working!\r\n");
            return EN_LORAWAN_BUTT;
        }
    }
    return EN_LORAWAN_SUCCESS;
}

static void LoraWanClearBuff(void)
{
    memset(m_u8LoraWanRcvBuffer,'\0',sizeof(m_u8LoraWanRcvBuffer));
    LoraWan_SET_RCV_FALG(0);
}

#define LORAWAN_SEND_DATA_LEN_MAX (256)

#define LORAWAN_WAIT_SEND_COUNT 5

int8_t ReadAtBuff[LORAWAN_SEND_DATA_LEN_MAX] = {'\0'};
int8_t ReadHexBuff[LORAWAN_SEND_DATA_LEN_MAX] = {'\0'};
int16_t LoraWanSendData(void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    if(NULL == p_pvBuffer)
    {
        printf("LoraWan send data which is NULL!\r\n");
        return ERR_BADF;
    }
    if(p_u16BufferSize > LORAWAN_SEND_DATA_LEN_MAX)
    {
        printf("LoraWan data is beyond!\r\n");
        return ERR_BEYOND_MAX;
    }
    
    memset(ReadAtBuff,0,sizeof(ReadAtBuff));
    memset(ReadHexBuff,0,sizeof(ReadHexBuff));
    Hex2Char(p_pvBuffer,ReadHexBuff,p_u16BufferSize);
    
    strcat((char*)ReadAtBuff,"AT+CMSGHEX=");
    strcat((char*)ReadAtBuff,(char*)ReadHexBuff);
    strcat((char*)ReadAtBuff,"\r\n");

    UartSend(&huart1,(uint8_t *)ReadAtBuff,strlen((char*)ReadAtBuff));
    printf("LoraWanSendData send msg is %s", ReadAtBuff);
    LoraWanSetAtCmdState(EN_CMD_ACK,EN_AT_LW_STATE_WAIT_ACK);

    return ERR_OK;
}

static int16_t LoraWanLowPower(void)
{
    uint8_t Ret = 0;

    LoraWanAtCommandSend(EN_CMD_LOWPOWER);

    Ret = LoraWanWaitAtState(EN_CMD_LOWPOWER);
    
    if(Ret != EN_AT_LW_STATE_ACK_OK)
    {
        printf("LoraWan Enter lowpower fail,Ret = %d!\r\n",Ret);
    }
    else
    {
        printf("LoraWan Enter lowpower success!\r\n");
    }
    return Ret;
}

#define HandleDataLen (512)
uint8_t MoudleHandleData[HandleDataLen] = {'\0'};
stLoraWanReceivehandle_t stLoraWanDownlinkMsgTab[EN_MAX_FUN] = 
{
    {EN_JOIN_FUN, JoinDataUnpack},
    {EN_SENDDATA_FUN, SendDataunpack},
    {EN_RECBUFF_FUN, ReceiveDataUnpack},
    {EN_LOWPOWER_FUN, LowpowerUnpack},
    {EN_VER_FUN, ReadVerInformationUnpack},
    {EN_POWER_FUN, LoraWanPowerUnpack},
    {EN_RESET_FUN, LoraWanResetUnpack},
    {EN_ACK_FUN,LoraWanReceiveACK},
};

void ModuleDownLinkMsgProc(void)
{
    if(LORAWAN_GET_RCV_FALG() == 0)
    {
        return;
    }
    memset(MoudleHandleData,'\0', strlen((char*)MoudleHandleData));
    memcpy(MoudleHandleData, LoraWanGetRcvBuffer(), LoraWanRecBuffLen());
    LoraWanClearBuff();
    /* Join network! */

    if(strstr((char*)MoudleHandleData,"+JOIN") != NULL)
    {
        stLoraWanDownlinkMsgTab[EN_JOIN_FUN].fp_MsgProc(MoudleHandleData);
    }

    /*Send data unpack*/
    if(strstr((char*)MoudleHandleData,"+CMSGHEX") != NULL)
    {
        if(strstr((char*)MoudleHandleData,"PORT") != NULL)
        {
            /* Receive downlinkdata */
            stLoraWanDownlinkMsgTab[EN_RECBUFF_FUN].fp_MsgProc(MoudleHandleData);
        }
        else if(strstr((char*)MoudleHandleData,"ACK Received") != NULL)
        {
            stLoraWanDownlinkMsgTab[EN_ACK_FUN].fp_MsgProc(MoudleHandleData);
        }
        else
        {
            /* Send uplinkdata */
            stLoraWanDownlinkMsgTab[EN_SENDDATA_FUN].fp_MsgProc(MoudleHandleData);
        }  
    }

    /*LoraWan enter sleep mode*/
    if(strstr((char*)MoudleHandleData,"+LOWPOWER") != NULL)
    {
        stLoraWanDownlinkMsgTab[EN_LOWPOWER_FUN].fp_MsgProc(MoudleHandleData);
    }
    /*LoraWan read version information*/
    if(strstr((char*)MoudleHandleData,"+VER") != NULL)
    {
        stLoraWanDownlinkMsgTab[EN_VER_FUN].fp_MsgProc(MoudleHandleData);
    }
    /*Lorawan read power*/
    if(strstr((char*)MoudleHandleData,"+POWER") != NULL)
    {
        stLoraWanDownlinkMsgTab[EN_POWER_FUN].fp_MsgProc(MoudleHandleData);
    }
    if(strstr((char*)MoudleHandleData,"+RESET") != NULL)
    {
        stLoraWanDownlinkMsgTab[EN_RESET_FUN].fp_MsgProc(MoudleHandleData);
    }
}
static int16_t JoinDataUnpack(uint8_t* pBuff)
{
    if(NULL == pBuff)
    {
        printf("LoraWan receive invalid data!");
        return ERR_BADF;
    }
    if((strstr((char*)pBuff,"Joined already") != NULL) || (strstr((char*)pBuff,"Network joined") != NULL))
    {
        LoraWanSetAtCmdState(EN_CMD_JOIN, EN_AT_LW_STATE_ACK_OK);
        return EN_JOIN_SUCCESS;
    }
    else if(strstr((char*)pBuff, "Join failed") != NULL)
    {
        LoraWanSetAtCmdState(EN_CMD_JOIN, EN_AT_LW_STATE_ACK_ERR);
    }
    else if(strstr((char*)pBuff, "LoRaWAN modem is busy") !=NULL)
    {
        LoraWanSetAtCmdState(EN_CMD_JOIN, EN_AT_LW_STATE_BUSY);
        return EN_JOIN_BUSY;
    }
    else
    {
        LoraWanSetAtCmdState(EN_CMD_JOIN, EN_AT_LW_STATE_BUT);
        return EN_JOIN_BUTT;
    }
    return 0;
}

static int16_t SendDataunpack(uint8_t* pBuff)
{
    if(NULL == pBuff)
    {
        printf("LoraWan receive invalid data!");
        return ERR_BADF;
    }

    if(strstr((char*)pBuff,"Wait ACK") != NULL)
    {
        return EN_SEND_DATA_SUCCESS;
    }
    return ERR_OK;
}

static int16_t LowpowerUnpack(uint8_t* pBuff)
{
    if(NULL == pBuff)
    {
        printf("LoraWan receive invalid data!");
        return ERR_BADF;
    }
    if(strstr((char*)pBuff,"SLEEP") != NULL)
    {
        LoraWanSetAtCmdState(EN_CMD_LOWPOWER,EN_AT_LW_STATE_ACK_OK);
    }
    else
    {
        LoraWanSetAtCmdState(EN_CMD_LOWPOWER,EN_AT_LW_STATE_ACK_ERR);
    }
    return ERR_OK;
}

#define LORAWAN_RCVDATA_MAX_LEN (256)
uint8_t u8LoraWanReceivedata[LORAWAN_RCVDATA_MAX_LEN] = {'\0'};
uint8_t u8LoraWanRcvHex[LORAWAN_RCVDATA_MAX_LEN / 2] = {'\0'};

static int16_t ReceiveDataUnpack(uint8_t* pBuff)
{
    uint8_t u8StrLoraWanRSSI[3] = {0};
    int32_t Ret = 0;
    if(pBuff == NULL)
    {
        printf("LoraWan receive data which is empty!\r\n");
        return ERR_BADF;
    }
    if(strstr((char*)pBuff,"PORT") != NULL)
    {
        memset(u8LoraWanReceivedata,0,sizeof(u8LoraWanReceivedata));
        Ret = GetInsideStr((int8_t *)pBuff,(int8_t *)u8LoraWanReceivedata,"RX: \"","\"\r\n");
        Ret = GetInsideStr((int8_t *)pBuff,(int8_t *)u8StrLoraWanRSSI,"RSSI -",", SNR");
        m_stLoraWanItem.u32SignalStrength = atoi((char *)u8StrLoraWanRSSI);
        if(Ret == -1)
        {
            return Ret;
        }
        LoraWanSetAtCmdState(EN_CMD_ACK, EN_AT_LW_STATE_ACK_OK);
        LoraWanSetAtCmdState(EN_CMD_RCVDATA, EN_AT_LW_STATE_ACK_OK);
        Char2Hex((uint8_t*)u8LoraWanReceivedata, (uint8_t*)u8LoraWanRcvHex, strlen((char *)u8LoraWanReceivedata));
        printf("LoraWan receive data is %s\r\n",u8LoraWanReceivedata);
        Pro_CmdParseHandle((uint8_t*)u8LoraWanRcvHex);
        printf("LoraWan receive data success!\r\n");
        return 0;
    }

    return -1;
}

static int16_t ReadVerInformationUnpack(uint8_t* pBuff)
{
    if(NULL == pBuff)
    {
        printf("LoraWan receive invalid data!");
        return ERR_BADF;
    }
    if(strstr((char*)pBuff,"+VER") != NULL)
    {
        LoraWanSetAtCmdState(EN_CMD_VER, EN_AT_LW_STATE_ACK_OK);
    }
    else
    {
        LoraWanSetAtCmdState(EN_CMD_VER, EN_AT_LW_STATE_ACK_ERR);
    }

    return ERR_OK;
}

static void* MoudleRcvBuf(void)
{
    return MoudleHandleData;
}

static int16_t LoraWanPowerUnpack(uint8_t* pBuff)
{
    int32_t pPowerValue = NULL;
    if(NULL == pBuff)
    {
        printf("LoraWan receive invalid data!\r\n");
        return ERR_BADF;
    }
    if(strstr((char*)pBuff,"+POWER") != NULL)
    {
        LoraWanSetAtCmdState(EN_CMD_POWER,EN_AT_LW_STATE_ACK_OK);

        pPowerValue = GetStrVale((int8_t*)pBuff, "+POWER: ", "\r\n", strlen("+POWER: "));
        if (-1 != pPowerValue)
        {
            m_stLoraWanItem.s32TxPower = pPowerValue;
        }

        printf("LoraWan Module power is %d\r\n", pPowerValue);
    }
    else
    {
        LoraWanSetAtCmdState(EN_CMD_POWER, EN_AT_LW_STATE_ACK_ERR);
    }

    return ERR_OK;
}

static int16_t LoraWanResetUnpack(uint8_t* pBuff)
{
    if(NULL == pBuff)
   {
       printf("LoraWan reset invalid data!");
       return ERR_BADF;
   }
   if(strstr((char*)pBuff,"+RESET") != NULL)
   {
        LoraWanSetAtCmdState(EN_CMD_RESET,EN_AT_LW_STATE_ACK_OK);
   }
   else
   {
        LoraWanSetAtCmdState(EN_CMD_RESET,EN_AT_LW_STATE_ACK_ERR);
   }
   return ERR_OK;
}

static int16_t LoraWanReceiveACK(uint8_t* pBuff)
{
    if(NULL == pBuff)
   {
       printf("LoraWan ACK invalid!");
       return ERR_BADF;
   }
   if(strstr((char*)pBuff,"ACK Received") != NULL)
   {
        LoraWanSetAtCmdState(EN_CMD_ACK,EN_AT_LW_STATE_ACK_OK);
   }
   else
   {
        LoraWanSetAtCmdState(EN_CMD_ACK,EN_AT_LW_STATE_ACK_ERR);
   }
   return ERR_OK;
}


static int16_t LoraWanReadModuleVersion(void)
{
    EnLWState_t Ret = EN_AT_LW_STATE_BUT;
    
    LoraWanAtCommandSend(EN_CMD_VER);
    Ret = (EnLWState_t)LoraWanWaitAtState(EN_CMD_VER);
    
    if(Ret != EN_AT_LW_STATE_ACK_OK)
    {
        printf("LoraWan At State Value is %d\r\n",Ret);
    }
    else
    {
        printf("LoraWan Version information is %s",(char*)MoudleRcvBuf());
    }
    return Ret;
}

static int16_t LoraMoudleReset(void)
{
    uint8_t Ret = 0;

    LoraWanAtCommandSend(EN_CMD_RESET);

    Ret = LoraWanWaitAtState(EN_CMD_RESET);
    
    if(Ret != EN_AT_LW_STATE_ACK_OK)
    {
        printf("LoraWan reset fail value is %d\r\n",Ret);
    }
    else
    {
        printf("LoraWan have alreadly reseted!\r\n");
    }
    return Ret;
}

static int16_t lorawan_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    switch(p_u32Command)
    {
        case EN_CMD_ENTER_LOWPOWER:
        {
            if(0 == m_stLoraWanItem.u8LoraWanPsmState)
            {
                ModuleUartEnable();
                LoraWanLowPower();
            }
            return ERR_OK;
        }break;
        case EN_CMD_GET_RSSI:
        {
            *(uint32_t *)p_u32Arg = m_stLoraWanItem.u32SignalStrength;
        }break;
        case EN_CMD_SEARCH_NETWORK:
        {
            ModuleUartEnable();
            return LoraWanSearchNetWork();
        }
        case EN_CMD_GET_LPSATAE:
        {
            *(uint32_t *)p_u32Arg = m_stLoraWanItem.u8LoraWanPsmState;
            break;
        }
        case EN_CMD_SET_MUART_STATE:
        {
            return ConfigModuleUart(*(uint32_t *)p_u32Arg);
        }
        case EN_CMD_GET_LORAWAN_NET_STATE:
        {
            *(uint32_t *)p_u32Arg = m_stLoraWanItem.u8NetState;
            break;
        }
        case EN_CMD_GET_LORAWAN_MoudleVER:
        {
            ModuleUartEnable();
            if(EN_AT_LW_STATE_ACK_OK != LoraWanReadModuleVersion())
            {
                return EN_AT_LW_STATE_ACK_ERR;
            }
            memcpy((char *)p_u32Arg,(char *)m_stLoraWanItem.u8LoraVer, LORAWAN_VERSION_MAX);
        }
        default : break;
    }
    return 0;
}
#endif
