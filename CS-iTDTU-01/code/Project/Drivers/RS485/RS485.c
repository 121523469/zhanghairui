#include "RS485.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include "SrvErrorNo.h"
#include "SrvIODeviceItem.h"
#include "common_interface.h"
#include "musart.h"

#define RS485_BUFF_SIZE             (512u)
#define RS485_COM                   &huart2
#define RS485_DEFAULT_BAUD          9600
#define RS485_POWER_GPIO            GPIOA
#define RS485_POWER_PIN             GPIO_PIN_1
#define RS485_TR_GPIO               GPIOA
#define RS485_TR_PIN                GPIO_PIN_1
#define RS485POWRON()               do{HAL_GPIO_WritePin(RS485_POWER_GPIO,RS485_POWER_PIN,GPIO_PIN_RESET);}while(0)
#define RS485POWROFF()              do{HAL_GPIO_WritePin(RS485_POWER_GPIO,RS485_POWER_PIN,GPIO_PIN_SET);}while(0)
#define RS485_RCV_FLAG_SET(flag)    do{m_strs485Paras.u8RcvFlag = flag;}while(0)
#define RS485_RCV_LEN_CLR()         do{m_strs485Paras.u32RcvLen = 0;}while(0)
#define RS485_RCV_FLAG_GET()        (m_strs485Paras.u8RcvFlag)



static int16_t rs485_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t rs485_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t rs485_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t rs485_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t rs485_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem ,StSrvIODevice_t* p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);
static void rs485GPIOInit();
static void rs485UartEnable();
static void rs485UartDisable();
static void* getrs485RcvBuff();
static void rs485ClearRcvBuff();
static void rs485callback(void *p_pvParamSrc, void *p_pvParamLen);
static EnRS485Err_t findBaudRate(uint32_t p_u32BaudRate);
static EnRS485Err_t rs485UartConfig(EnRS485UartStatus_t p_enUartStatus);
static int8_t rs485SendData(void *p_pvBuffer, uint16_t p_u16BufferSize);
static EnRS485Err_t rs485TRStatusSet(EnRS485TRStatus_t p_enTRStatus);

StSrvIODevice_t stRS485Device;
static StSrvIODeviceItem_t m_stRS485DeviceItem;
static StSrvIOOperations_t m_stRS485Operation = 
{
    .open = rs485_open,
    .close = rs485_close,
    .read = rs485_read,
    .write = rs485_write,
    .ioctl = rs485_ioctl,
};

static RS485Paras_t m_strs485Paras = 
{
    .u32BaudRate = RS485_DEFAULT_BAUD,
    .u8RcvFlag = 0,
    .EnUartStatus = RS485_UART_DISABLE,
    .EnTRStatus = RS485_RX,
    .u32RcvLen = 0,
};

static uint8_t m_u8RS485ReceiveBuffer[RS485_BUFF_SIZE];
static const uint32_t m_u32BaudRate[] = {9600,38400,57600,115200};

void RS485Configure()
{
    //SrvIODeviceItem_register(&m_stRS485DeviceItem, EN_ID_RS485, &m_stRS485Operation, IO_READ | IO_WRITE);
    //rs485GPIOInit();
    //rs485UartEnable();
    //MX_TIM3_Init();
    //RS485_RCV_FLAG_SET(0);
}


static int16_t rs485_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    p_pstSrvIODevice->pvPrivateData = (void *)0;
    p_pstSrvIODevice->pstDeviceItem->u8Count++;

    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 1)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;

        UartRcvFunCbRegister(RS485_COM,EN_CB_ONRECEIVE,rs485callback);
        return ERR_BEYOND_MAX;
    }

    if (EN_ID_RS485 == p_pstSrvIODeviceItem->enDeviceId)
    {
        p_pstSrvIODevice->pvPrivateData = (void *)0;
    }
    return ERR_OK;
}

static int16_t rs485_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 0)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
    }
    p_pstSrvIODevice->pstDeviceItem = NULL;

    return ERR_OK;
}

static int16_t rs485_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem ,StSrvIODevice_t* p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    switch(p_u32Command)
    {
        
    }
    return ERR_OK;
}

static int16_t rs485_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= RS485_BUFF_SIZE )
    {
        return ERR_OUT_OF_RANGE;
    }

    if(1 == RS485_RCV_FLAG_GET())
    {
        memmove(p_pvBuffer,getrs485RcvBuff(),p_u16BufferSize);
        RS485_RCV_FLAG_SET(0);
    }
    else
    {
        return ERR_FAULT;
    }
    
    return ERR_OK;
}

static int16_t rs485_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    int8_t EnErr = RS485_ERR_OK;

    if(p_pstSrvIODevice->u32Position + p_u16BufferSize > RS485_BUFF_SIZE)
    {
        return ERR_OUT_OF_RANGE;
    }
    
    EnErr = rs485SendData(p_pvBuffer, p_u16BufferSize);
    
    return EnErr;
}

static void rs485GPIOInit()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = RS485_POWER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_POWER_GPIO, &GPIO_InitStruct);
    RS485POWROFF();

    GPIO_InitStruct.Pin = RS485_TR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_TR_GPIO, &GPIO_InitStruct);
    rs485TRStatusSet(RS485_RX);
}

static void rs485callback(void *p_pvParamSrc, void *p_pvParamLen)
{
    if((NULL == p_pvParamSrc) || (0 == p_pvParamLen))
    {
        return;
    }
    
    if(*(uint32_t *)p_pvParamLen > RS485_BUFF_SIZE)
    {
        return;
    }
    
    rs485ClearRcvBuff();
    memmove(getrs485RcvBuff(),(uint8_t *)p_pvParamSrc,*(uint32_t *)p_pvParamLen);
    m_strs485Paras.u32RcvLen = *(uint32_t *)p_pvParamLen;
    RS485_RCV_FLAG_SET(1);
}

static void *getrs485RcvBuff()
{
    return m_u8RS485ReceiveBuffer;
}

static void rs485ClearRcvBuff()
{
    memset(getrs485RcvBuff(),0,sizeof(getrs485RcvBuff()));
}

static EnRS485Err_t rs485TRStatusSet(EnRS485TRStatus_t p_enTRStatus)
{
    if(RS485_TX == p_enTRStatus)
    {
        HAL_GPIO_WritePin(RS485_TR_GPIO,RS485_TR_PIN,GPIO_PIN_SET);
        delay_ms(1);
    }
    else if(RS485_RX == p_enTRStatus)
    {
        HAL_GPIO_WritePin(RS485_TR_GPIO,RS485_TR_PIN,GPIO_PIN_RESET);
        delay_ms(1);
    }
    else
    {
        return RS485_ERR_INVALIDPARA;
    }
    return RS485_ERR_OK;
}

static EnRS485Err_t findBaudRate(uint32_t p_u32BaudRate)
{
    uint8_t i;

    for(i = 0u; i < sizeof(m_u32BaudRate) / 4u; i++)
    {
        if(p_u32BaudRate == m_u32BaudRate[i])
        {
            return RS485_ERR_OK;
        }
    }
    
    return RS485_ERR_NULL;
}

static void rs485UartEnable()
{
    if(RS485_UART_DISABLE == m_strs485Paras.EnUartStatus)
    {
        UartInit(RS485_COM,m_strs485Paras.u32BaudRate);
        m_strs485Paras.EnUartStatus = RS485_UART_ENABLE;
    }
}

static void rs485UartDisable()
{
    if(RS485_UART_ENABLE == m_strs485Paras.EnUartStatus)
    {
        UartDeinit(RS485_COM);
        m_strs485Paras.EnUartStatus = RS485_UART_DISABLE;
    }
}

static EnRS485Err_t rs485UartConfig(EnRS485UartStatus_t p_enUartStatus)
{
    if(RS485_UART_ENABLE == p_enUartStatus)
    {
        rs485UartEnable();
    }
    else if(RS485_UART_DISABLE == p_enUartStatus)
    {
        rs485UartDisable();
    }
    else
    {
        return RS485_ERR_INVALIDPARA;
    }
    
    return RS485_ERR_OK;
}

static int8_t rs485SendData(void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    int8_t Err = RS485_ERR_OK;
    
    if(NULL == p_pvBuffer)
    {
        return RS485_ERR_NULL;
    }

    rs485TRStatusSet(RS485_TX);
    Err = UartSend(RS485_COM,(uint8_t *)p_pvBuffer,p_u16BufferSize);
    rs485TRStatusSet(RS485_RX);

    return Err;
}



