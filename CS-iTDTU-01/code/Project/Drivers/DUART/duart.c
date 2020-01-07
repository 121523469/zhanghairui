#include "duart.h"
#include "stdio.h"
#include "SrvErrorNo.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"
#include <string.h>
#include "ioctlcmd.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <LowLevelIOInterface.h>
#include "communication.h"
#include "stm32l0xx_ll_lpuart.h"
#include "main.h"
#include "musart.h"

UART_HandleTypeDef *duart = &hlpuart1;
#define DUART_INIT() MX_LPUART1_UART_Init()
#define DUART_DeInit() HAL_UART_DeInit(duart)

EnMenuState_t DuartMenuState = EN_DUART_MENU_IDLE;

volatile uint8_t m_sCallBackFlag = 0;
volatile uint8_t u8Putchar_Timeout_flag = 0;
StSrvIODevice_t stDebugUartDevice;
static int16_t duart_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t duart_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t duart_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem ,StSrvIODevice_t* p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);

static StSrvIODeviceItem_t m_stduartDeviceItem;
static StSrvIOOperations_t m_stduartOperation = 
{
    .open = duart_open,
    .close = duart_close,
    .ioctl = duart_ioctl,
};

/*************************************************************************************************/
/** @brief 注册Debug Uart Device
*/
/*************************************************************************************************/
void duart_configure()
{
    SrvIODeviceItem_register(&m_stduartDeviceItem, EN_ID_DUART, &m_stduartOperation, IO_READ | IO_WRITE);
    DUART_INIT();
    MX_TIM2_Init();
}

/*************************************************************************************************/
/** @brief Open Debug Uart Device
*/
/*************************************************************************************************/
static int16_t duart_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
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

/*************************************************************************************************/
/** @brief Close Debug Uart Device
*/
/*************************************************************************************************/
static int16_t duart_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
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

static int16_t duart_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem ,StSrvIODevice_t* p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
  if(p_pstSrvIODeviceItem == NULL)
  {
    return EN_ERR_DUART_NULL;
  }
  else if(p_pstSrvIODevice == NULL)
  {
    return EN_ERR_DUART_NULL;
  }

  switch(p_u32Command)
  {
    case EN_CMD_REGSERCALLBACK:
    {
      /* register nbiot callback func */
      MuartRevdFrameProcRegister(EN_CB_ONRECEIVE, (pfUartRecvFrameCB)p_u32Arg);
    }break;
    case EN_CMD_GET_DUART_MENU_STATE:
    {
        *(uint32_t *)p_u32Arg = DUART_GET_MENU_STATE();
    }break;
    case EN_CMD_EXIT_MENU:
    {
        DUART_SET_MENU_STATE(EN_DUART_MENU_IDLE);
        Tim2Disable();
        DuartEnableIt();
    }break;
  }
  return EN_ERR_DUART_OK;
}

/**
* @brief This function handles AES and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
*/
void AES_RNG_LPUART1_IRQHandler(void)
{
    static char temp = 0;

    if(LL_LPUART_IsActiveFlag_WKUP(LPUART1) && LL_LPUART_IsEnabledIT_WKUP(LPUART1))
    {
        LL_LPUART_ClearFlag_WKUP(LPUART1);
    }

    if(EN_DUART_MENU_IDLE == DUART_GET_MENU_STATE())
    {
        if(LPUART1->ISR & (UART_FLAG_RXNE))
        {
            temp = (uint8_t)READ_REG(duart->Instance->RDR);
            if('\r' == temp)
            {
                DUART_SET_MENU_STATE(EN_DUART_MENU_ENTER);
                TIM_Reset_Timer2();
            }
        }
    }
    else if(EN_DUART_MENU_ENTER == DUART_GET_MENU_STATE())
    {
        HAL_UART_RxCpltCallback(duart);
        HAL_UART_IRQHandler(duart);
        TIM_Reset_Timer2();
    }

    if ( LPUART1->ISR & UART_FLAG_ORE)
    {
        __HAL_UART_CLEAR_IT(&hlpuart1, UART_CLEAR_OREF);
    }
}

/**
  * @brief Rx Transfer completed callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == (duart))
    {
        m_sCallBackFlag = 1;
    }
}

size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    HAL_UART_Transmit(duart, (uint8_t *)buf, 1, 10000);
    return 1;
}

size_t __read(int handle, unsigned char *buf, size_t bufSize)
{
    uint8_t ch;
    TIM_Reset_Timer2();

    HAL_UART_Receive_IT(duart,&ch,1);
    while(!m_sCallBackFlag)
    {
        FeedDog();
    }
    m_sCallBackFlag = 0;
    *buf = ch;
    Tim2Disable();
    return (int)1;
}

int putchar(int c)
{
    /* put character to stdout */
    unsigned char uc = c;
    if (__write(_LLIO_STDOUT, &uc, 1) == 1)
    {
        return uc;
    }

    return EOF;
}

void DuartEnableIt(void)
{
    __HAL_UART_ENABLE_IT(duart, UART_IT_RXNE);
}

void DuartDisableIt(void)
{
    __HAL_UART_DISABLE_IT(duart, UART_IT_RXNE);
}
