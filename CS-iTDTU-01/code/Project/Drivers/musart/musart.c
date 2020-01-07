/**
******************************************************************************
* File Name          : MUSART.c
* Description        : This file provides code for the configuration
*                      of the USART instances.
******************************************************************************/
#include "usart.h"
#include "gpio.h"
//#include "tim.h"
#include "musart.h"
#include "string.h"
#include "stm32l0xx_hal_tim.h"

/**************************************************************************************************
 * Private define   USART1用到TIM2   USART2用到TIM21    LPUART1用到TIM22
 * 串口中断每次接收一个字节开启对应的定时器，当定时器中断到来的时候说明在定时的时间内没有接收到下一个字节
 * 此时认为一帧数据接受完成
 *************************************************************************************************/
typedef struct
{
    UART_HandleTypeDef *p_stMusart;/**<  UART handle Structure poiter */
}StUsart_t;

static StUsartPara_t m_stMsuartPara =
{
    .p_sthuart = &huart1,
};

/**
* @brief  USART init function.
*@param p_stusart  USART PARA
*/
/* */
int8_t MuartInit(void)
{
    MX_USART1_UART_Init();
    return EN_ERR_OK;
}

void MuartDeinit(void)
{
    __HAL_UART_DISABLE(&huart1);
    __USART1_CLK_DISABLE();
    HAL_UART_DeInit(&huart1);
}


/**
* @brief This function This function UART send.
*@param eg:huart1 
*@param p_pu8Buffer send  buffer pointer
*@param p_u16Length send data len
*/
int8_t MuartSend(uint8_t *p_pu8Buffer, uint32_t p_u32Length)
{
    if(p_pu8Buffer == NULL)
    {
        return EN_ERR_NULL;
    }

    HAL_UART_Transmit(m_stMsuartPara.p_sthuart, p_pu8Buffer, p_u32Length, 10000);
    return EN_ERR_OK;
}

/**
* @brief This function This function registering callback.
*@param p_enCallbackType  callback type
*@param eg: datalenth
*/
int8_t MuartRevdFrameProcRegister(EnCallBack_t p_enCallbackType, pfUartRecvFrameCB p_pf)
{
    if(p_pf == NULL)
    {
        printf("MUART_regcallback p_pf is NULL!\r\n");
        return EN_ERR_NULL;
    }

    switch (p_enCallbackType)
    {
        case EN_CB_ONRECEIVE:/**Rcv CallBack poiter*/
            pfUart1RcvFrameCB = p_pf;
            return EN_ERR_OK;

        case EN_CB_ONTRANSCOMPLETE:/**Send CallBack poiter*/
            //m_stMsuartPara.stMuartCB.pfTCCB = p_pf;
            return EN_ERR_OK;

        default:
            printf("MUART_regcallback p_enCallbackType is invalid,%d!\r\n", p_enCallbackType);
            return EN_ERR_PARAM;
    }
 //   return EN_ERR_PARAM;
}

