/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "string.h"
#include "stm32l0xx_ll_lpuart.h"
#include "product_config.h"
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

#define BUFFER_SIZE (500)   /**< Buffersize for Receive Buffer and message buffer */

//static uint8_t  m_u8lpusartRevBuffer[BUFFER_SIZE];
//static uint16_t  m_u16lpusartRevCount = 0;

static uint8_t m_u8usart1RevBuffer[BUFFER_SIZE];
static uint32_t  m_u16usart1RevCount = 0;

pfUartRecvFrameCB pfUart1RcvFrameCB = NULL;

/* LPUART1 init function */
void MX_LPUART1_UART_Init(void)
{
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 9600;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    //huart1.Init.BaudRate = 57600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
#ifdef UART_RXTX_SWAP_FEATURE
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
    huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
#endif
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if(uartHandle->Instance==LPUART1)
    {
        /* USER CODE BEGIN LPUART1_MspInit 0 */

        /* USER CODE END LPUART1_MspInit 0 */
        /* LPUART1 clock enable */
        __HAL_RCC_LPUART1_CLK_ENABLE();

        /**LPUART1 GPIO Configuration    
        PB10     ------> LPUART1_TX
        PB11     ------> LPUART1_RX 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* LPUART1 interrupt Init */
        /* USER CODE BEGIN LPUART1_MspInit 1 */
        LL_LPUART_SetWKUPType(LPUART1, LL_LPUART_WAKEUP_ON_RXNE | LL_LPUART_WAKEUP_ON_STARTBIT);
        LL_LPUART_EnableIT_WKUP(LPUART1);
        LL_LPUART_EnableInStopMode(LPUART1);
        HAL_NVIC_SetPriority(AES_RNG_LPUART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(AES_RNG_LPUART1_IRQn);

        /* USER CODE END LPUART1_MspInit 1 */
    }
    else if(uartHandle->Instance==USART2)
    {
        /* USER CODE BEGIN USART2_MspInit 0 */

        /* USER CODE END USART2_MspInit 0 */
        /* USART2 clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        /**USART2 GPIO Configuration    
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        /* USER CODE BEGIN USART2_MspInit 1 */

        /* USER CODE END USART2_MspInit 1 */
    }
    else if(uartHandle->Instance==USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else
    {
        /*do nothing*/
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
    if(uartHandle->Instance==LPUART1)
    {
        /* USER CODE BEGIN LPUART1_MspDeInit 0 */

        /* USER CODE END LPUART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_LPUART1_CLK_DISABLE();

        /**LPUART1 GPIO Configuration    
        PB10     ------> LPUART1_TX
        PB11     ------> LPUART1_RX 
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

        /* LPUART1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(AES_RNG_LPUART1_IRQn);
        /* USER CODE BEGIN LPUART1_MspDeInit 1 */

        /* USER CODE END LPUART1_MspDeInit 1 */
    }
    else if(uartHandle->Instance==USART2)
    {
        /* USER CODE BEGIN USART2_MspDeInit 0 */

        /* USER CODE END USART2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration    
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX 
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

        /* USART2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
        /* USER CODE BEGIN USART2_MspDeInit 1 */

        /* USER CODE END USART2_MspDeInit 1 */
    }
    else if(uartHandle->Instance==USART1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

        /* USART2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
} 

void USART1_IRQHandler(void)
{
    while( USART1->ISR & (UART_FLAG_RXNE))
    {
        if(m_u16usart1RevCount < BUFFER_SIZE)
        {
            m_u8usart1RevBuffer[m_u16usart1RevCount++] =(uint8_t)READ_REG(huart1.Instance->RDR);
            __HAL_TIM_DISABLE_IT(&htim22, TIM_IT_UPDATE);
            /**<Disable the Peripheral */
            __HAL_TIM_DISABLE(&htim22);

            __HAL_TIM_CLEAR_IT(&htim22, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim22);
        }
        else
        {
            m_u16usart1RevCount = 0;
        }
    }

    if ( USART1->ISR & UART_FLAG_ORE)
    {
        __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_OREF);
    }

    TIM22->CNT = 0;
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM22_IRQHandler(void)
{
    /**< USER CODE BEGIN TIM22_IRQn 1 */
    if(__HAL_TIM_GET_FLAG(&htim22, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim22, TIM_IT_UPDATE) !=RESET)
        {
            __HAL_TIM_DISABLE(&htim22);
            __HAL_TIM_DISABLE_IT(&htim22, TIM_IT_UPDATE);
            /*disable muart irq*/
            //if(!((1 == m_u16MusartRevCount) && ('\r' == m_u8MusartRevBuffer[0])))
            {
                if(NULL != pfUart1RcvFrameCB)
                {
                    /*Copy uart buffer to NBIOT buffer*/
                    pfUart1RcvFrameCB(m_u8usart1RevBuffer, (void *)&m_u16usart1RevCount);
                }
            }
            //printf("%s",m_u8usart1RevBuffer);
            memset(m_u8usart1RevBuffer, 0, sizeof(m_u8usart1RevBuffer));
            m_u16usart1RevCount = 0;
            /*enable muart irq*/
            
            /**<Disable the Peripheral */
            __HAL_TIM_CLEAR_IT(&htim22, TIM_IT_UPDATE);
            __HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER1);
        }
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
