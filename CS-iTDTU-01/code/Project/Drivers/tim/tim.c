/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"
#include "duart.h"
/* USER CODE BEGIN 0 */
#include "lptim.h"
#include "lorawan.h"
#include "led.h"
uint32_t Alarm_Frequency_Count = 0;
uint8_t Alarm_Frequency_Send = 0;
uint32_t SampleCycle_Counter = 0;
uint8_t Flag_ADC_Sample = 1;
uint8_t u8SecCount = 0;

Tim21ItCallback fp_Tim21ItCb = NULL;

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;
TIM_HandleTypeDef htim6;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 16000-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 60000-1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* TIM2 init function */
void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 800-1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 300-1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* TIM21 init function */
/*用于无线模块窗口管理*/
void MX_TIM21_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim21.Instance = TIM21;
    htim21.Init.Prescaler = 8000-1;
    htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim21.Init.Period = 25000-1;
    htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    //__HAL_TIM_CLEAR_IT(&htim21, TIM_IT_UPDATE);
    //HAL_TIM_Base_Start_IT(&htim21);
}
/* TIM22 init function */
void MX_TIM22_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 800-1;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 300-1;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 800-1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 10 - 1;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);

}

void Tim6Reset(void)
{
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
    /**<Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim6);

    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

    TIM6->CNT = 0;
    HAL_TIM_Base_Start_IT(&htim6);
}

void Tim6Disable(void)
{
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
    /**<Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim6);
    HAL_TIM_Base_MspDeInit(&htim6);
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM6_IRQHandler(void)
{
    /**< USER CODE BEGIN TIM6_IRQn 1 */
    if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) !=RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
            //ModuleDownLinkMsgProc();
            LedCtrlTimeoutCallback();
        }
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

    if(tim_baseHandle->Instance==TIM2)
    {
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if(tim_baseHandle->Instance==TIM21)
    {
        /* TIM21 clock enable */
        __HAL_RCC_TIM21_CLK_ENABLE();

        /* TIM21 interrupt Init */
        HAL_NVIC_SetPriority(TIM21_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM21_IRQn);
    }
    else if(tim_baseHandle->Instance==TIM22)
    {
        /* TIM22 clock enable */
        __HAL_RCC_TIM22_CLK_ENABLE();

        /* TIM22 interrupt Init */
        HAL_NVIC_SetPriority(TIM22_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM22_IRQn);
    }
    else if(tim_baseHandle->Instance==TIM6)
    {
        /* TIM6 clock enable */
        __HAL_RCC_TIM6_CLK_ENABLE();

        /* TIM6 interrupt Init */
        HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM6_IRQn);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM21)
  {
  /* USER CODE BEGIN TIM21_MspDeInit 0 */

  /* USER CODE END TIM21_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM21_CLK_DISABLE();

    /* TIM21 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM21_IRQn);
  /* USER CODE BEGIN TIM21_MspDeInit 1 */

  /* USER CODE END TIM21_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM22)
  {
  /* USER CODE BEGIN TIM22_MspDeInit 0 */

  /* USER CODE END TIM22_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM22_CLK_DISABLE();

    /* TIM22 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM22_IRQn);
  /* USER CODE BEGIN TIM22_MspDeInit 1 */

  /* USER CODE END TIM22_MspDeInit 1 */
  }
} 

void TIM2_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) !=RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

            if(EN_DUART_MENU_EXTI == DUART_GET_MENU_STATE())
            {
                DUART_SET_MENU_STATE(EN_DUART_MENU_IDLE);
                Tim2Disable();
            }
            else if(EN_DUART_MENU_ENTER == DUART_GET_MENU_STATE())
            {
               printf("Configuration mode timeout,system is rebooting...\r\n"); 
               NVIC_SystemReset();
            }
        }
    }

}

void Tim2Disable(void)
{
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    /**<Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim2);
}

void TIM_Reset_Timer2(void)
{
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    /**<Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim2);

    htim2.Instance->CNT = 0;
    HAL_TIM_Base_Start_IT(&htim2);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
}

#if 0
void TIM21_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim21, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim21, TIM_IT_UPDATE) !=RESET)
        {
            //__HAL_TIM_DISABLE_IT(&htim21, TIM_IT_UPDATE);
            /**<Disable the Peripheral */
            //__HAL_TIM_DISABLE(&htim21);

            __HAL_TIM_CLEAR_IT(&htim21, TIM_IT_UPDATE);

            //TIM21->CNT = 0;
            //HAL_TIM_Base_Start_IT(&htim21);
        }
    }
}
#endif

void Tim21Reset(void)
{
    __HAL_TIM_DISABLE_IT(&htim21, TIM_IT_UPDATE);
    /**<Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim21);

    __HAL_TIM_CLEAR_IT(&htim21, TIM_IT_UPDATE);

    TIM21->CNT = 0;
    HAL_TIM_Base_Start_IT(&htim21);
}

void Tim21Disable(void)
{
    __HAL_TIM_DISABLE_IT(&htim21, TIM_IT_UPDATE);
    /**<Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim21);
}

void Tim21RegisterCb(Tim21ItCallback fp_CbFun)
{
    fp_Tim21ItCb = fp_CbFun;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim21.Instance)    /* Timing Report */
    {
        if (NULL != fp_Tim21ItCb)
        {
            fp_Tim21ItCb();
        }
    }
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
