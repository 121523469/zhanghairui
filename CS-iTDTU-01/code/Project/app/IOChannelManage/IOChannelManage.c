#include "gpio.h"
#include <string.h>
#include "ADCDriver.h"
#include "SrvErrorNo.h"
#include "IOChannelManage.h"

enum
{
    EVENT_DI1_IRQ = 0,
    EVENT_DI2_IRQ,
    EVENT_MAX,
};

static IOChannel_t IOChannelHead;

#define true    (1)
#define false   (0)

#define ANALOGCH_POWERON()    do{HAL_GPIO_WritePin(AIN_CH_POWER_GPIO,AIN_CH_POWER_PIN,GPIO_PIN_RESET);}while(0)
#define ANALOGCH_POWEROFF()   do{HAL_GPIO_WritePin(AIN_CH_POWER_GPIO,AIN_CH_POWER_PIN,GPIO_PIN_SET);}while(0)

#define SUPPLYPOWER_POWERON()    do{HAL_GPIO_WritePin(SUPPLY_POWER_GPIO_PORT,SUPPLY_POWER_GPIO_PIN,GPIO_PIN_SET);}while(0)
#define SUPPLYPOWER_POWEROFF()   do{HAL_GPIO_WritePin(SUPPLY_POWER_GPIO_PORT,SUPPLY_POWER_GPIO_PIN,GPIO_PIN_RESET);}while(0)


#define DO_CHANNEL_SET()        do{HAL_GPIO_WritePin(DO_CH1_GPIO_PORT,DO_CH1_GPIO_PIN,GPIO_PIN_RESET);}while(0)
#define DO_CHANNEL_RESET()      do{HAL_GPIO_WritePin(DO_CH1_GPIO_PORT,DO_CH1_GPIO_PIN,GPIO_PIN_SET);}while(0)



IOChannel_t stAnalogInputCH1;
IOChannel_t stAnalogInputCH2;
IOChannel_t stDigitalInputCH1;
IOChannel_t stDigitalInputCH2;
IOChannel_t stDigitalOutputCH1;
IOChannel_t stRS485CH1;
IOChannel_t stSupplyPowerOutPut;


static volatile uint8_t EventIrqFlag[EVENT_MAX] = {0};

static int16_t AnalogCHCollector(EnChannelType_t p_enChannelType,uint8_t *p_u8Buff,uint8_t p_u8Size);
static int16_t DigitalCHCollector(EnChannelType_t p_enChannelType,uint8_t *p_u8Buff,uint8_t p_u8Size);
static void AnalogInputTypeInit(void);

StIOChannelPara stIOChannelPara = 
{
    .u16ChannelStatus = 0,
    .u8AnalogType = 0,
};

static void IOChannelListInit(IOChannel_t *const me)
{
    if(NULL == me)
    {
        return;
    }
    me->next = NULL;
}

static uint8_t IOChannelExist(IOChannel_t *const obj)
{
    IOChannel_t *cur = (&IOChannelHead)->next;

    while(NULL != cur)
    {
        if(cur->enChannelType == obj->enChannelType)
        {
            return true;
        }
        cur = cur->next;
    }
    
    return false;
}

static void IOChannelListInsert(IOChannel_t *const obj)
{
    IOChannel_t *head = &IOChannelHead;
    IOChannel_t *cur = head->next;

    if((NULL == obj) || ( IOChannelExist(obj) == true ))
    {
        return;
    }

    if(NULL == cur)
    {
        head->next = obj;
        obj->next = NULL;
    }
    else
    {
        while(NULL != cur->next)
        {
            cur = cur->next;
        }
        cur->next = obj;
        obj->next = NULL;
    }
}

static void IOChannelRegister(IOChannel_t *const obj,EnChannelType_t enChannelType,EnChannelState_t EnChannelState,Collector fpCollector)
{
    static uint8_t First = 0;

    if(0 == First)
    {
        First = 1;
        IOChannelListInit(&IOChannelHead);
    }
    
    if(NULL == obj)
    {
        return;
    }
    
    obj->EnChannelState = EnChannelState;
    obj->enChannelType = enChannelType;
    obj->fpCollector = fpCollector;
    obj->next = NULL;

    if(CHANNEL_DISABLE == EnChannelState)
    {
        stIOChannelPara.u16ChannelStatus &= ~(1 << enChannelType);
    }
    else if(CHANNEL_ENABLE == EnChannelState)
    {
        stIOChannelPara.u16ChannelStatus |= 1 << enChannelType;
    }
    else
    {
        /**/
    }
    IOChannelListInsert(obj);
}

static void AnalogChannel1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = AIN_CH1_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AIN_CH1_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = AICH1_TYPE_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AICH1_TYPE_GPIO_PORT, &GPIO_InitStruct);

    IOChannelRegister(&stAnalogInputCH1,EN_TYPE_AI1,CHANNEL_ENABLE,AnalogCHCollector);
}

static void AnalogChannel2Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = AIN_CH2_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AIN_CH2_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = AICH2_TYPE_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AICH2_TYPE_GPIO_PORT, &GPIO_InitStruct);
    
    IOChannelRegister(&stAnalogInputCH2,EN_TYPE_AI2,CHANNEL_ENABLE,AnalogCHCollector);
}


static void DigitalInputCh1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = DIN_CH1_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIN_CH1_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
    
    IOChannelRegister(&stDigitalInputCH1,EN_TYPE_DI1,CHANNEL_ENABLE,DigitalCHCollector);
}

static void DigitalInputCh2Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = DIN_CH2_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIN_CH2_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
    
    IOChannelRegister(&stDigitalInputCH2,EN_TYPE_DI2,CHANNEL_ENABLE,DigitalCHCollector);
}

static void DigitalOutPutCh1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = DO_CH1_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DO_CH1_GPIO_PORT, &GPIO_InitStruct);
    
    DO_CHANNEL_SET();
    IOChannelRegister(&stDigitalOutputCH1,EN_TYPE_DO1,CHANNEL_ENABLE,NULL);
}

static void RS485ChannelInit(void)
{
    IOChannelRegister(&stRS485CH1,EN_TYPE_RS485,CHANNEL_ENABLE,NULL);
}

static void AnalogCHPowerCtrlInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = AIN_CH_POWER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AIN_CH_POWER_GPIO, &GPIO_InitStruct);
    
    ANALOGCH_POWEROFF();
}

static void SupplyPowerInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = SUPPLY_POWER_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SUPPLY_POWER_GPIO_PORT, &GPIO_InitStruct);

    SUPPLYPOWER_POWEROFF();
    
    IOChannelRegister(&stSupplyPowerOutPut,EN_TYPE_SUPPLY,CHANNEL_ENABLE,NULL);
}

static int16_t AnalogCHCollector(EnChannelType_t p_enChannelType,uint8_t *p_u8Buff,uint8_t p_u8Size)
{
    int16_t Ret = -1;
    uint32_t u32AICHSample = 0;
    uint16_t u16AICHActual = 0;
    
    if(NULL == p_u8Buff)
    {
        return ERR_NODEV;
    }

    Ret = SrvIODevice_open(&stADC, EN_ID_ADC, IO_READ | IO_WRITE);
    if ((ERR_OK != Ret) && (ERR_BEYOND_MAX != Ret))
    {
        printf("SrvIODevice_open open stADC fail,ret=%d!\r\n", Ret);
        return Ret;
    }
    SUPPLYPOWER_POWERON();
    ANALOGCH_POWERON();
    delay_ms(20);
    switch(p_enChannelType)
    {
        case EN_TYPE_AI1:
        {
            Ret = SrvIODevice_ioctl(&stADC, EN_ADC_CHN_VOL_CH1, (uint32_t)(&u32AICHSample));
            if(ERR_OK == Ret)
            {
                printf("Analog input channel 1 voltage %dmV\r\n",u32AICHSample);
                if((EN_AI_TYPE_VOL == stIOChannelPara.u8AnalogType & (1<< EN_TYPE_AI1)))
                {
                    u16AICHActual = (uint16_t)(u32AICHSample * 162 /62/ 10);
                }
                else
                {
                    u16AICHActual = (uint16_t)(u32AICHSample * 162 /62/ 10) / 220;
                }
            }
            else
            {
                printf("AnalogCHCollector fail,channel is %d\r\n",EN_TYPE_AI1);
                return ERR_FAULT;
            }
            break;
        }
        case EN_TYPE_AI2:
        {
            Ret = SrvIODevice_ioctl(&stADC, EN_ADC_CHN_VOL_CH2, (uint32_t)(&u32AICHSample));
            if(ERR_OK == Ret)
            {
                printf("Analog input channel 2 voltage %dmV\r\n",u32AICHSample);
                if((EN_AI_TYPE_VOL == stIOChannelPara.u8AnalogType & (1<< EN_TYPE_AI2)))
                {
                    u16AICHActual = (uint16_t)(u32AICHSample * 162 /62/ 10);
                }
                else
                {
                    u16AICHActual = (uint16_t)(u32AICHSample * 162 /62/ 10) / 22;
                }
            }
            else
            {
                printf("AnalogCHCollector fail,channel is %d\r\n",EN_TYPE_AI2);
                return ERR_FAULT;
            }
            break;
        }
        default:
        {
            printf("p_enChannelType invalid,p_enChannelType is %d\r\n",p_enChannelType);
            break;
        }
    }
    
    memmove(p_u8Buff,&u16AICHActual,p_u8Size);
    ANALOGCH_POWEROFF();
    SUPPLYPOWER_POWEROFF();
    
    return Ret;
}

static int16_t DigitalCHCollector(EnChannelType_t p_enChannelType,uint8_t *p_u8Buff,uint8_t p_u8Size)
{
    if(NULL == p_u8Buff)
    {
        return ERR_NODEV;
    }

    if(EN_TYPE_DI1 == p_enChannelType)
    {
        *p_u8Buff = HAL_GPIO_ReadPin(DIN_CH1_GPIO_PORT,DIN_CH1_GPIO_PIN);
    }
    else if(EN_TYPE_DI2 == p_enChannelType)
    {
        *p_u8Buff = HAL_GPIO_ReadPin(DIN_CH2_GPIO_PORT,DIN_CH2_GPIO_PIN);
    }
    else
    {
        *p_u8Buff = 0;
        printf("p_enChannelType invalid,p_enChannelType is %d\r\n",p_enChannelType);
    }
    
    return ERR_OK;
}


static void AnalogInputTypeInit(void)
{
    ANALOGCH_POWERON();
    delay_ms(200);
    
    if(GPIO_PIN_SET == HAL_GPIO_ReadPin(AICH1_TYPE_GPIO_PORT,AICH1_TYPE_GPIO_PIN))
    {
        stIOChannelPara.u8AnalogType &= ~(1 << EN_TYPE_AI1);
    }
    else
    {
        stIOChannelPara.u8AnalogType |= (1 << EN_TYPE_AI1);
    }

    if(GPIO_PIN_SET == HAL_GPIO_ReadPin(AICH2_TYPE_GPIO_PORT,AICH2_TYPE_GPIO_PIN))
    {
        stIOChannelPara.u8AnalogType &= ~(1 << EN_TYPE_AI2);
    }
    else
    {
        stIOChannelPara.u8AnalogType |= (1 << EN_TYPE_AI2);
    }
    
    ANALOGCH_POWEROFF();
}

uint8_t GetAnalogType(void)
{
    return stIOChannelPara.u8AnalogType;
}


void IOChannelInitialize(void)
{
    AnalogChannel1Init();
    AnalogChannel2Init();
    AnalogCHPowerCtrlInit();
    DigitalInputCh1Init();
    DigitalInputCh2Init();
    DigitalOutPutCh1Init();
    RS485ChannelInit();
    SupplyPowerInit();
    AnalogInputTypeInit();
}

int8_t IOChannelCollector(IOChannel_t* const obj, EnChannelType_t p_enChannelType, uint8_t *p_u8Buff,uint8_t p_u8Size)
{
    int8_t Ret = ERR_OK;
    
    if(true != IOChannelExist(obj))
    {
        return ERR_NODEV;
    }
    
    if(obj->fpCollector == (void *)0)
    {
        return ERR_NODEV;
    }

    if(CHANNEL_ENABLE == obj->EnChannelState)
    {
        Ret = obj->fpCollector(p_enChannelType,p_u8Buff,p_u8Size);
    }
    
    return ERR_OK;
}


void SetDIIrqFlag(EnChannelType_t enChannelType,uint8_t u8Status)
{
    if(EN_TYPE_DI1 == enChannelType)
    {
        EventIrqFlag[EVENT_DI1_IRQ] = u8Status;
    }
    else if(EN_TYPE_DI2 == enChannelType)
    {
        EventIrqFlag[EVENT_DI2_IRQ] = u8Status;
    }
    else
    {
        printf("Invalid digital input enChannelType!\r\n");
    }
}

uint8_t GetDIIrqFlag(EnChannelType_t enChannelType)
{
    if(EN_TYPE_DI1 == enChannelType)
    {
        return EventIrqFlag[EVENT_DI1_IRQ];
    }
    else if(EN_TYPE_DI2 == enChannelType)
    {
        return EventIrqFlag[EVENT_DI2_IRQ];
    }
    else
    {
        printf("Invalid digital input channel!\r\n");
    }
    
    return 0;
}

void DOChannelOutput(EnDOChannelStatus EnDoStatus)
{
    if(EN_DO_SET == EnDoStatus)
    {
        DO_CHANNEL_SET();
    }
    else
    {
        DO_CHANNEL_RESET();
    }
}

uint8_t GetDIChannelLevel(EnChannelType_t enChannelType)
{
    if(EN_TYPE_DI1 == enChannelType)
    {
        return HAL_GPIO_ReadPin(DIN_CH1_GPIO_PORT,DIN_CH1_GPIO_PIN);
    }
    else if(EN_TYPE_DI2 == enChannelType)
    {
        return HAL_GPIO_ReadPin(DIN_CH2_GPIO_PORT,DIN_CH2_GPIO_PIN);
    }
    else
    {
        printf("Invalid digital input channel!\r\n");
    }
    return 0;
}

const IOChannels_t IOChannel = 
{
    .Init = IOChannelInitialize,
    .Collector = IOChannelCollector,
    .SetDIIrqFlag = SetDIIrqFlag,
    .GetDIIrqStatus = GetDIIrqFlag,
    .GetAnalogType = GetAnalogType,
    .DOChannelOutput = DOChannelOutput,
    .GetDIChannelLevel = GetDIChannelLevel,
};

