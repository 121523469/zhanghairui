#ifndef __MUSART_H
#define __MUSART_H

#include "usart.h"
#include "gpio.h"

/**<callback fun struction def*/
typedef enum
{
    EN_CB_ONRECEIVE,/**<callback of received one frame*/
    EN_CB_ONTRANSCOMPLETE/**<callback of transmit finished*/
}EnCallBack_t;

typedef struct _StUsartCallBack_
{
    void (*pfRecvCB)(void *p_pvParamSrc, void *p_pvParamLen);
    void (*pfTCCB)(void *p_pvParam);
}StUsartCallBack;

typedef struct
{
    UART_HandleTypeDef *p_sthuart;/**<uart struction eg:huart2*/
    USART_TypeDef   *p_stinstance;
    StUsartCallBack stMuartCB;
}StUsartPara_t;

/**<Err list*/
typedef enum {
    EN_ERR_OK,        /**<err none*/
    EN_ERR_NULL,      /**<Null pointer*/
    EN_ERR_TIMOUT, /**<timeout err*/
    EN_ERR_PARAM      /**<para err*/
}EnUartErrCode_t;

extern int8_t MuartInit(void);
extern void MuartDeinit(void);

extern int8_t MuartRevdFrameProcRegister(EnCallBack_t p_enCallbackType, pfUartRecvFrameCB p_pf);
extern int8_t MuartSend(uint8_t *p_pu8Buffer, uint32_t p_u32Length);

#endif

