#ifndef __DUART_H
#define __DUART_H
#include "usart.h"
#include "SrvIODevice.h"

/**< ioctl控制命令 */
typedef enum
{
    EN_CMD_GET_DUART_MENU_STATE,/*Get Duart menu state*/
    EN_CMD_EXIT_MENU,
    //EN_CMD_REGRECVCALLBACK,      /**< 注册回调函数 */
    EN_CMD_REGSERCALLBACK,
    EN_CMD_SETSLEEPMODULE,       /**< 睡眠模式 */
    EN_CMD_SETMANUCONFIGMODULE,  /**< 出厂配置模式 */
    EN_CMD_SERIALTRANSMODULE,    /**< 透传模式 */
    EN_CMD_PA3EXTIMODE,          /**< PA3切换为外部中断模式 */
    EN_CMD_PA3UARTRXMODE,        /**< PA3切换为UART2 RX 模式 */
}EnModuleCmd_t;

typedef enum
{
    EN_DCB_ONRECEIVE,
    EN_DCB_ONTRANSCOMPLETE
}EnDUartCallBack_t;

typedef enum
{
	  EN_DCB_USART2,
}EnDUsartType_t;

typedef struct
{
    UART_HandleTypeDef *p_sthuart;
    USART_TypeDef   *p_stinstance;
    uint32_t baudrate;
    uint32_t wordlength;
    uint32_t stopbits;
    uint32_t parity;
}StDUsartPara_t;


typedef enum 
{
    EN_ERR_DUART_OK,
    EN_ERR_DUART_NULL,
    EN_ERR_DUART_TIMOUT,
    EN_ERR_DUART_PARAM
}EnDUartErrCode_t;


typedef enum
{
    EN_DUART_MENU_IDLE,
    EN_DUART_MENU_EXTI,
    EN_DUART_MENU_ENTER,
}EnMenuState_t;

extern EnMenuState_t DuartMenuState;


#define DUART_GET_MENU_STATE()      DuartMenuState
#define DUART_SET_MENU_STATE(state) do \
{\
    DuartMenuState = (state);\
}while(0)

extern volatile uint8_t u8Putchar_Timeout_flag;
extern void duart_configure(void);
extern void setPA3EXTImode(void);

extern StSrvIODevice_t stDebugUartDevice;

extern int8_t DUART_init(StDUsartPara_t p_stusart);
extern void DuartEnableIt(void);
extern void DuartDisableIt(void);
extern int8_t DUART_send(UART_HandleTypeDef *p_psthuart,uint8_t *p_pu8Buffer, uint16_t p_u16Length);
extern int8_t DUART_regcallback(EnDUartCallBack_t p_enCallbackType,EnDUsartType_t p_enUsartType, void (*p_pf)(void *p_pvParam));
extern uint8_t *DUART_getusart2recvbuffer(void);
extern StDUsartPara_t *DUART_getusartparapt(void);
#endif
