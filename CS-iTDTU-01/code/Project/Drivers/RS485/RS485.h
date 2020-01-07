#ifndef _RS485_H_
#define _RS485_H_

#include "SrvIODevice.h"

#pragma pack(1)
typedef struct
{
    uint8_t u8SlaveAddr;
    uint8_t u8Function;
    uint16_t u16RegAddr;
    uint16_t u16RegCount;
    uint16_t u16CRC;
}ModbusReadReq_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8_t u8SlaveAddr;
    uint8_t u8Function;
    uint8_t u8ByteCount;
    uint8_t *Value;
    uint16_t u16CRC;
}ModbusReadRes_t;
#pragma pack()

typedef enum
{
    RS485_ERR_OK = 0,
    RS485_ERR_INVALIDPARA,
    RS485_ERR_NULL,
    RS485_ERR_BEYOND,
}EnRS485Err_t;

typedef enum
{
    RS485_UART_DISABLE = 0,
    RS485_UART_ENABLE,
}EnRS485UartStatus_t;

typedef enum
{
    RS485_TX = 0,
    RS485_RX = 1,
}EnRS485TRStatus_t;

typedef enum
{
    RS485_CHAGNE_BAUDRATE,
}EnRS485IOCtlCMD_t;

typedef struct
{
    uint32_t u32BaudRate;
    volatile uint8_t u8RcvFlag;
    EnRS485UartStatus_t EnUartStatus;
    volatile EnRS485TRStatus_t EnTRStatus;
    volatile uint32_t u32RcvLen;
}RS485Paras_t;

extern void RS485Configure();
#endif