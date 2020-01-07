/*************************************************************************************************/
/** @file   SrvIODevice.h
    @brief  Header for SrvIODevice.h
    @company Chinastar
    @date 
    @author dingtieshu
    @version  1.0
*/
/*************************************************************************************************/
#ifndef _SRVIODEVICE_H_
#define _SRVIODEVICE_H_

/**************************************************************************************************
 * include files
 *************************************************************************************************/
#include <stdint.h>
#include "SrvList.h"

#define IO_READ  (1u)      /**< Flag for read */
#define IO_WRITE (2u)      /**< Flag for write */     
/**************************************************************************************************
 * type defines
 *************************************************************************************************/
/** Device Enum. List all available device IDs */
typedef enum
{
    EN_ID_MANUSTORAGE = 0,
    EN_ID_USERSTORAGE  = 1,
    EN_ID_GPRS = 2,
    EN_ID_ZIGBEE,
    EN_ID_NBIOT,
    EN_ID_LORAWAN,
    EN_ID_ADC,
    EN_ID_FLASH,
    EN_ID_DUART,
    EN_ID_QMC5883L,
    EN_ID_COUNT,
    EN_ID_NFC,
    EN_ID_LED,
    EN_ID_RS485,
}EnDeviceId_t;

typedef struct StSrvIOOperations StSrvIOOperations_t;

/** Device item struct */
typedef struct
{
    StSrvListLink_t stListLink;           /**< Link list. Must be the first element */
    EnDeviceId_t  enDeviceId;             /**< Device ID */
    StSrvIOOperations_t *pstIOOperations; /**< IO operations struct pointer */
    uint8_t u8Count;                      /**< Device count opened */
    uint16_t u16Flags;                    /**< Operation flags */
}StSrvIODeviceItem_t;
/** Device struct */
typedef struct 
{ 
    uint16_t u16Flags;                  /**< Open flags */
    void *pvPrivateData;                /**< Praviate data */
    uint32_t u32Position;               /**< Read write position */ 
    StSrvIODeviceItem_t *pstDeviceItem; /**< Pointer to Device Item */
}StSrvIODevice_t;
/** IO Operation struct */
struct StSrvIOOperations
{
    int16_t (* open)(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
    int16_t (* close)(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
    int16_t (* read)(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
    int16_t (* seek)(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset);
    int16_t (* write)(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
    int16_t (* ioctl)(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);
};
/**************************************************************************************************
 * global function declarations
 *************************************************************************************************/
extern int16_t SrvIODevice_open(StSrvIODevice_t* const me, EnDeviceId_t p_enDeviceId, uint16_t p_u16Flags);
extern int16_t SrvIODevice_close(StSrvIODevice_t* const me);
extern int16_t SrvIODevice_read(StSrvIODevice_t* const me, void *p_pvBuffer, uint16_t p_u16BufferSize);
extern int16_t SrvIODevice_write(StSrvIODevice_t* const me, void *p_pvBuffer, uint16_t p_u16BufferSize);
extern int16_t SrvIODevice_seek(StSrvIODevice_t* const me, uint32_t p_u32Offset);
extern int16_t SrvIODevice_ioctl(StSrvIODevice_t* const me, uint32_t p_u32Command, uint32_t p_u32Arg);

#endif //_SRVIODEVICE_H_

