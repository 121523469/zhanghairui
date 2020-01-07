#ifndef _SRVIODEVICEITEM_H_
#define _SRVIODEVICEITEM_H_
/**************************************************************************************************
 * includes
 *************************************************************************************************/
#include <stdint.h>
#include "SrvIODevice.h"
/**************************************************************************************************
 * global function declarations
 *************************************************************************************************/
extern void SrvIODeviceItem_unregister(StSrvIODeviceItem_t* const me, EnDeviceId_t p_enDeviceId);
extern void SrvIODeviceItem_register(StSrvIODeviceItem_t* const me, EnDeviceId_t p_enDeviceId, StSrvIOOperations_t *p_pstIOOperations, uint16_t p_u16Flags);
extern StSrvIODeviceItem_t* SrvIODeviceItem_getDeviceItem(EnDeviceId_t p_enDeviceId);
#endif  //_SRVIODEVICEITEM_H_
