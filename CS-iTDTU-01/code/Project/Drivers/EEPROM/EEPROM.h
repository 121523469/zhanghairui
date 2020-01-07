#ifndef _EEPROM_H_
#define _EEPROM_H_
#include "SrvIODevice.h"
/** EEPROM错误码 */
typedef enum
{
    EN_EEPROM_ERR_OK = 0,
    EN_EEPROM_ERR_WRITE = 1
}EnEEPROMStatus_t;

extern StSrvIODevice_t stEEPROMUserDevice;
extern StSrvIODevice_t stEEPROMManuDevice;
extern void EEPROM_configure(void);

#endif
