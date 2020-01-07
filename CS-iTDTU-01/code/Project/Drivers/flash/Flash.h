#ifndef _FLASH_H_
#define _FLASH_H_
#include "SrvIODevice.h"
#include "stm32l0xx_hal.h"


#define APP_ADDR_START      (0x08008000)
#define APP_ADDR_END        (0x0801BFFF)

#define BACKUP_ADDR_START   (0x0801C000)
#define BACKUP_ADDR_END     (0x0802FFFF)

#define SHARE_INFO_SIZE     (FLASH_PAGE_SIZE)

#define BACKUP_PAGE         (((BACKUP_ADDR_END - BACKUP_ADDR_START) + 1) / 128)

#define VTOR_OFFSET         0x8000


typedef enum
{
    EN_FLASH_CMD_ERASE,
}EnFlashIoctlCmd;

extern void Flash_configure(void);
extern void UpdateVTOR(uint32_t u32OffSet);

extern StSrvIODevice_t stFlash;
#endif
