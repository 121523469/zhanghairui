#include <stdint.h>
#include "Flash.h"
#include "SrvErrorNo.h"
#include "SrvIODeviceItem.h"


StSrvIODevice_t stFlash;
/**************************************************************************************************
 * static function prototypes
 *************************************************************************************************/
static int16_t flash_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t flash_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t flash_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t flash_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t flash_seek(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset);
static int16_t flash_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg);


static void flash_readdwrod(uint32_t p_u32Address, uint32_t *p_pu32Parameter, uint16_t p_u16ParaLength);
static void flash_writedword(uint32_t p_u32Address, uint32_t *p_pvParameter, uint16_t p_u16ParaLength);
static void EraseFlash(uint32_t p_u32Address,uint16_t Page);

/*
 * static variables
 */
static StSrvIODeviceItem_t m_stFlashDeviceItem;            
static StSrvIOOperations_t m_stFlashOperation = 
{
    .open = flash_open,
    .close = flash_close,
    .read = flash_read,
    .write = flash_write,
    .seek = flash_seek,
    .ioctl = flash_ioctl,
};

void Flash_configure(void)
{
    SrvIODeviceItem_register(&m_stFlashDeviceItem, EN_ID_FLASH , &m_stFlashOperation, IO_READ | IO_WRITE);
}

static int16_t flash_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    p_pstSrvIODevice->pstDeviceItem->u8Count++;

    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 1)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
        return ERR_BEYOND_MAX;
    }

    if (EN_ID_FLASH == p_pstSrvIODeviceItem->enDeviceId)
    {
        p_pstSrvIODevice->pvPrivateData = (void *)0;
    }
    return ERR_OK;

}
static int16_t flash_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 0)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
    }
    p_pstSrvIODevice->pstDeviceItem = NULL;

    return ERR_OK;
}
static int16_t flash_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    if(p_pstSrvIODevice == NULL)
    {
        return ERR_FAULT;
    }
    if(p_pvBuffer == NULL)
    { 
        return ERR_FAULT;
    }
    
    flash_readdwrod(p_pstSrvIODevice->u32Position,p_pvBuffer,p_u16BufferSize);
    
    return ERR_OK;
}

static int16_t flash_seek(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset)
{
    if(p_pstSrvIODevice == NULL)
    {
        return ERR_FAULT;
    }
    if(BACKUP_ADDR_START + p_u32Offset > BACKUP_ADDR_END)
    {
        return ERR_OUT_OF_RANGE;
    }
    p_pstSrvIODevice->u32Position = BACKUP_ADDR_START + p_u32Offset;
    
    return ERR_OK;
}

static int16_t flash_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    if(p_pstSrvIODevice == NULL)
    {
        return ERR_FAULT;
    }
    if(p_pvBuffer == NULL)
    {
        return ERR_FAULT;
    }
    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= BACKUP_ADDR_END)
    {
        return ERR_OUT_OF_RANGE;
    }

    /* 写入数据  */
    flash_writedword(p_pstSrvIODevice->u32Position,p_pvBuffer,p_u16BufferSize);
    return ERR_OK;
}

static int16_t flash_ioctl(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    switch(p_u32Command)
    {
        case EN_FLASH_CMD_ERASE:
        {
            EraseFlash(p_pstSrvIODevice->u32Position,p_u32Arg);
        }
        default:break;
    }
    return 0;
}

static void flash_readdwrod(uint32_t p_u32Address, uint32_t *p_pu32Parameter, uint16_t p_u16ParaLength)
{
    uint16_t i =0;
    
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    for(i = 0; i< p_u16ParaLength; i++)
    {
        p_pu32Parameter[i++] = *(uint32_t *)p_u32Address;
    }
    /* Locks the Data memory */
    HAL_FLASH_Lock();
}

static void flash_writedword(uint32_t p_u32Address, uint32_t *p_pvParameter, uint16_t p_u16ParaLength)
{
    uint16_t i =0;
    
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    for(i = 0; i< p_u16ParaLength; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,p_u32Address,*p_pvParameter) == HAL_OK)
        {
            p_u32Address += 4;
            p_pvParameter++;
        }
    }
    HAL_FLASH_Lock();
}

static void EraseFlash(uint32_t p_u32Address,uint16_t Page)
{
    uint16_t i = 0;
    
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    for(i = 0; i < Page; i++)
    {
        FLASH_PageErase(p_u32Address + (i * FLASH_PAGE_SIZE));
        CLEAR_BIT(FLASH->PECR, FLASH_PECR_PROG);
    }
    HAL_FLASH_Lock();
}

void UpdateVTOR(uint32_t u32OffSet)
{
    SCB->VTOR = FLASH_BASE | u32OffSet;
}


