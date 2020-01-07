#include "stm32l0xx_hal.h"
#include "SrvErrorNo.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"
#include <string.h>
#include "EEPROM.h"

#define EEPROM_USER_START_ADDR      0x08080000        /**< 保存用户配置参数起始地址 */
#define EEPROM_USER_END_ADDR        0x080801FF        /**< 保存用户配置参数结束地址 */

#define EEPROM_MANU_START_ADDR      0x08080400         /**< 保存生产参数起始地址 */
#define EEPROM_MANU_END_ADDR        0x080807F0         /**< 保存生产参数结束地址 */

StSrvIODevice_t stEEPROMUserDevice;
StSrvIODevice_t stEEPROMManuDevice;
/**************************************************************************************************
 * static function prototypes
 *************************************************************************************************/
static int16_t eeprom_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t eeprom_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice);
static int16_t eeprom_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t eeprom_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize);
static int16_t eeprom_seek(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset);
static uint32_t eeprom_writeword(uint32_t p_u32Address, uint32_t *p_pvParameter, uint8_t p_u8ParaLength);

/**************************************************************************************************
 * static variables
 *************************************************************************************************/
static StSrvIODeviceItem_t m_stManuStorageDeviceItem;            /**< 生产参数存储设备 */
static StSrvIODeviceItem_t m_stUserStorageDeviceItem;            /**< 用户参数存储设备 */
static StSrvIOOperations_t m_stFlashOperation = 
{
    .open = eeprom_open,                                         /**< 从链表中找到EEPROM设备 */
    .close = eeprom_close,                                       /**< 关闭已找到的EEPROM设备 */
    .read = eeprom_read,                                         /**< 从EEPROM中读取数据 */
    .write = eeprom_write,                                       /**< 写数据到EEPROM */
    .seek = eeprom_seek,                                         /**< 写入数据地址 */
};

/* EEPROM地址 */
typedef struct {
uint32_t u32startaddress;                                        /**< EEPROM起始地址 */
uint32_t u32endaddress;                                          /**< EEPROM结束地址 */
}StAddressMap_t;

/* 生产参数保存地址 */
static StAddressMap_t m_stManuSection = 
{
  .u32startaddress = EEPROM_MANU_START_ADDR,                     /**< 保存生产参数起始地址 */
  .u32endaddress = EEPROM_MANU_END_ADDR                          /**< 保存生产参数结束地址 */
};

/* 用户参数保存地址 */
static StAddressMap_t m_stUserSection = 
{
  .u32startaddress = EEPROM_USER_START_ADDR,                     /**< 保存用户参数起始地址 */
  .u32endaddress = EEPROM_USER_END_ADDR                          /**< 保存用户参数结束地址 */
};

/*************************************************************************************************/
/** @brief 注册EEPROM设备
*/
/*************************************************************************************************/
void EEPROM_configure(void)
{
    /* 注册生产参数存储Device */
    SrvIODeviceItem_register(&m_stManuStorageDeviceItem, EN_ID_MANUSTORAGE , &m_stFlashOperation, IO_READ | IO_WRITE);
    /* 注册用户参数存储Device */
    SrvIODeviceItem_register(&m_stUserStorageDeviceItem, EN_ID_USERSTORAGE, &m_stFlashOperation, IO_READ | IO_WRITE);
}

/*************************************************************************************************/
/** @brief 打开EEPROM设备

    @param[in] p_pstSrvIODeviceItem Device item
    @param[in] p_pstSrvIODevice     Device

    @retval _SRVERRORNO_H_
*/
/*************************************************************************************************/
static int16_t eeprom_open(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    p_pstSrvIODevice->pvPrivateData = (void *)0;
    p_pstSrvIODevice->pstDeviceItem->u8Count++;
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 1)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
        return ERR_BEYOND_MAX;
    }
    if (p_pstSrvIODeviceItem->enDeviceId == EN_ID_MANUSTORAGE)
    {
       p_pstSrvIODevice->pvPrivateData = (void *)&m_stManuSection;
    }
    
    if (p_pstSrvIODeviceItem->enDeviceId == EN_ID_USERSTORAGE)
    {
       p_pstSrvIODevice->pvPrivateData = (void *)&m_stUserSection;
    }
        
    return ERR_OK;
}

/*************************************************************************************************/
/** @brief Close EEPROM设备

    @param[in] p_pstSrvIODeviceItem Device item
    @param[in] p_pstSrvIODevice     Device

    @retval _SRVERRORNO_H_
*/
/*************************************************************************************************/
static int16_t eeprom_close(StSrvIODeviceItem_t *p_pstSrvIODeviceItem, StSrvIODevice_t *p_pstSrvIODevice)
{
    if(p_pstSrvIODevice->pstDeviceItem->u8Count > 0)
    {
        p_pstSrvIODevice->pstDeviceItem->u8Count--;
    }
    
    p_pstSrvIODevice->pstDeviceItem = NULL;
    
    return ERR_OK;
}

/*************************************************************************************************/
/** @brief 从EEPROM设备读取数据

    @param[in] p_pstSrvIODevice   EEPROM Device
    @param[in] p_pvBuffer         保存数据的Buffer
    @param[in] p_u16BufferSize    需要读取数据长度 <n>字节

    @retval _SRVERRORNO_H_
*/
/*************************************************************************************************/
static int16_t eeprom_read(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    StAddressMap_t  *pstAddressMap = (StAddressMap_t  *)(p_pstSrvIODevice->pvPrivateData); 
    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= pstAddressMap -> u32endaddress)
    {
        return ERR_OUT_OF_RANGE;
    }
    memcpy((uint8_t *)p_pvBuffer,(uint8_t *)p_pstSrvIODevice->u32Position,p_u16BufferSize);
    return ERR_OK;
}

/*************************************************************************************************/
/** @brief 写数据到EEPROM 设备，每次写入4字节数据。

    @param[in] p_pstSrvIODevice   EEPROM Device
    @param[in] p_pvBuffer         写入数据的存储地址
    @param[in] p_u16BufferSize    写入数据的长度

    @retval _SRVERRORNO_H_
*/
/*************************************************************************************************/
static int16_t eeprom_write(StSrvIODevice_t *p_pstSrvIODevice, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    StAddressMap_t  *pstAddressMap = (StAddressMap_t  *)(p_pstSrvIODevice->pvPrivateData); 
    if(p_pstSrvIODevice->u32Position + p_u16BufferSize >= pstAddressMap -> u32endaddress)
    {
        return ERR_OUT_OF_RANGE;
    }

    /* 写入数据  */
    if(eeprom_writeword(p_pstSrvIODevice->u32Position,p_pvBuffer,p_u16BufferSize) == EN_EEPROM_ERR_WRITE)
    {
       return EN_EEPROM_ERR_WRITE;
    }
    return ERR_OK;
}

/*************************************************************************************************/
/** @brief 寻找写入数据的地址

    @param[in] p_pstSrvIODevice   EEPROM Device
    @param[in] p_u32Offset        地址偏移量，基地址+偏移量 = 写入数据地址

    @retval _SRVERRORNO_H_
*/
/*************************************************************************************************/
static int16_t eeprom_seek(StSrvIODevice_t *p_pstSrvIODevice, uint32_t p_u32Offset)
{
    StAddressMap_t  *pstAddressMap = (StAddressMap_t  *)(p_pstSrvIODevice->pvPrivateData); 
    if(pstAddressMap -> u32startaddress + p_u32Offset > pstAddressMap -> u32endaddress )
    {
       return ERR_OUT_OF_RANGE;
    }
    
    /* 写入数据地址 = 基地址 + 偏移量 */
    p_pstSrvIODevice->u32Position = pstAddressMap -> u32startaddress + p_u32Offset ;
    
    return ERR_OK;
}

/**
  * @brief  Program parameter at a specified address
  * @retval StFlashStatus_t Status
  */
static uint32_t eeprom_writeword(uint32_t p_u32Address, uint32_t *p_pu32Parameter, uint8_t p_u8ParaLength)
{
    uint8_t i =0;
    EnEEPROMStatus_t status = EN_EEPROM_ERR_OK;
    /* Unlocks the data memory */
    HAL_FLASHEx_DATAEEPROM_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    for(i = 0; i< p_u8ParaLength; i++)
    {
        if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAM_WORD,p_u32Address,*p_pu32Parameter) == HAL_OK)
        {
            /* 每次写入4字节数据 */
            p_u32Address += 4;
            p_pu32Parameter++;
        }
    }
    
    /* Locks the Data memory */
    HAL_FLASHEx_DATAEEPROM_Lock();

    return status;
}
