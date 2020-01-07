/*************************************************************************************************/
/** @file   SrvIODevice.c
    @brief  Implementation of the Class SrvIODevice
    @company Chinastar
    @date 
    @author: dingtieshu
    @version  1.0
*/
/*************************************************************************************************/

/**************************************************************************************************
 * include files
 *************************************************************************************************/
#include "stdint.h"
#include "SrvErrorNo.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"

/*************************************************************************************************/
/** @brief  Open devices with ID p_enDeviceId and p_u16Flags
    @param[in,out]  me pointed to device
    @param[in] p_enDeviceId device ID
    @param[in] p_u16Flags open flags 
    @return int16
*/
/*************************************************************************************************/
int16_t SrvIODevice_open(StSrvIODevice_t* const me, EnDeviceId_t p_enDeviceId, uint16_t p_u16Flags)
{
    int16_t         s16Result = ERR_NODEV;

    me->u16Flags      = p_u16Flags;
    me->pvPrivateData = (void *)0;
    me->u32Position   = 0u;

    me->pstDeviceItem = SrvIODeviceItem_getDeviceItem(p_enDeviceId);
    if(me->pstDeviceItem != (void *)0)
    {
        if((me->pstDeviceItem->u16Flags & p_u16Flags) == p_u16Flags)
        {
            s16Result =  me->pstDeviceItem->pstIOOperations->open(me->pstDeviceItem, me);
        }
        else
        {
            s16Result = ERR_ACCESS;
        }
    }

    return s16Result;
}

/*************************************************************************************************/
/** @brief  Close devices pointed by me
    @param[in,out]  me pointed to device
    @return int16
*/
/*************************************************************************************************/
int16_t SrvIODevice_close(StSrvIODevice_t* const me)
{
    int16_t s16Result = ERR_BADF;

    if((me != (void *)0) && (me->pstDeviceItem != (void *)0))
    {
        s16Result = me->pstDeviceItem->pstIOOperations->close(me->pstDeviceItem, me);
        me->pstDeviceItem = (void *)0;
    }

    return s16Result;
}

/*************************************************************************************************/
/** @brief  Read devices pointed by me
    @param[in,out]  me pointed to device
    @param[out] p_pvBuffer pointer to buffer
    @param[in] p_u16BufferSize
    @return int16
*/
/*************************************************************************************************/
int16_t SrvIODevice_read(StSrvIODevice_t* const me, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result = ERR_BADF;
    }
    else if(me->pstDeviceItem == (void *)0)
    {
        s16Result = ERR_NODEV;
    }
    else if(p_pvBuffer == (void *)0)
    {
        s16Result = ERR_FAULT;
    }
    else if((me->u16Flags & IO_READ) != IO_READ)
    {
        s16Result = ERR_ACCESS;
    }
    else
    {
    	s16Result = me->pstDeviceItem->pstIOOperations->read(me, p_pvBuffer, p_u16BufferSize);
    }

    return s16Result;
}

/*************************************************************************************************/
/** @brief  Close devices pointed by me
    @param[in,out]  StSrvIODevice_t* const me
    @param[in] EnDeviceId_t p_enDeviceId
    @param[in] uint16_t p_u16Flags 
    @return int16
*/
/*************************************************************************************************/
int16_t SrvIODevice_write(StSrvIODevice_t* const me, void *p_pvBuffer, uint16_t p_u16BufferSize)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result = ERR_BADF;
    }
    else if(me->pstDeviceItem == (void *)0)
    {
        s16Result = ERR_NODEV;
    }
    else if(p_pvBuffer == (void *)0)
    {
        s16Result = ERR_FAULT;
    }
    else if((me->u16Flags & IO_WRITE) != IO_WRITE)
    {
        s16Result = ERR_ACCESS;
    }
    else
    {
    	s16Result = me->pstDeviceItem->pstIOOperations->write(me, p_pvBuffer, p_u16BufferSize);
    }

    return s16Result;
}
#if 0
/*============================================================================
  SrvIODevice_aio_read
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me
 * @param[in,out] p_pstAIOCtrlBlock
 *
 * @return int16
 */
int16_t SrvIODevice_aio_read(StSrvIODevice_t* const me, StAIOCtrlBlock_t *p_pstAIOCtrlBlock)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result = ERR_BADF;
    }
    else if(p_pstAIOCtrlBlock == (void *)0)
    {
        s16Result = ERR_FAULT;
    }
    else if(me->pstDeviceItem == (void *)0)
    {
        s16Result = ERR_NODEV;
    }
    else if((me->u16Flags & IO_READ) != IO_READ)
    {
        s16Result = ERR_ACCESS;
    }
    else
    {
    	s16Result = me->pstDeviceItem->pstIOOperations->aio_read(me, p_pstAIOCtrlBlock);
    }

    return s16Result;
}

/*============================================================================
  SrvIODevice_aio_write
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out]  me
 * @param[in,out]  p_pstAIOCtrlBlock
 *
 * @return int16
 */
int16_t SrvIODevice_aio_write(StSrvIODevice_t* const me, StAIOCtrlBlock_t *p_pstAIOCtrlBlock)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result =  ERR_BADF;
    }
    else if(me->pstDeviceItem == (void *)0)
    {
        s16Result = ERR_NODEV;
    }
    else if(p_pstAIOCtrlBlock == (void *)0)
    {
        s16Result = ERR_FAULT;
    }
    else if((me->u16Flags & IO_WRITE) != IO_WRITE)
    {
        s16Result = ERR_ACCESS;
    }
    else
    {
    	s16Result =  me->pstDeviceItem->pstIOOperations->aio_write(me, p_pstAIOCtrlBlock);
    }

    return s16Result;
}

/*============================================================================
  SrvIODevice_aio_cancel
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out]  me
 *
 * @return int16
 */
int16_t SrvIODevice_aio_cancel(StSrvIODevice_t* const me)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result =  ERR_BADF;
    }
    else
    {
        s16Result =  me->pstDeviceItem->pstIOOperations->aio_cancel(me);
    }

    return s16Result;
}

/*============================================================================
  SrvIODevice_aio_error
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out]  me
 *
 * @return int16
 */
int16_t SrvIODevice_aio_error(StSrvIODevice_t* const me)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result =  ERR_BADF;
    }
    else
    {
        s16Result = me->pstDeviceItem->pstIOOperations->aio_error(me);
    }

    return s16Result;
}
#endif 

/*************************************************************************************************/
/** @brief  seek write read position in devices pointed by me
    @param[in,out]  me pointed to device
    @param[in] p_u32Offset read/write offset in device 
    @return int16
*/
/*************************************************************************************************/
int16_t SrvIODevice_seek(StSrvIODevice_t* const me, uint32_t p_u32Offset)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result =  ERR_BADF;
    }
    else if(me->pstDeviceItem == (void *)0)
    {
        s16Result = ERR_NODEV;
    }
    else
    {
        s16Result =  me->pstDeviceItem->pstIOOperations->seek(me, p_u32Offset);
    }

    return s16Result;
}

/*************************************************************************************************/
/** @brief  io control operation for device
    @param[in,out]  me pointed to device
    @param[in] p_u32Command command to be executed
    @param[in] p_u32Arg argument for command
    @return int16
*/
/*************************************************************************************************/
int16_t SrvIODevice_ioctl(StSrvIODevice_t* const me, uint32_t p_u32Command, uint32_t p_u32Arg)
{
    int16_t s16Result = 0;

    if(me == (void *)0)
    {
        s16Result =  ERR_BADF;
    }
    else if(me->pstDeviceItem == (void *)0)
    {
        s16Result = ERR_NODEV;
    }
    else
    {
        s16Result =  me->pstDeviceItem->pstIOOperations->ioctl(me->pstDeviceItem, me, p_u32Command, p_u32Arg);
    }

    return s16Result;
}



