/****************************************************/
/** @file   SrvIODeviceItem.c
    @brief  Implementation of the Class SrvIODeviceItem
    @company Chinastar
    @date 
    @author: dingtieshu
    @version  1.0

*/
/*****************************************************/


/*****************************************************************************/
/* import section */
/*****************************************************************************/
#include "stdint.h"
#include "SrvIODevice.h"
#define true  (1==1)
#define false (1==0)
/*****************************************************************************/
/* static function prototypes */
/*****************************************************************************/

/*****************************************************************************/
/* static variables */
/*****************************************************************************/
static StSrvList_t m_stDeviceLinkList; /**<     */

/*****************************************************************************/
/* global functions */
/*****************************************************************************/

/*============================================================================
  SrvIODeviceItem_unregister
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out]  me
 * @param[in]      p_enDeviceId
 *
 * @return void
 */
void SrvIODeviceItem_unregister(StSrvIODeviceItem_t* const me, EnDeviceId_t p_enDeviceId)
{
}

/*============================================================================
  SrvIODeviceItem_register
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out]  me
 * @param[in]      p_enDeviceId
 * @param[in,out]  *p_pstIOOperations
 * @param[in]      p_u16Flags
 *
 * @return void
 */
void SrvIODeviceItem_register(StSrvIODeviceItem_t* const me, EnDeviceId_t p_enDeviceId, StSrvIOOperations_t *p_pstIOOperations, uint16_t p_u16Flags)
{
    static uint8_t first = true;

    me->enDeviceId      = p_enDeviceId;
    me->pstIOOperations = p_pstIOOperations;
    me->u8Count         = 0u;
    me->u16Flags        = p_u16Flags;
    if(first == true)
    {
        first = false;
        SrvList_configure(&m_stDeviceLinkList);
    }

    SrvList_addTail(&m_stDeviceLinkList, &me->stListLink);
}

/*============================================================================
  SrvIODeviceItem_getDeviceItem
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in] p_enDeviceId
 *
 * @return SrvIODeviceItem*
 */
StSrvIODeviceItem_t* SrvIODeviceItem_getDeviceItem(EnDeviceId_t p_enDeviceId)
{
    StSrvList_t         *pstLinkedList = &m_stDeviceLinkList;
    StSrvIODeviceItem_t *pstDeviceItem;


    pstDeviceItem = (StSrvIODeviceItem_t*) SrvList_getFirstEntry(pstLinkedList);
    while(pstDeviceItem != (void *)0)
    {
        if(pstDeviceItem->enDeviceId == p_enDeviceId)
        {
            break;
        }
        pstDeviceItem= (StSrvIODeviceItem_t*) SrvList_getNextEntry(pstLinkedList);
    }

    return  pstDeviceItem;
}

