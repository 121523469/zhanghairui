/****************************************************/
/** @file   SrvList.c
    @brief  Implementation of the Class SrvList
    @company Chinastar
    @date 
    @author: dingtieshu
    @version  1.0

*/
/*****************************************************/


/****************************************************************************
 * includes
 ****************************************************************************/
#include "stdint.h"
#include "SrvList.h"

/*****************************************************************************
 * static function prototypes
 ******************************************************************************/

/*****************************************************************************
 * static variables
 *****************************************************************************/

/*****************************************************************************
 * global variables
 *****************************************************************************/

/*****************************************************************************
 * global function prototypes
 *****************************************************************************/


/*============================================================================
  SrvList_configure
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me

 *
 * @return void
 */
void SrvList_configure(StSrvList_t* const me)
{
    me->stHead.pstNext = &me->stHead;
    me->stHead.pstPrev = &me->stHead;
}

/*============================================================================
  SrvList_bIsEmpty
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me

 *
 * @return bool
 */
uint8_t SrvList_isEmpty(StSrvList_t* const me)
{
    return (uint8_t)(me->stHead.pstNext == &me->stHead ? 1u : 0u);
}

/*============================================================================
  SrvList_addHead
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me
 * @param[in,out] *p_pstNew
 *
 * @return void
 */
void SrvList_addHead(StSrvList_t* const me, StSrvListLink_t* p_pstNew)
{
    StSrvListLink_t* pstInsert = &me->stHead;

    pstInsert->pstPrev  = p_pstNew;
    p_pstNew->pstNext   = pstInsert->pstNext;
    p_pstNew->pstPrev   = pstInsert;
    pstInsert->pstNext  = p_pstNew;
}

/*============================================================================
  SrvList_addTail
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me
 * @param[in,out] *p_pstNew
 *
 * @return void
 */
void SrvList_addTail(StSrvList_t* const me, StSrvListLink_t* p_pstNew)
{
    StSrvListLink_t* pstTail = me->stHead.pstPrev;

    pstTail->pstNext->pstPrev  = p_pstNew;
    p_pstNew->pstNext   = pstTail->pstNext;
    p_pstNew->pstPrev   = pstTail;
    pstTail->pstNext    = p_pstNew;
}

/*============================================================================
  SrvList_deleteEntry
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me
 * @param[in,out] *p_pstEntry
 *
 * @return void
 */
void SrvList_deleteEntry(StSrvList_t* const me, StSrvListLink_t* p_pstEntry)
{
    StSrvListLink_t* pstLPrev = p_pstEntry->pstPrev;
    StSrvListLink_t* pstLNext = p_pstEntry->pstNext;

    pstLNext->pstPrev = pstLPrev;
    pstLPrev->pstNext = pstLNext;
}

/*============================================================================
  SrvList_getFirstEntry
  ============================================================================*/
/**
 *
 *
 * @brief
 *
 * @param[in,out] me

 *
 * @return StSrvListLink_t*
 */
StSrvListLink_t* SrvList_getFirstEntry(StSrvList_t* const me)
{
    StSrvListLink_t* pstResult = (void *)0;

    me->pstActualIter = me->stHead.pstNext;
    me->pstTmp        = me->pstActualIter->pstNext;

    if(me->pstActualIter != &me->stHead)
    {
        pstResult = me->pstActualIter;
    }

    return  pstResult;
}

/**
 *
 *
 * @brief
 *
 * @param[in,out] me

 *
 * @return StSrvListLink_t*
 */
StSrvListLink_t* SrvList_getNextEntry(StSrvList_t* const me)
{
    StSrvListLink_t* pstResult = (void *)0;

    me->pstActualIter = me->pstTmp;
    me->pstTmp        = me->pstActualIter->pstNext;

    if(me->pstActualIter != &me->stHead)
    {
        pstResult = me->pstActualIter;
    }

    return  pstResult;
}

