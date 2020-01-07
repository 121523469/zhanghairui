/****************************************************/
/** @file   SrvList.h
    @brief  Header file of the Class SrvList
    @company Chinastar
    @date 
    @author: dingtieshu
    @version  1.0

*/
/*****************************************************/
#ifndef _SRVLIST_H_
#define _SRVLIST_H_
struct StSrvListLink
{
    struct StSrvListLink *pstNext;
    struct StSrvListLink *pstPrev;
};
typedef struct StSrvListLink StSrvListLink_t;

typedef struct 
{
     StSrvListLink_t stHead;
     StSrvListLink_t *pstActualIter;
     StSrvListLink_t *pstTmp;

}StSrvList_t;

extern void SrvList_configure(StSrvList_t* const me);
extern uint8_t SrvList_isEmpty(StSrvList_t* const me);
extern void SrvList_addHead(StSrvList_t* const me, StSrvListLink_t* p_pstNew);
extern void SrvList_addTail(StSrvList_t* const me, StSrvListLink_t* p_pstNew);
extern void SrvList_deleteEntry(StSrvList_t* const me, StSrvListLink_t* p_pstEntry);
extern StSrvListLink_t* SrvList_getFirstEntry(StSrvList_t* const me);
extern StSrvListLink_t* SrvList_getNextEntry(StSrvList_t* const me);
#endif


