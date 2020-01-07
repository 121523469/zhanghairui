#include "character_convert.h"
#include <string.h>
#include <stdio.h>
/** @brief convert Hex to char

    @param[in] p_pvSrc
    @param[in] p_pvDest
    @param[in] p_u8Length

    @retval    uint8_t       dest data lenght   
*/
uint8_t Hex2Char(int8_t* p_pvSrc,int8_t* p_pvDest,volatile uint8_t p_u8Length)
{
    uint8_t i = 0;
    uint8_t j = 0;
    while(p_u8Length--)
    {
        p_pvDest[i++] = HEXTOCHAR((p_pvSrc[j]>>4)&0xF);
        p_pvDest[i++] = HEXTOCHAR(p_pvSrc[j]&0xF);
        j++;
    }
    return i;
}

/** @brief convert char to Hex

    @param[in] p_pvSrc
    @param[in] p_pvDest
    @param[in] p_u8Length

    @retval    uint8_t       dest data lenght
*/
uint8_t Char2Hex(uint8_t* p_pvSrc,uint8_t* p_pvDest,volatile uint16_t p_u16Length)
{
    uint16_t i = 0;
    uint16_t j = 0;
    while(p_u16Length--)
    {
        p_pvDest[i++] = (CHARTOHEX(p_pvSrc[j]) << 4) + (CHARTOHEX(p_pvSrc[j+1]) & 0x0F);
        j = j + 2;
    }
    return i;
}

int32_t GetStrVale(int8_t *Src, int8_t *Start, int8_t *End, int8_t CmdLen)
{
    uint8_t StrLen = 0;
    int8_t *pStrStart = NULL;
    int8_t *pStrEnd = NULL;
    int8_t StrBuf[16] = {0};

    pStrStart = (int8_t*)strstr((char*)Src, (char*)Start);
    if(NULL == pStrStart)
    {
        printf("GetStrVale:no Start str!\r\n");
        return -1;
    }

    pStrEnd = (int8_t*)strstr((char*)pStrStart + strlen((char*)Start), (char*)End);
    if(NULL == pStrEnd)
    {
        printf("GetStrVale:no End str!\r\n");
        return -1;
    }

    StrLen = pStrEnd - pStrStart - CmdLen;

    if(StrLen < 1)
    {
        printf("StrLen err, StrLen = %d!\r\n", StrLen);
        return -1;
    }

    if(StrLen > sizeof(StrBuf))
    {
        printf("StrLen beyond max size, StrLen = %d, MaxSize = %d\r\n", StrLen,sizeof(StrBuf));
        return -1;
    }

    memcpy((char *)StrBuf, pStrStart + CmdLen, StrLen);
    return atoi((char*)StrBuf);
}

int32_t GetInsideStr(int8_t *Src, int8_t *Des,int8_t *Start, int8_t *End, uint16_t size)
{
    uint8_t StrLen = 0;
    int8_t *pStrStart = NULL;
    int8_t *pStrEnd = NULL;

    pStrStart = (int8_t*)strstr((char*)Src, (char*)Start);
    if(NULL == pStrStart)
    {
        printf("GetInsideStr:no Start str!\r\n");
        return -1;
    }

    pStrEnd = (int8_t*)strstr((char*)Src + strlen((char*)Start), (char*)End);
    if(NULL == pStrEnd)
    {
        printf("GetInsideStr:no End str!\r\n");
        return -1;
    }

    StrLen = pStrEnd - pStrStart - strlen((char*)Start);

    if(StrLen < 1)
    {
        printf("StrLen err, StrLen = %d!\r\n", StrLen);
        return -1;
    }

    if(StrLen > size)
    {
        printf("StrLen beyond max size! StrLen = %d, MaxSize = %d\r\n",StrLen,size);
        return -1;
    }
    memcpy((char *)Des, pStrStart + strlen((char*)Start), StrLen);
    return StrLen;
}
