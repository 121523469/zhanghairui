/*
 * ringbuffer.h: ring buffer implementation for the dvb driver
 */

#ifndef _CHARACTER_CONVERT_H_
#define _CHARACTER_CONVERT_H_

#include <stdlib.h>
#include <stdint.h>
#define HEXTOCHAR(hx)        (uint8_t)((hx) >= 0x0A ? (hx) + 'A' - 0x0A : (hx) + '0')                  /**< Hex to char */
#define CHARTOHEX(charData)  (uint8_t)((charData) >= 0x41 ? (charData) - 'A' + 0x0A : (charData) - '0')    /**< char to hex */

extern uint8_t Hex2Char(int8_t* p_pvSrc, int8_t* p_pvDest, volatile uint8_t p_u8Length);
extern uint8_t Char2Hex(uint8_t* p_pvSrc, uint8_t* p_pvDest, volatile uint16_t p_u16Length);
extern int32_t GetStrVale(int8_t *Src, int8_t *Start, int8_t *End, int8_t CmdLen);
extern int32_t GetInsideStr(int8_t *Src, int8_t *Des,int8_t *Start, int8_t *End, uint16_t size);
#endif /* _CHARACTER_CONVERT_H_ */

