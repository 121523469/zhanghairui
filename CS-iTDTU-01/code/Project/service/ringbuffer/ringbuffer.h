/*
 * ringbuffer.h: ring buffer implementation for the dvb driver
 */

#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#include <stdlib.h>
#include <stdint.h>


#define RING_BUFFER_ERR     (-1)
#define RING_BUFFER_OK    (0)

typedef struct 
{
        uint8_t   *pu8data;
        uint32_t  u32size;
        uint32_t  u32pread;
        uint32_t  u32pwrite;
        int32_t   s32error;
}StRingBuffer_t;

/* initialize ring buffer */
extern void ringbuffer_init(StRingBuffer_t *p_pstrbuf, void *p_pvdata, uint32_t len);

/* test whether buffer is empty */
extern int32_t ringbuffer_isempty(StRingBuffer_t *p_pstrbuf);

/* return the number of free bytes in the buffer */
extern uint32_t ringbuffer_free(StRingBuffer_t *p_pstrbuf);

/* return the number of bytes waiting in the buffer */
extern uint32_t ringbuffer_avail(StRingBuffer_t *p_pstrbuf);

/*
** Reset the read and write pointers to zero and flush the buffer
** This counts as a read and write operation
*/
extern void ringbuffer_reset(StRingBuffer_t *p_pstrbuf);

/* flush buffer */
extern void ringbuffer_flush(StRingBuffer_t *p_pstrbuf);

extern void ringbuffer_read(StRingBuffer_t *p_pstrbuf, uint8_t *p_pu8buf, uint32_t p_u32len);

extern uint32_t ringbuffer_write(StRingBuffer_t *p_pstrbuf, const uint8_t *p_pu8buf, uint32_t p_u32len);

#endif /* _RINGBUFFER_H_ */

