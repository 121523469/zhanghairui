#include "ringbuffer.h"
#include "string.h"

void ringbuffer_init(StRingBuffer_t *p_pstrbuf, void *p_pvdata, uint32_t p_u32len)
{
    p_pstrbuf->u32pread = p_pstrbuf->u32pwrite = 0;
    p_pstrbuf->pu8data = p_pvdata;
    p_pstrbuf->u32size = p_u32len;
    p_pstrbuf->s32error = 0;
}

int32_t ringbuffer_isempty(StRingBuffer_t *p_pstrbuf)
{
    return (p_pstrbuf->u32pread == p_pstrbuf->u32pwrite);
}

uint32_t ringbuffer_free(StRingBuffer_t *p_pstrbuf)
{
    int32_t free;

    free = p_pstrbuf->u32pread - p_pstrbuf->u32pwrite;
    if (free <= 0)
    {
        free += p_pstrbuf->u32size;
    }

    return free;
}

uint32_t ringbuffer_avail(StRingBuffer_t *p_pstrbuf)
{
    int32_t avail;

    avail = p_pstrbuf->u32pwrite - p_pstrbuf->u32pread;
    if (avail < 0)
    {
       avail += p_pstrbuf->u32size;
    }

    return avail;
}

void ringbuffer_flush(StRingBuffer_t *p_pstrbuf)
{
    p_pstrbuf->u32pread = p_pstrbuf->u32pwrite;
    p_pstrbuf->s32error = 0;
}

void ringbuffer_read(StRingBuffer_t *p_pstrbuf, uint8_t *p_pu8buf, uint32_t p_u32len)
{
    int32_t s32todo = p_u32len;
    int32_t s32split;
    p_pstrbuf->u32pread %= p_pstrbuf->u32size;

    s32split = (p_pstrbuf->u32pread + p_u32len > p_pstrbuf->u32size) ? p_pstrbuf->u32size - p_pstrbuf->u32pread : 0;
    if (s32split > 0) 
    {
        memcpy(p_pu8buf, p_pstrbuf->pu8data + p_pstrbuf->u32pread, s32split);
        p_pu8buf += s32split;
        s32todo -= s32split;
        p_pstrbuf->u32pread = 0;
    }
    memcpy(p_pu8buf, p_pstrbuf->pu8data + p_pstrbuf->u32pread, s32todo);

    p_pstrbuf->u32pread = p_pstrbuf->u32pread + s32todo;
}

uint32_t ringbuffer_write(StRingBuffer_t *p_pstrbuf, const uint8_t *p_pu8buf, uint32_t p_u32len)
{
    int32_t s32todo = p_u32len;
    int32_t s32split;
    p_pstrbuf->u32pwrite %= p_pstrbuf->u32size;

    s32split = (p_pstrbuf->u32pwrite + p_u32len > p_pstrbuf->u32size) ? p_pstrbuf->u32size - p_pstrbuf->u32pwrite : 0;

    if (s32split > 0) 
    {
        memcpy(p_pstrbuf->pu8data + p_pstrbuf->u32pwrite, p_pu8buf, s32split);
        p_pu8buf += s32split;
        s32todo -= s32split;
        p_pstrbuf->u32pwrite = 0;
    }

    memcpy(p_pstrbuf->pu8data + p_pstrbuf->u32pwrite, p_pu8buf, s32todo);
    p_pstrbuf->u32pwrite = p_pstrbuf->u32pwrite + s32todo;

    return p_u32len;
}

