#include "telemetry.h"
#include <string.h>

/* Little-endian on Cortex-M and on every desktop we test on; memcpy keeps it
 * alignment-safe. */
static uint8_t *put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    return p + 4;
}

static uint8_t *put_f32(uint8_t *p, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    return put_u32(p, u);
}

static const uint8_t *get_u32(const uint8_t *p, uint32_t *v)
{
    *v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return p + 4;
}

static const uint8_t *get_f32(const uint8_t *p, float *f)
{
    uint32_t u;
    p = get_u32(p, &u);
    memcpy(f, &u, 4);
    return p;
}

size_t telem_encode(const telem_frame_t *f, uint8_t *buf)
{
    uint8_t *p = buf;
    *p++ = TELEM_SYNC0;
    *p++ = TELEM_SYNC1;
    *p++ = (uint8_t)TELEM_PAYLOAD_BYTES;
    uint8_t *payload = p;
    p = put_u32(p, f->tick);
    p = put_f32(p, f->theta_e);
    p = put_f32(p, f->theta_m);
    p = put_f32(p, f->omega_m);
    p = put_f32(p, f->i_d);
    p = put_f32(p, f->i_q);
    p = put_f32(p, f->i_q_ref);
    p = put_f32(p, f->v_d);
    p = put_f32(p, f->v_q);
    p = put_f32(p, f->v_bus);
    *p++ = f->mode;
    *p++ = f->fault;
    uint8_t x = 0;
    for (const uint8_t *q = payload; q < p; q++) x ^= *q;
    *p++ = x;
    return (size_t)(p - buf);
}

static bool decode_payload(const uint8_t *p, telem_frame_t *out)
{
    p = get_u32(p, &out->tick);
    p = get_f32(p, &out->theta_e);
    p = get_f32(p, &out->theta_m);
    p = get_f32(p, &out->omega_m);
    p = get_f32(p, &out->i_d);
    p = get_f32(p, &out->i_q);
    p = get_f32(p, &out->i_q_ref);
    p = get_f32(p, &out->v_d);
    p = get_f32(p, &out->v_q);
    p = get_f32(p, &out->v_bus);
    out->mode = *p++;
    out->fault = *p++;
    return true;
}

void telem_decoder_init(telem_decoder_t *d)
{
    d->n = 0;
}

bool telem_decode_byte(telem_decoder_t *d, uint8_t byte, telem_frame_t *out)
{
    if (d->n == 0) {
        if (byte == TELEM_SYNC0) d->buf[d->n++] = byte;
        return false;
    }
    if (d->n == 1) {
        if (byte == TELEM_SYNC1) d->buf[d->n++] = byte;
        else d->n = (byte == TELEM_SYNC0) ? 1 : 0;
        return false;
    }
    if (d->n == 2) {
        if (byte == TELEM_PAYLOAD_BYTES) d->buf[d->n++] = byte;
        else d->n = (byte == TELEM_SYNC0) ? 1 : 0;
        return false;
    }
    d->buf[d->n++] = byte;
    if (d->n < TELEM_FRAME_BYTES) return false;

    d->n = 0;
    uint8_t x = 0;
    for (uint32_t i = 3; i < 3 + TELEM_PAYLOAD_BYTES; i++) x ^= d->buf[i];
    if (x != d->buf[TELEM_FRAME_BYTES - 1]) return false;
    return decode_payload(&d->buf[3], out);
}

/* ---- ring --------------------------------------------------------------- */

void telem_ring_init(telem_ring_t *r)
{
    r->head = 0;
    r->tail = 0;
    r->dropped = 0;
}

uint32_t telem_ring_used(const telem_ring_t *r)
{
    return (r->head - r->tail) & (TELEM_RING_BYTES - 1u);
}

uint32_t telem_ring_free(const telem_ring_t *r)
{
    return TELEM_RING_BYTES - 1u - telem_ring_used(r);
}

bool telem_ring_push_frame(telem_ring_t *r, const telem_frame_t *f)
{
    uint8_t tmp[TELEM_FRAME_BYTES];
    size_t n = telem_encode(f, tmp);
    if (telem_ring_free(r) < n) {
        r->dropped++;
        return false;
    }
    uint32_t h = r->head;
    for (size_t i = 0; i < n; i++) {
        r->buf[h] = tmp[i];
        h = (h + 1u) & (TELEM_RING_BYTES - 1u);
    }
    r->head = h;   /* single store publishes the frame */
    return true;
}

uint32_t telem_ring_pop(telem_ring_t *r, uint8_t *out, uint32_t max)
{
    uint32_t used = telem_ring_used(r);
    if (max > used) max = used;
    uint32_t t = r->tail;
    for (uint32_t i = 0; i < max; i++) {
        out[i] = r->buf[t];
        t = (t + 1u) & (TELEM_RING_BYTES - 1u);
    }
    r->tail = t;
    return max;
}
