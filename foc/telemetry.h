/*
 * telemetry.h - binary telemetry frame + byte ring buffer.
 *
 * Frame layout (little-endian, packed):
 *   0xAA 0x55 | len (u8, = TELEM_PAYLOAD_BYTES) | payload | xor-checksum of payload
 * Payload:
 *   u32 tick | f32 theta_e theta_m omega_m i_d i_q i_q_ref v_d v_q v_bus | u8 mode | u8 fault
 * Python struct format for the payload: "<I9fBB" (see tools/telemetry.py).
 *
 * No HAL dependencies so the codec is unit-tested on the desktop and the same
 * bytes are decoded by the Python tool.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEM_SYNC0 0xAAu
#define TELEM_SYNC1 0x55u
#define TELEM_PAYLOAD_BYTES 42u
#define TELEM_FRAME_BYTES   (3u + TELEM_PAYLOAD_BYTES + 1u)

typedef struct {
    uint32_t tick;
    float theta_e, theta_m, omega_m;
    float i_d, i_q, i_q_ref;
    float v_d, v_q;
    float v_bus;
    uint8_t mode;
    uint8_t fault;
} telem_frame_t;

/* Encode into buf (>= TELEM_FRAME_BYTES). Returns bytes written. */
size_t telem_encode(const telem_frame_t *f, uint8_t *buf);

/* Byte-at-a-time decoder that resynchronises on garbage. */
typedef struct {
    uint8_t buf[TELEM_FRAME_BYTES];
    uint8_t n;
} telem_decoder_t;

void telem_decoder_init(telem_decoder_t *d);
/* Returns true when a complete, checksum-valid frame was just parsed into *out. */
bool telem_decode_byte(telem_decoder_t *d, uint8_t byte, telem_frame_t *out);

/* Single-producer / single-consumer byte ring. Producer: an ISR calling
 * telem_ring_push_frame(); consumer: the main loop draining to the UART. */
#define TELEM_RING_BYTES 2048u   /* power of two */

typedef struct {
    uint8_t buf[TELEM_RING_BYTES];
    volatile uint32_t head;  /* write index */
    volatile uint32_t tail;  /* read index */
    uint32_t dropped;        /* frames dropped because the ring was full */
} telem_ring_t;

void     telem_ring_init(telem_ring_t *r);
uint32_t telem_ring_used(const telem_ring_t *r);
uint32_t telem_ring_free(const telem_ring_t *r);
/* Push a whole frame or nothing. Returns false (and counts a drop) when full. */
bool     telem_ring_push_frame(telem_ring_t *r, const telem_frame_t *f);
/* Pop up to max bytes into out. Returns bytes copied. */
uint32_t telem_ring_pop(telem_ring_t *r, uint8_t *out, uint32_t max);

#ifdef __cplusplus
}
#endif
#endif /* TELEMETRY_H */
