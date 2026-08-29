#include "telemetry.h"
#include "minitest.h"
#include <stdio.h>
#include <string.h>

static telem_frame_t sample(uint32_t tick)
{
    telem_frame_t f;
    f.tick = tick;
    f.theta_e = 1.25f;
    f.theta_m = -0.5f;
    f.omega_m = 12.5f;
    f.i_d = 0.01f;
    f.i_q = 0.75f;
    f.i_q_ref = 0.8f;
    f.v_d = -0.2f;
    f.v_q = 3.3f;
    f.v_bus = 12.0f;
    f.mode = 3;
    f.fault = 0;
    return f;
}

static void assert_frames_equal(const telem_frame_t *a, const telem_frame_t *b)
{
    ASSERT_EQ_INT(a->tick, b->tick);
    ASSERT_NEAR(a->theta_e, b->theta_e, 0.0f);
    ASSERT_NEAR(a->theta_m, b->theta_m, 0.0f);
    ASSERT_NEAR(a->omega_m, b->omega_m, 0.0f);
    ASSERT_NEAR(a->i_d, b->i_d, 0.0f);
    ASSERT_NEAR(a->i_q, b->i_q, 0.0f);
    ASSERT_NEAR(a->i_q_ref, b->i_q_ref, 0.0f);
    ASSERT_NEAR(a->v_d, b->v_d, 0.0f);
    ASSERT_NEAR(a->v_q, b->v_q, 0.0f);
    ASSERT_NEAR(a->v_bus, b->v_bus, 0.0f);
    ASSERT_EQ_INT(a->mode, b->mode);
    ASSERT_EQ_INT(a->fault, b->fault);
}

static void test_roundtrip(void)
{
    telem_frame_t f = sample(123456);
    uint8_t buf[TELEM_FRAME_BYTES];
    size_t n = telem_encode(&f, buf);
    ASSERT_EQ_INT(n, TELEM_FRAME_BYTES);
    ASSERT_EQ_INT(buf[0], 0xAA);
    ASSERT_EQ_INT(buf[1], 0x55);
    ASSERT_EQ_INT(buf[2], TELEM_PAYLOAD_BYTES);

    telem_decoder_t d;
    telem_decoder_init(&d);
    telem_frame_t out;
    int got = 0;
    for (size_t i = 0; i < n; i++) if (telem_decode_byte(&d, buf[i], &out)) got++;
    ASSERT_EQ_INT(got, 1);
    assert_frames_equal(&f, &out);
}

static void test_resync_through_garbage(void)
{
    telem_frame_t f = sample(7);
    uint8_t good[TELEM_FRAME_BYTES];
    telem_encode(&f, good);

    /* garbage, a false sync, a truncated frame, then two good frames */
    uint8_t stream[512];
    size_t n = 0;
    const uint8_t junk[] = { 0x00, 0xAA, 0x12, 0xAA, 0x55, 0x03, 0xFF, 0xAA, 0xAA, 0x55 };
    memcpy(stream + n, junk, sizeof junk); n += sizeof junk;
    memcpy(stream + n, good, 20); n += 20;                  /* truncated */
    memcpy(stream + n, good, TELEM_FRAME_BYTES); n += TELEM_FRAME_BYTES;
    f.tick = 8;
    telem_encode(&f, good);
    memcpy(stream + n, good, TELEM_FRAME_BYTES); n += TELEM_FRAME_BYTES;

    telem_decoder_t d;
    telem_decoder_init(&d);
    telem_frame_t out;
    int got = 0;
    uint32_t last_tick = 0;
    for (size_t i = 0; i < n; i++) {
        if (telem_decode_byte(&d, stream[i], &out)) { got++; last_tick = out.tick; }
    }
    ASSERT_TRUE(got >= 1);
    ASSERT_EQ_INT(last_tick, 8);
}

static void test_bad_checksum_rejected(void)
{
    telem_frame_t f = sample(1);
    uint8_t buf[TELEM_FRAME_BYTES];
    telem_encode(&f, buf);
    buf[10] ^= 0x01;
    telem_decoder_t d;
    telem_decoder_init(&d);
    telem_frame_t out;
    int got = 0;
    for (size_t i = 0; i < TELEM_FRAME_BYTES; i++) if (telem_decode_byte(&d, buf[i], &out)) got++;
    ASSERT_EQ_INT(got, 0);
}

static void test_ring(void)
{
    telem_ring_t r;
    telem_ring_init(&r);
    ASSERT_EQ_INT(telem_ring_used(&r), 0);

    /* fill until it refuses, count frames */
    int pushed = 0;
    for (int i = 0; i < 200; i++) {
        if (telem_ring_push_frame(&r, &(telem_frame_t){ .tick = (uint32_t)i })) pushed++;
    }
    ASSERT_TRUE(pushed > 30);
    ASSERT_TRUE(r.dropped > 0);
    ASSERT_EQ_INT(telem_ring_used(&r), (long)pushed * TELEM_FRAME_BYTES);

    /* drain in odd-sized chunks and decode: frames come out in order */
    telem_decoder_t d;
    telem_decoder_init(&d);
    telem_frame_t out;
    uint8_t chunk[37];
    uint32_t next = 0;
    int decoded = 0;
    for (;;) {
        uint32_t n = telem_ring_pop(&r, chunk, sizeof chunk);
        if (n == 0) break;
        for (uint32_t i = 0; i < n; i++) {
            if (telem_decode_byte(&d, chunk[i], &out)) {
                ASSERT_EQ_INT(out.tick, next);
                next++;
                decoded++;
            }
        }
    }
    ASSERT_EQ_INT(decoded, pushed);
    ASSERT_EQ_INT(telem_ring_used(&r), 0);

    /* wrap the indices around many times */
    for (int i = 0; i < 5000; i++) {
        ASSERT_TRUE(telem_ring_push_frame(&r, &(telem_frame_t){ .tick = (uint32_t)i + 1000 }));
        uint32_t n = telem_ring_pop(&r, chunk, sizeof chunk);
        for (uint32_t k = 0; k < n; k++) {
            if (telem_decode_byte(&d, chunk[k], &out)) ASSERT_EQ_INT(out.tick, next++ + 1000 - pushed);
        }
        while (telem_ring_used(&r) > 0) {
            n = telem_ring_pop(&r, chunk, sizeof chunk);
            for (uint32_t k = 0; k < n; k++) {
                if (telem_decode_byte(&d, chunk[k], &out)) ASSERT_EQ_INT(out.tick, next++ + 1000 - pushed);
            }
        }
    }
}

/* Write a few frames to disk so tools/telemetry.py --selftest can prove the
 * Python decoder agrees with the C encoder. */
static void write_sample_file(void)
{
    FILE *fp = fopen("build/telem_sample.bin", "wb");
    if (!fp) return;
    uint8_t buf[TELEM_FRAME_BYTES];
    const uint8_t junk[] = { 0x01, 0xAA, 0x02 };
    fwrite(junk, 1, sizeof junk, fp);
    for (uint32_t i = 0; i < 5; i++) {
        telem_frame_t f = sample(i);
        f.i_q = 0.1f * (float)i;
        telem_encode(&f, buf);
        fwrite(buf, 1, sizeof buf, fp);
    }
    fclose(fp);
}

int main(void)
{
    RUN_TEST(test_roundtrip);
    RUN_TEST(test_resync_through_garbage);
    RUN_TEST(test_bad_checksum_rejected);
    RUN_TEST(test_ring);
    write_sample_file();
    MINITEST_MAIN_END();
}
