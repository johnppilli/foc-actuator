#include "svpwm.h"
#include <math.h>

static void limit_to_circle(float *va, float *vb, float v_bus)
{
    float max_mag = SVPWM_MAX_MAG(v_bus);
    float mag2 = (*va) * (*va) + (*vb) * (*vb);
    if (mag2 > max_mag * max_mag) {
        float k = max_mag / sqrtf(mag2);
        *va *= k;
        *vb *= k;
    }
}

foc_abc_t svpwm(float v_alpha, float v_beta, float v_bus)
{
    limit_to_circle(&v_alpha, &v_beta, v_bus);

    foc_ab_t ab = { v_alpha, v_beta };
    foc_abc_t v = foc_inv_clarke(ab);

    float vmax = v.a, vmin = v.a;
    if (v.b > vmax) vmax = v.b;
    if (v.c > vmax) vmax = v.c;
    if (v.b < vmin) vmin = v.b;
    if (v.c < vmin) vmin = v.c;

    /* Shift all three by the same common-mode voltage so the waveform is
     * centred in the bus. This is exactly the symmetric-zero-vector SVPWM. */
    float v_com = -0.5f * (vmax + vmin);
    float inv_bus = 1.0f / v_bus;

    foc_abc_t d;
    d.a = foc_clampf(0.5f + (v.a + v_com) * inv_bus, 0.0f, 1.0f);
    d.b = foc_clampf(0.5f + (v.b + v_com) * inv_bus, 0.0f, 1.0f);
    d.c = foc_clampf(0.5f + (v.c + v_com) * inv_bus, 0.0f, 1.0f);
    return d;
}

int svpwm_sector(float v_alpha, float v_beta)
{
    /* Sign test against the three 120-degree-spaced axes. */
    int n = 0;
    if (v_beta > 0.0f) n += 1;
    if ((FOC_SQRT3 * v_alpha - v_beta) > 0.0f) n += 2;
    if ((-FOC_SQRT3 * v_alpha - v_beta) > 0.0f) n += 4;
    static const int map[8] = { 0, 2, 6, 1, 4, 3, 5, 0 };
    int s = map[n];
    return s == 0 ? 1 : s;   /* exact zero vector: call it sector 1 */
}

foc_abc_t svpwm_classic(float v_alpha, float v_beta, float v_bus)
{
    limit_to_circle(&v_alpha, &v_beta, v_bus);

    float mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    float ang = foc_wrap_2pi(atan2f(v_beta, v_alpha));
    int sector = (int)(ang / (FOC_PI / 3.0f)) + 1;
    if (sector > 6) sector = 6;
    float th = ang - (float)(sector - 1) * (FOC_PI / 3.0f);

    /* Dwell times as fractions of the period (Ts = 1). */
    float k  = FOC_SQRT3 * mag / v_bus;
    float t1 = k * sinf(FOC_PI / 3.0f - th);
    float t2 = k * sinf(th);
    float t0 = 1.0f - t1 - t2;
    if (t0 < 0.0f) t0 = 0.0f;
    float h = 0.5f * t0;

    foc_abc_t d;
    switch (sector) {
    case 1: d.a = t1 + t2 + h; d.b = t2 + h;      d.c = h;           break;
    case 2: d.a = t1 + h;      d.b = t1 + t2 + h; d.c = h;           break;
    case 3: d.a = h;           d.b = t1 + t2 + h; d.c = t2 + h;      break;
    case 4: d.a = h;           d.b = t1 + h;      d.c = t1 + t2 + h; break;
    case 5: d.a = t2 + h;      d.b = h;           d.c = t1 + t2 + h; break;
    default:d.a = t1 + t2 + h; d.b = h;           d.c = t1 + h;      break;
    }
    d.a = foc_clampf(d.a, 0.0f, 1.0f);
    d.b = foc_clampf(d.b, 0.0f, 1.0f);
    d.c = foc_clampf(d.c, 0.0f, 1.0f);
    return d;
}

foc_abc_t svpwm_sine(float v_alpha, float v_beta, float v_bus)
{
    foc_ab_t ab = { v_alpha, v_beta };
    foc_abc_t v = foc_inv_clarke(ab);
    float inv_bus = 1.0f / v_bus;
    foc_abc_t d;
    d.a = foc_clampf(0.5f + v.a * inv_bus, 0.0f, 1.0f);
    d.b = foc_clampf(0.5f + v.b * inv_bus, 0.0f, 1.0f);
    d.c = foc_clampf(0.5f + v.c * inv_bus, 0.0f, 1.0f);
    return d;
}

foc_abc_t svpwm_duty_to_phase_voltage(foc_abc_t duty, float v_bus)
{
    float pa = duty.a * v_bus, pb = duty.b * v_bus, pc = duty.c * v_bus;
    float vn = (pa + pb + pc) * (1.0f / 3.0f);
    foc_abc_t v = { pa - vn, pb - vn, pc - vn };
    return v;
}
