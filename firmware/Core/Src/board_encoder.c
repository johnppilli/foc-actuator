/*
 * board_encoder.c - AS5600 over I2C1 (PB7 SDA, PB8 SCL), non-blocking.
 *
 * A 2-byte register read at 400 kHz takes ~100 us, twice the control period,
 * so reads are started from the loop every BOARD_ENC_DECIMATION ticks and
 * complete in the I2C interrupt. The loop picks up the latest value with
 * board_encoder_take(). I2C interrupts run at a lower priority than the
 * control loop, so the callback and the loop never run concurrently.
 */
#include "board.h"

static uint8_t s_buf[2];
static volatile uint16_t s_raw;
static volatile bool s_fresh;
static volatile uint32_t s_errors;
static uint32_t s_busy_since_ms;

void board_encoder_init(void)
{
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    s_fresh = false;
    s_errors = 0;
    s_busy_since_ms = 0;
}

void board_encoder_request(void)
{
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) return;   /* previous read still running */
    if (HAL_I2C_Mem_Read_IT(&hi2c1, (uint16_t)(BOARD_ENC_I2C_ADDR << 1), BOARD_ENC_REG_RAW_ANGLE,
                            I2C_MEMADD_SIZE_8BIT, s_buf, 2) != HAL_OK) {
        s_errors++;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1) return;
    s_raw = (uint16_t)(((uint16_t)s_buf[0] << 8) | s_buf[1]) & 0x0FFFu;
    s_fresh = true;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)
{
    if (h->Instance == I2C1) s_errors++;
}

bool board_encoder_take(uint16_t *raw)
{
    if (!s_fresh) return false;
    s_fresh = false;
    *raw = s_raw;
    return true;
}

uint32_t board_encoder_errors(void)
{
    return s_errors;
}

/* If the bus wedges (SDA held low by a confused slave), the HAL state never
 * returns to READY. Re-initialising the peripheral clears it. */
void board_encoder_poll(void)
{
    uint32_t now = HAL_GetTick();
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
        s_busy_since_ms = 0;
        return;
    }
    if (s_busy_since_ms == 0) {
        s_busy_since_ms = now;
    } else if (now - s_busy_since_ms > 50u) {
        s_errors++;
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        s_busy_since_ms = 0;
    }
}
