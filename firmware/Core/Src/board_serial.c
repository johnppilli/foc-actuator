/*
 * board_serial.c - USART2 on the ST-LINK virtual COM port (PB3 TX, PB4 RX).
 *
 * Out: binary telemetry frames drained from g_telem_ring (filled by the
 *      control loop) plus occasional text replies to commands. The Python
 *      decoder resynchronises across the text.
 * In:  newline-terminated ASCII commands:
 *        m idle|open|calib|torque|haptic   select mode
 *        q <amps>                          iq reference (torque mode)
 *        d <amps>                          id reference (normally 0)
 *        h <0..5>                          haptic preset (haptic mode)
 *        o <volts> <elec rad/s>            open-loop amplitude and speed
 *        r                                 clear fault -> idle
 *        ?                                 status text
 */
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart2;

static uint8_t s_rx_byte;
static char s_line[64];
static volatile uint8_t s_line_len;
static volatile bool s_line_ready;
static uint8_t s_tx_buf[256];
static volatile bool s_tx_busy;

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance != USART2) return;
    RCC_PeriphCLKInitTypeDef pc = { 0 };
    pc.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    pc.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) Error_Handler();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = { 0 };
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void board_serial_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = BOARD_UART_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) Error_Handler();

    s_line_len = 0;
    s_line_ready = false;
    s_tx_busy = false;
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART2) return;
    char c = (char)s_rx_byte;
    if (c == '\n' || c == '\r') {
        if (s_line_len > 0 && !s_line_ready) {
            s_line[s_line_len] = '\0';
            s_line_ready = true;
        }
    } else if (!s_line_ready && s_line_len < sizeof s_line - 1) {
        s_line[s_line_len++] = c;
    }
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART2) s_tx_busy = false;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART2) return;
    s_tx_busy = false;
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);   /* re-arm after overrun/noise */
}

void board_serial_print(const char *s)
{
    uint32_t t0 = HAL_GetTick();
    while (s_tx_busy && HAL_GetTick() - t0 < 20u) { }
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)strlen(s), 50);
}

static foc_mode_t parse_mode(const char *s, bool *ok)
{
    *ok = true;
    if (!strcmp(s, "idle"))   return MODE_IDLE;
    if (!strcmp(s, "open"))   return MODE_OPENLOOP;
    if (!strcmp(s, "calib"))  return MODE_CALIB;
    if (!strcmp(s, "torque")) return MODE_TORQUE;
    if (!strcmp(s, "haptic")) return MODE_HAPTIC;
    *ok = false;
    return MODE_IDLE;
}

static void handle_command(char *line)
{
    char reply[160];
    char *save = 0;
    char *cmd = strtok_r(line, " \t", &save);
    if (!cmd) return;
    char *a1 = strtok_r(0, " \t", &save);
    char *a2 = strtok_r(0, " \t", &save);

    switch (cmd[0]) {
    case 'm': {
        bool ok = false;
        foc_mode_t m = a1 ? parse_mode(a1, &ok) : MODE_IDLE;
        if (!ok) { board_serial_print("\nerr: m idle|open|calib|torque|haptic\n"); return; }
        foc_loop_request_mode(m);
        snprintf(reply, sizeof reply, "\nok mode %s\n", a1);
        board_serial_print(reply);
        return;
    }
    case 'q':
        if (!a1) { board_serial_print("\nerr: q <amps>\n"); return; }
        foc_loop_set_iq_ref(strtof(a1, 0));
        board_serial_print("\nok\n");
        return;
    case 'd':
        if (!a1) { board_serial_print("\nerr: d <amps>\n"); return; }
        foc_loop_set_id_ref(strtof(a1, 0));
        board_serial_print("\nok\n");
        return;
    case 'h':
        if (!a1) { board_serial_print("\nerr: h <preset>\n"); return; }
        foc_loop_set_haptic_preset(atoi(a1));
        board_serial_print("\nok\n");
        return;
    case 'o':
        if (!a1 || !a2) { board_serial_print("\nerr: o <volts> <elec rad/s>\n"); return; }
        foc_loop_set_openloop(strtof(a1, 0), strtof(a2, 0));
        board_serial_print("\nok\n");
        return;
    case 'r':
        foc_loop_clear_fault();
        board_serial_print("\nok fault cleared\n");
        return;
    case '?':
        foc_loop_status_text(reply, sizeof reply);
        board_serial_print(reply);
        return;
    default:
        board_serial_print("\nerr: unknown command (m q d h o r ?)\n");
        return;
    }
}

void board_serial_poll(void)
{
    if (s_line_ready) {
        char line[sizeof s_line];
        strncpy(line, s_line, sizeof line);
        line[sizeof line - 1] = '\0';
        s_line_len = 0;
        s_line_ready = false;
        handle_command(line);
    }
    if (!s_tx_busy) {
        uint32_t n = telem_ring_pop(&g_telem_ring, s_tx_buf, sizeof s_tx_buf);
        if (n > 0) {
            s_tx_busy = true;
            if (HAL_UART_Transmit_IT(&huart2, s_tx_buf, (uint16_t)n) != HAL_OK) s_tx_busy = false;
        }
    }
}
