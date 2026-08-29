/*
 * board_current.c - phase current sensing on the B-G431B-ESC1.
 *
 * The three low-side shunts feed the G431's internal op-amps, wired on the
 * board as standalone amplifiers with an external gain network:
 *   OPAMP1: +PA1 -PA3 out PA2 -> ADC1_IN3   (phase U)
 *   OPAMP2: +PA7 -PA5 out PA6 -> ADC2_IN3   (phase V)
 *   OPAMP3: +PB0 -PB2 out PB1 -> ADC1_IN12  (phase W)
 *
 * ADC1 and ADC2 run injected sequences triggered by TIM1 TRGO (= OC4REF, set
 * up in board_pwm.c to fire just before the PWM counter peak, when all
 * low-side switches are on and current flows through the shunts). ADC1's
 * end-of-injected-sequence interrupt calls foc_loop_isr().
 */
#include "board.h"
#include <string.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1, hopamp2, hopamp3;
static float s_offset_counts[3] = { 2048.0f, 2048.0f, 2048.0f };

void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = { 0 };
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    if (h->Instance == OPAMP1) {
        g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (h->Instance == OPAMP2) {
        g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (h->Instance == OPAMP3) {
        g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1) {
        RCC_PeriphCLKInitTypeDef pc = { 0 };
        pc.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        pc.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) Error_Handler();
        __HAL_RCC_ADC12_CLK_ENABLE();
        /* highest priority in the system: this is the control loop */
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    } else if (h->Instance == ADC2) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    }
    /* input pins are already analog from HAL_OPAMP_MspInit */
}

static void opamp_init(OPAMP_HandleTypeDef *h, OPAMP_TypeDef *inst)
{
    h->Instance = inst;
    h->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
    h->Init.Mode = OPAMP_STANDALONE_MODE;
    h->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    h->Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
    h->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    h->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(h) != HAL_OK) Error_Handler();
    if (HAL_OPAMP_Start(h) != HAL_OK) Error_Handler();
}

static void adc_init(ADC_HandleTypeDef *h, ADC_TypeDef *inst)
{
    h->Instance = inst;
    h->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 170/4 = 42.5 MHz */
    h->Init.Resolution = ADC_RESOLUTION_12B;
    h->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    h->Init.GainCompensation = 0;
    h->Init.ScanConvMode = ADC_SCAN_DISABLE;
    h->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    h->Init.LowPowerAutoWait = DISABLE;
    h->Init.ContinuousConvMode = DISABLE;
    h->Init.NbrOfConversion = 1;
    h->Init.DiscontinuousConvMode = DISABLE;
    h->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    h->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    h->Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    h->Init.DMAContinuousRequests = DISABLE;
    h->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    h->Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(h) != HAL_OK) Error_Handler();
}

static void injected_channel(ADC_HandleTypeDef *h, uint32_t channel, uint32_t rank, uint32_t nbr)
{
    ADC_InjectionConfTypeDef j = { 0 };
    j.InjectedChannel = channel;
    j.InjectedRank = rank;
    j.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    j.InjectedSingleDiff = ADC_SINGLE_ENDED;
    j.InjectedOffsetNumber = ADC_OFFSET_NONE;
    j.InjectedOffset = 0;
    j.InjectedNbrOfConversion = nbr;
    j.InjectedDiscontinuousConvMode = DISABLE;
    j.AutoInjectedConv = DISABLE;
    j.QueueInjectedContext = DISABLE;
    j.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    j.InjecOversamplingMode = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(h, &j) != HAL_OK) Error_Handler();
}

void board_current_init(void)
{
    opamp_init(&hopamp1, OPAMP1);
    opamp_init(&hopamp2, OPAMP2);
    opamp_init(&hopamp3, OPAMP3);

    adc_init(&hadc1, ADC1);
    ADC_MultiModeTypeDef mm = { 0 };
    mm.Mode = ADC_MODE_INDEPENDENT;
    mm.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
    mm.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mm) != HAL_OK) Error_Handler();
    injected_channel(&hadc1, ADC_CHANNEL_3, ADC_INJECTED_RANK_1, 2);    /* U on PA2 */
    injected_channel(&hadc1, ADC_CHANNEL_12, ADC_INJECTED_RANK_2, 2);   /* W on PB1 */

    adc_init(&hadc2, ADC2);
    injected_channel(&hadc2, ADC_CHANNEL_3, ADC_INJECTED_RANK_1, 1);    /* V on PA6 */

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
}

void board_current_start(void)
{
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) Error_Handler();
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) Error_Handler();
}

void board_current_read_raw(uint16_t raw[3])
{
    raw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    raw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    raw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

void board_current_read(float i_abc[3])
{
    uint16_t raw[3];
    board_current_read_raw(raw);
    for (int k = 0; k < 3; k++) {
        i_abc[k] = BOARD_CURRENT_SIGN * ((float)raw[k] - s_offset_counts[k]) * BOARD_AMPS_PER_COUNT;
    }
}

void board_current_zero(const uint16_t raw[3])
{
    for (int k = 0; k < 3; k++) s_offset_counts[k] = (float)raw[k];
}

void board_current_offsets(float out[3])
{
    memcpy(out, s_offset_counts, sizeof s_offset_counts);
}

/* ADC1 end of injected sequence: both ADCs have converted (ADC2 only has one
 * channel, so it finished first). This is the 20 kHz control tick. */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1) foc_loop_isr();
}
