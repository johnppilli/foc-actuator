/*
 * board_pwm.c - TIM1 three-phase PWM plus the ADC trigger on channel 4.
 *
 * MX_TIM1_Init (CubeMX, main.c) sets up the three complementary pairs at
 * 20 kHz centre-aligned. This file adds, in code so CubeMX regeneration does
 * not wipe it:
 *   - CH4 in PWM mode 2 with TRGO = OC4REF: one rising edge per PWM period,
 *     BOARD_ADC_TRIGGER_LEAD ticks before the counter peak (all low-side
 *     switches on, so the shunts carry the phase currents).
 *   - OSSI enabled with idle states low, so when the main output is disabled
 *     every gate is actively driven off instead of left floating.
 */
#include "board.h"
#include "foc.h"

void board_pwm_init(void)
{
    TIM_OC_InitTypeDef oc = { 0 };
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = BOARD_PWM_ARR - BOARD_ADC_TRIGGER_LEAD;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    TIM_MasterConfigTypeDef mc = { 0 };
    mc.MasterOutputTrigger = TIM_TRGO_OC4REF;
    mc.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    mc.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &mc) != HAL_OK) Error_Handler();

    TIM_BreakDeadTimeConfigTypeDef bdt = { 0 };
    bdt.OffStateRunMode = TIM_OSSR_DISABLE;
    bdt.OffStateIDLEMode = TIM_OSSI_ENABLE;      /* drive idle levels when MOE = 0 */
    bdt.LockLevel = TIM_LOCKLEVEL_OFF;
    bdt.DeadTime = 100;                          /* ~590 ns at 170 MHz, same as CubeMX */
    bdt.BreakState = TIM_BREAK_DISABLE;
    bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bdt.BreakFilter = 0;
    bdt.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    bdt.Break2State = TIM_BREAK2_DISABLE;
    bdt.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    bdt.Break2Filter = 0;
    bdt.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK) Error_Handler();

    board_pwm_set_duty(0.5f, 0.5f, 0.5f);
}

void board_pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);     /* no pin: only feeds TRGO */
    board_pwm_output(false);                      /* HAL_TIM_PWM_Start set MOE; start safe */
}

void board_pwm_set_duty(float a, float b, float c)
{
    const float arr = (float)BOARD_PWM_ARR;
    a = foc_clampf(a, 0.0f, 1.0f);
    b = foc_clampf(b, 0.0f, 1.0f);
    c = foc_clampf(c, 0.0f, 1.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(a * arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(b * arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(c * arr));
}

void board_pwm_output(bool enable)
{
    if (enable) __HAL_TIM_MOE_ENABLE(&htim1);
    else        __HAL_TIM_MOE_DISABLE(&htim1);
}
