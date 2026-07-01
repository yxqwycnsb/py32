/**
  ******************************************************************************
  * @file     bsp_tim1_pwm.c
  * @brief    TIM1 PWM driver for PA0 / TIM1_CH1
  ******************************************************************************
  */

#include "bsp_tim1_pwm.h"

static TIM_HandleTypeDef htim1_pwm;

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void BSP_TIM1_PWM_Init(uint16_t prescaler, uint16_t period)
{
  TIM_OC_InitTypeDef sConfig = {0};

  htim1_pwm.Instance = TIM1;
  htim1_pwm.Init.Prescaler = prescaler;
  htim1_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1_pwm.Init.Period = period;
  htim1_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1_pwm.Init.RepetitionCounter = 0;
  htim1_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim1_pwm) != HAL_OK)
  {
    while (1) {}
  }

  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = 0;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1_pwm, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    while (1) {}
  }

  if (HAL_TIM_PWM_Start(&htim1_pwm, TIM_CHANNEL_1) != HAL_OK)
  {
    while (1) {}
  }
}

void BSP_TIM1_PWM_SetDuty(uint8_t duty)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1_pwm);
  uint32_t ccr;

  if (duty > 100U)
  {
    duty = 100U;
  }

  ccr = ((period + 1U) * duty) / 100U;
  if (duty == 0U)
  {
    ccr = 0U;
  }
  else if (ccr == 0U)
  {
    ccr = 1U;
  }
  else if (ccr > period)
  {
    ccr = period;
  }

  __HAL_TIM_SET_COMPARE(&htim1_pwm, TIM_CHANNEL_1, ccr);
}
