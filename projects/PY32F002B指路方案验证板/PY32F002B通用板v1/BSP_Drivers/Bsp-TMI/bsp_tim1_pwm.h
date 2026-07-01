#ifndef __BSP_TIM1_PWM_H
#define __BSP_TIM1_PWM_H

#include "py32f0xx_hal.h"

void BSP_TIM1_PWM_Init(uint16_t prescaler, uint16_t period);
void BSP_TIM1_PWM_SetDuty(uint8_t duty);

#endif /* __BSP_TIM1_PWM_H */
