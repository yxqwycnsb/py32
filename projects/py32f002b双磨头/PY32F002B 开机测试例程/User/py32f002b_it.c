/**
  ******************************************************************************
  * @file    py32f002b_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_it.h"
#include "bsp_ws2812b.h"

#include "protothread.h"

/******************************************************************************
/*          Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  g_systick_ms++;

  /* Drive etimer polling from SysTick to keep event timers working
   * without needing to call etimer_request_poll() in main loop.
   */
  if(etimer_pending() && (etimer_next_expiration_time() <= protothread_clock_time()))
  {
    etimer_request_poll();
  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
