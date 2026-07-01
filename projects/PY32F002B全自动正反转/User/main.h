/**
  ******************************************************************************
  * @file    main.h
  * @author  MCU 应用团队
  * @brief   主程序头文件
  *          存放应用公共定义与函数原型。
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

/* 防止头文件被递归重复包含 -----------------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含头文件 ----------------------------------------------------------------*/
#include "py32f0xx_hal.h"
//#include "py32f002bxx_Start_Kit.h"

/* 私有包含 ------------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 导出变量原型 --------------------------------------------------------------*/
/* 导出函数原型 --------------------------------------------------------------*/
void APP_ErrorHandler(void);
void APP_OnSysTick1ms(void);
uint32_t APP_GetForwardCount(void);
uint32_t APP_GetReverseCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****文件结束******************/
