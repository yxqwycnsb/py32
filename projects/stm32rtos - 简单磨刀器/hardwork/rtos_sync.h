#ifndef __RTOS_SYNC_H
#define __RTOS_SYNC_H

#include "FreeRTOS.h"
#include "event_groups.h"

// 电机事件组与完成标志位
extern EventGroupHandle_t xMotorEventGroup;
#define MOTOR2_DONE_BIT (1U << 0)

#endif /* __RTOS_SYNC_H */
