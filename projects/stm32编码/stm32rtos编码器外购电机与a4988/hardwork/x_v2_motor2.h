#ifndef __X_V2_MOTOR2_H
#define __X_V2_MOTOR2_H

#include <stdint.h>
#include <stdbool.h>

// 电机 2 初始化
void x_v2_motor2_init(void);
// 使能或关闭电机 2
void x_v2_motor2_enable(bool enable);
// 立即停止电机 2
void x_v2_motor2_stop(void);
// 将当前位置清零
void x_v2_motor2_reset_position(void);
// 获取当前角度，返回单位为 0.1 度
int32_t x_v2_motor2_get_angle_tenths(void);
// 获取总线电流，单位 mA
int32_t x_v2_motor2_get_bus_current_ma(void);
// 获取相电流，单位 mA
int32_t x_v2_motor2_get_phase_current_ma(void);
// 发指令让电机2物理运动到绝对位置 0 度（即用户推动的稳定位置）
void x_v2_motor2_return_to_zero(void);

#endif
