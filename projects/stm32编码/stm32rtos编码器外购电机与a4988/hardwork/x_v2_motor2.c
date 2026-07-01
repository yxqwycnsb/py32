#include "x_v2_motor2.h"
#include "x_v2_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "X_V2.h"

#define X_V2_MOTOR2_ADDR 1


// 记录上一次读取到的电机角度，避免通信超时时返回异常值
static float s_last_angle_deg = 0.0f;
// 记录最近一次读取到的电流值，单位 mA
static int32_t s_last_bus_current_ma = 0;
static int32_t s_last_phase_current_ma = 0;

// 等待串口接收到一帧完整数据，超时返回 false
static bool _wait_rx_frame(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    taskENTER_CRITICAL();
    rxCount = 0;
    rxFrameFlag = false;
    taskEXIT_CRITICAL();
    while (rxFrameFlag == false)
    {
        if ((xTaskGetTickCount() - start) >= timeout)
        {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    taskENTER_CRITICAL();
    rxFrameFlag = false;
    taskEXIT_CRITICAL();
    return true;
}

// 读取电机当前编码器位置，并换算成角度值（单位：度）
static float _read_cpos_deg(void)
{
    X_V2_Read_Sys_Params(X_V2_MOTOR2_ADDR, (SysParams_t)S_CPOS);
    if (!_wait_rx_frame(500)) return s_last_angle_deg;
    taskENTER_CRITICAL();
    bool valid = (rxCmd[1] == 0x0F && rxCount >= 8);
    uint8_t sign_byte = rxCmd[3];
    uint32_t pos = 0;
    if (valid)
    {
        pos = ((uint32_t)rxCmd[4] << 24) |
              ((uint32_t)rxCmd[5] << 16) |
              ((uint32_t)rxCmd[6] << 8) |
              ((uint32_t)rxCmd[7]);
    }
    taskEXIT_CRITICAL();
    if (valid)
    {
        float deg = (float)pos * 0.1f;
        if (sign_byte) deg = -deg;
        s_last_angle_deg = deg;
    }
    return s_last_angle_deg;
}

// 读取系统参数返回的 32 位有符号值
static int32_t _read_sys_param_i32(SysParams_t param, int32_t fallback)
{
    X_V2_Read_Sys_Params(X_V2_MOTOR2_ADDR, param);
    if (!_wait_rx_frame(500)) return fallback;
    taskENTER_CRITICAL();
    bool valid = (rxCmd[1] == 0x0F && rxCount >= 8);
    uint8_t sign_byte = rxCmd[3];
    uint32_t raw = 0;
    if (valid)
    {
        raw = ((uint32_t)rxCmd[4] << 24) |
              ((uint32_t)rxCmd[5] << 16) |
              ((uint32_t)rxCmd[6] << 8) |
              ((uint32_t)rxCmd[7]);
    }
    taskEXIT_CRITICAL();
    if (valid)
    {
        int32_t value = (int32_t)raw;
        if (sign_byte) value = -value;
        return value;
    }
    return fallback;
}

// 电机 2 初始化：先初始化串口，再关闭使能并清零当前位置
void x_v2_motor2_init(void)
{
    x_v2_usart2_init(115200);
    x_v2_motor2_enable(false);
    x_v2_motor2_reset_position();
}

// 控制电机是否使能
void x_v2_motor2_enable(bool enable)
{
    X_V2_En_Control(X_V2_MOTOR2_ADDR, enable, false);
}

// 立即停止电机
void x_v2_motor2_stop(void)
{
    X_V2_Stop_Now(X_V2_MOTOR2_ADDR, false);
}

// 将当前电机位置清零，同时清空缓存角度
void x_v2_motor2_reset_position(void)
{
    X_V2_Reset_CurPos_To_Zero(X_V2_MOTOR2_ADDR);
    s_last_angle_deg = 0.0f;
}

// 读取当前角度，返回值单位为 0.1 度
int32_t x_v2_motor2_get_angle_tenths(void)
{
    return (int32_t)(_read_cpos_deg() * 10.0f);
}

// 发指令让电机2物理运动到绝对位置 0 度（即用户推动后 reset_position 设定的零点）
void x_v2_motor2_return_to_zero(void)
{
    X_V2_Traj_Pos_Control(X_V2_MOTOR2_ADDR, 0, 500, 500, 6.0f, 0.0f, 0, false);
}

// 读取总线电流，单位 mA
int32_t x_v2_motor2_get_bus_current_ma(void)
{
    s_last_bus_current_ma = _read_sys_param_i32(S_CBUS, s_last_bus_current_ma);
    return s_last_bus_current_ma;
}

// 读取相电流，单位 mA
int32_t x_v2_motor2_get_phase_current_ma(void)
{
    s_last_phase_current_ma = _read_sys_param_i32(S_CPHA, s_last_phase_current_ma);
    return s_last_phase_current_ma;
}


