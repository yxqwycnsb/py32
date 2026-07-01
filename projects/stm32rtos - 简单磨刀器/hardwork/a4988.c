#include "a4988.h"
#include "stm32f10x.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>
#include "rtos_sync.h"

// 本模块调试日志开关（0=关闭日志，1=输出详细日志）
#ifndef A4988_DEBUG
#define A4988_DEBUG 0
#endif

#if !A4988_DEBUG
#if defined(usartSendString)
#undef usartSendString
#endif
#define usartSendString(...) ((void)0)
#endif

// ================= FreeRTOS support =================
#define USE_FREERTOS 1

#if USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#define Delay_GetTick() xTaskGetTickCount()
#endif

Stepper_State_t stepper1_state = {0};
Stepper_State_t stepper2_state = {0};
volatile int32_t stepper1_pulse_count = 0;
volatile int32_t stepper2_pulse_count = 0;

// 保持力矩开关（1=结束后保持上电，0=结束后断电）
volatile uint8_t g_hold_enable1 = 1;
volatile uint8_t g_hold_enable2 = 1;

// 当前细分倍率（1=全步；16=1/16），默认按硬件初始化为1/16
volatile uint8_t g_microstep_ratio = 16;

uint32_t a4988_get_steps_per_rev(void)
{
    return (uint32_t)STEPS_PER_REV_FULL * (uint32_t)g_microstep_ratio;
}

int32_t a4988_get_pulse_count(void)
{
    return stepper1_pulse_count;
}

typedef struct {
    float max_velocity, max_acceleration, max_jerk;
    float t1, t2, t3, t4, t5, t6, t7;
    float v_max_reached, total_time;
    float s1, s2, s3, s4, s5, s6, s7;
    float v1, v2, v3, v4, v5, v6, v7;
} SCurveProfile_t;

typedef struct {
    volatile uint32_t target_pulses, current_pulses;
    volatile uint8_t is_running;
    volatile float current_freq, current_velocity, current_acceleration;
    volatile uint32_t start_time;
    SCurveProfile_t profile;
    volatile uint8_t current_phase;
} Motor2_SmoothCtrl_t;

static Motor2_SmoothCtrl_t motor2_smooth = {0};

// ================= 恒速扫描控制结构体 =================
typedef struct {
    volatile uint8_t is_running;        // 是否正在运行
    volatile int32_t start_pulse;       // 起始脉冲数
    volatile int32_t max_pulses;        // 最大脉冲数（相对起始点）
    volatile int32_t target_pulses;     // 目标脉冲数（绝对值）
    volatile uint8_t direction;         // 方向
    volatile uint16_t rpm;              // 恒定转速
    volatile uint16_t freq_hz;          // 对应的脉冲频率
    volatile uint32_t next_tick;        // 下一次脉冲时间
    volatile uint32_t pulse_period_ms;  // 脉冲周期(ms)
    volatile uint8_t pulse_high;        // 当前STEP电平
} Motor2_ConstSpeedCtrl_t;

static Motor2_ConstSpeedCtrl_t motor2_const_speed = {0};

#define STEPPER_TIMER TIM2

#define MOTOR2_STEP_GPIO_PORT MOTOR2_PORT
#define MOTOR2_STEP_PIN STEP2_PIN

static void _a4988_update_pwm_freq(float freq_hz);
static void _a4988_stop_motor_internal(void);
static void _a4988_motor2_pulse(void);
static void _a4988_motor2_step_delay(uint32_t half_us);
static void _scurve_calculate_profile(SCurveProfile_t *profile, float distance, float max_vel, float max_acc, float max_jerk);
static float _scurve_get_velocity_at_time(const SCurveProfile_t *profile, float t);

static void _motor1_pulse_isr(void)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(MOTOR_PORT, DIR_PIN);
    if (current_dir) stepper1_pulse_count--;
    else stepper1_pulse_count++;
    stepper1_state.direction = current_dir ? true : false;
}

static void _motor2_pulse_isr(void)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(MOTOR2_PORT, DIR2_PIN);
    if (current_dir) stepper2_pulse_count--;
    else stepper2_pulse_count++;
    stepper2_state.direction = current_dir ? true : false;
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }

    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
        _motor1_pulse_isr();
    }
}


static void _a4988_common_gpio_init(GPIO_TypeDef *port, uint16_t step_pin, uint16_t dir_pin, uint16_t en_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = dir_pin | en_pin;
    GPIO_Init(port, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = step_pin;
    GPIO_Init(port, &gpio);
}

void a4988_init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    _a4988_common_gpio_init(MOTOR_PORT, STEP_PIN, DIR_PIN, ENABLE_PIN);
    GPIO_ResetBits(MOTOR_PORT, DIR_PIN);
    GPIO_SetBits(MOTOR_PORT, ENABLE_PIN);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = MS1_PIN;
    GPIO_Init(MS1_PORT, &gpio);

    GPIO_SetBits(MS1_PORT, MS1_PIN);
    g_microstep_ratio = 16;

    tim.TIM_Period = 999;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(STEPPER_TIMER, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 500;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(STEPPER_TIMER, &oc);
    TIM_OC2PreloadConfig(STEPPER_TIMER, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(STEPPER_TIMER, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC2 | TIM_IT_Update, ENABLE);
    TIM_CtrlPWMOutputs(STEPPER_TIMER, ENABLE);
    TIM_Cmd(STEPPER_TIMER, DISABLE);
    _a4988_update_pwm_freq(0.0f);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void a4988_motor2_init(void)
{
    _a4988_common_gpio_init(MOTOR2_PORT, STEP2_PIN, DIR2_PIN, ENABLE2_PIN);
    GPIO_ResetBits(MOTOR2_PORT, DIR2_PIN);
    GPIO_SetBits(MOTOR2_PORT, ENABLE2_PIN);
}

void a4988_set_microstep(MicrostepMode_t mode)
{
    // 当前实现仅支持全步/1/16（只控制 MS1），MS2/MS3 假设已硬件连接。
    if (mode == MICROSTEP_FULL)
    {
        GPIO_ResetBits(MS1_PORT, MS1_PIN);
        g_microstep_ratio = 1;
    }
    else
    {
        GPIO_SetBits(MS1_PORT, MS1_PIN);
        g_microstep_ratio = 16;
    }
}

void a4988_set_hold_enable(bool enable)
{
    g_hold_enable1 = enable ? 1 : 0;
}

void a4988_motor2_set_hold_enable(bool enable)
{
    g_hold_enable2 = enable ? 1 : 0;
}

void a4988_set_direction(bool direction)
{
    stepper1_state.direction = direction;
    // 电机1机械方向修正：true=顺时针/正转时输出高电平
    GPIO_WriteBit(MOTOR_PORT, DIR_PIN, direction ? Bit_SET : Bit_RESET);
}

void a4988_motor2_set_direction(bool direction)
{
    stepper2_state.direction = direction;
    GPIO_WriteBit(MOTOR2_PORT, DIR2_PIN, direction ? Bit_RESET : Bit_SET);
}

void a4988_enable(bool enable)
{
    GPIO_WriteBit(MOTOR_PORT, ENABLE_PIN, enable ? Bit_RESET : Bit_SET);
    stepper1_state.enabled = enable;
    if (!enable) {
        _a4988_update_pwm_freq(0.0f);
        TIM_Cmd(STEPPER_TIMER, DISABLE);
    }
}

void a4988_motor2_enable(bool enable)
{
    GPIO_WriteBit(MOTOR2_PORT, ENABLE2_PIN, enable ? Bit_RESET : Bit_SET);
    stepper2_state.enabled = enable;
}

int32_t a4988_get_angle(void)
{
    uint32_t spr = a4988_get_steps_per_rev();
    if (spr == 0U) return 0;
    // 角度使用有符号脉冲计数，允许正/负方向显示，不会在低于 0 时发生无符号回绕。
    return (stepper1_pulse_count * 360) / (int32_t)spr;
}

void a4988_reset_angle(void)
{
    stepper2_pulse_count = 0;
}

static void _a4988_update_pwm_freq(float freq_hz)
{
    if (freq_hz <= 0.0f) {
        TIM_SetCompare2(STEPPER_TIMER, 0);
        return;
    }

    if (freq_hz < 5.0f) freq_hz = 5.0f;
    if (freq_hz > 10000.0f) freq_hz = 10000.0f;

    float period_us = 1000000.0f / freq_hz;
    uint32_t period_ticks = (uint32_t)(period_us);
    if (period_ticks < 100) period_ticks = 100;
    if (period_ticks > 65530) period_ticks = 65530;

    uint32_t arr = period_ticks - 1;
    uint32_t ccr = arr / 2;
    if (ccr == 0) ccr = 1;
    if (ccr >= arr) ccr = arr - 1;

    TIM_SetAutoreload(STEPPER_TIMER, arr);
    TIM_SetCompare2(STEPPER_TIMER, ccr);
    TIM_GenerateEvent(STEPPER_TIMER, TIM_EventSource_Update);
}

static void _a4988_update_pwm_freq2(float freq_hz)
{
    if (freq_hz <= 0.0f) {
        TIM_SetCompare2(TIM3, 0);
        return;
    }

    if (freq_hz < 5.0f) freq_hz = 5.0f;
    if (freq_hz > 10000.0f) freq_hz = 10000.0f;

    float period_us = 1000000.0f / freq_hz;
    uint32_t period_ticks = (uint32_t)(period_us);
    if (period_ticks < 100) period_ticks = 100;
    if (period_ticks > 65530) period_ticks = 65530;

    uint32_t arr = period_ticks - 1;
    uint32_t ccr = arr / 2;
    if (ccr == 0) ccr = 1;
    if (ccr >= arr) ccr = arr - 1;

    TIM_SetAutoreload(TIM3, arr);
    TIM_SetCompare2(TIM3, ccr);
    TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
}

static void _scurve_calculate_profile(SCurveProfile_t *profile, float distance, float max_vel, float max_acc, float max_jerk)
{
    profile->max_velocity = max_vel;
    profile->max_acceleration = max_acc;
    profile->max_jerk = max_jerk;

    float v_peak = max_vel;
    float t1 = max_acc / max_jerk;
    if (t1 < 0.0f) t1 = 0.0f;
    float v_jerk = (max_acc * max_acc) / max_jerk;
    float t2 = 0.0f, t3 = t1;

    if (v_jerk >= v_peak) {
        t1 = sqrtf(v_peak / max_jerk);
        t2 = 0.0f;
        t3 = t1;
    } else {
        t2 = (v_peak - v_jerk) / max_acc;
        if (t2 < 0.0f) t2 = 0.0f;
    }

    float v1 = 0.5f * max_jerk * t1 * t1;
    float s1 = (1.0f / 6.0f) * max_jerk * t1 * t1 * t1;
    float v2 = v1 + max_acc * t2;
    float s2 = s1 + v1 * t2 + 0.5f * max_acc * t2 * t2;
    float s3 = s2 + v2 * t3 + 0.5f * max_acc * t3 * t3 - (1.0f / 6.0f) * max_jerk * t3 * t3 * t3;
    float s_acc_half = s3;
    float s_acc_total = 2.0f * s_acc_half;
    float s_const = distance - s_acc_total;
    int iter = 0;

    while (s_const < 0.0f && iter < 12) {
        v_peak *= 0.9f;
        if (v_peak <= 0.0f) { v_peak = 0.0f; break; }
        if ((max_acc * max_acc) / max_jerk >= v_peak) {
            t1 = sqrtf(v_peak / max_jerk);
            t2 = 0.0f;
            t3 = t1;
        } else {
            t1 = max_acc / max_jerk;
            t2 = (v_peak - (max_acc * max_acc) / max_jerk) / max_acc;
            if (t2 < 0.0f) t2 = 0.0f;
            t3 = t1;
        }
        v1 = 0.5f * max_jerk * t1 * t1;
        s1 = (1.0f / 6.0f) * max_jerk * t1 * t1 * t1;
        v2 = v1 + max_acc * t2;
        s2 = s1 + v1 * t2 + 0.5f * max_acc * t2 * t2;
        s3 = s2 + v2 * t3 + 0.5f * max_acc * t3 * t3 - (1.0f / 6.0f) * max_jerk * t3 * t3 * t3;
        s_acc_half = s3;
        s_acc_total = 2.0f * s_acc_half;
        s_const = distance - s_acc_total;
        iter++;
    }

    float t4 = (s_const > 0.0f && v_peak > 0.0f) ? (s_const / v_peak) : 0.0f;
    profile->v_max_reached = v_peak;
    profile->t1 = t1; profile->t2 = t2; profile->t3 = t3;
    profile->t4 = t4; profile->t5 = t1; profile->t6 = t2; profile->t7 = t3;
    profile->total_time = 2.0f * (t1 + t2 + t3) + t4;
    profile->v1 = v1; profile->v2 = v2; profile->v3 = v_peak;
    profile->v4 = v_peak; profile->v5 = v_peak - 0.5f * max_jerk * t1 * t1;
    profile->v6 = profile->v5 - max_acc * t2;
    profile->v7 = 0.0f;
    profile->s1 = s1; profile->s2 = s2; profile->s3 = s3;
    profile->s4 = s3 + v_peak * t4;
    profile->s5 = profile->s4 + profile->v4 * t1 - (1.0f / 6.0f) * max_jerk * t1 * t1 * t1;
    profile->s6 = profile->s5 + profile->v5 * t2 - 0.5f * max_acc * t2 * t2;
    profile->s7 = distance;
}

static float _scurve_get_velocity_at_time(const SCurveProfile_t *profile, float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= profile->total_time) return 0.0f;
    float t_acc = 0.0f;

    if (t <= profile->t1) return 0.5f * profile->max_jerk * t * t;
    t_acc += profile->t1;
    if (t <= t_acc + profile->t2) return profile->v1 + profile->max_acceleration * (t - t_acc);
    t_acc += profile->t2;
    if (t <= t_acc + profile->t3) return profile->v2 + profile->max_acceleration * (t - t_acc) - 0.5f * profile->max_jerk * (t - t_acc) * (t - t_acc);
    t_acc += profile->t3;
    if (t <= t_acc + profile->t4) return profile->v3;
    t_acc += profile->t4;
    if (t <= t_acc + profile->t5) return profile->v4 - 0.5f * profile->max_jerk * (t - t_acc) * (t - t_acc);
    t_acc += profile->t5;
    if (t <= t_acc + profile->t6) return profile->v5 - profile->max_acceleration * (t - t_acc);
    t_acc += profile->t6;
    if (t <= t_acc + profile->t7) return profile->v6 - profile->max_acceleration * (t - t_acc) + 0.5f * profile->max_jerk * (t - t_acc) * (t - t_acc);
    return 10.0f;
}

int32_t degrees_to_steps(int32_t degrees)
{
    uint32_t spr = a4988_get_steps_per_rev();
    return (int32_t)((int64_t)degrees * (int64_t)spr / 360);
}

static void _a4988_stop_motor_internal(void)
{
    TIM_Cmd(STEPPER_TIMER, DISABLE);
    motor2_smooth.is_running = 0;
    motor2_const_speed.is_running = 0;

    _a4988_update_pwm_freq(0.0f);

    if (g_hold_enable1) {
        a4988_enable(true);
    } else {
        a4988_enable(false);
    }
}

static void _a4988_stop_motor2_internal(void)
{
    GPIO_ResetBits(MOTOR2_PORT, STEP2_PIN);
    if (g_hold_enable2) {
        a4988_motor2_enable(true);
    } else {
        a4988_motor2_enable(false);
    }
}

static void _a4988_motor2_step_delay(uint32_t half_us)
{
    if (half_us == 0U) half_us = 1U;
    GPIO_ResetBits(MOTOR2_PORT, STEP2_PIN);
    Delay_us(half_us);
    GPIO_SetBits(MOTOR2_PORT, STEP2_PIN);
    Delay_us(half_us);
    GPIO_ResetBits(MOTOR2_PORT, STEP2_PIN);
}

static void _a4988_motor2_begin_move(uint32_t steps, bool direction, uint16_t rpm)
{
    if (steps == 0U || rpm == 0U)
    {
        a4988_motor2_stop();
        return;
    }

    if (rpm > A4988_MAX_RPM) rpm = A4988_MAX_RPM;
    uint32_t spr = a4988_get_steps_per_rev();
    uint32_t pulse_period_ms = (uint32_t)(60000UL / ((uint32_t)rpm * spr));
    if (pulse_period_ms == 0U) pulse_period_ms = 1U;

    motor2_const_speed.is_running = 1;
    motor2_const_speed.start_pulse = stepper2_pulse_count;
    motor2_const_speed.max_pulses = (int32_t)steps;
    motor2_const_speed.target_pulses = stepper2_pulse_count + (direction ? (int32_t)steps : -(int32_t)steps);
    motor2_const_speed.direction = direction ? 1U : 0U;
    motor2_const_speed.rpm = rpm;
    motor2_const_speed.freq_hz = (uint16_t)(((uint32_t)rpm * spr) / 60U);
    motor2_const_speed.pulse_period_ms = pulse_period_ms;
    motor2_const_speed.next_tick = xTaskGetTickCount();
    motor2_const_speed.pulse_high = 0U;

    a4988_motor2_set_direction(direction);
    a4988_motor2_enable(true);
}

void a4988_stop_motor(void)
{
    _a4988_stop_motor_internal();
}

void a4988_motor2_stop(void)
{
    _a4988_stop_motor2_internal();
    motor2_smooth.is_running = 0;
    motor2_const_speed.is_running = 0;
}

void a4988_motor2_run_rpm(uint16_t rpm)
{
    uint32_t spr = a4988_get_steps_per_rev();
    uint32_t steps = spr;
    _a4988_motor2_begin_move(steps, true, rpm);
}

void a4988_motor2_run_steps(uint32_t steps, bool direction)
{
    _a4988_motor2_begin_move(steps, direction, 20);
}

static void motor2_smooth_pulses(uint32_t pulse_count, uint32_t time_ms, uint16_t max_freq)
{
    if (pulse_count == 0 || time_ms == 0) {
        _a4988_stop_motor2_internal();
        return;
    }

    motor2_smooth.target_pulses = pulse_count;
    motor2_smooth.current_pulses = 0;
    motor2_smooth.start_time = Delay_GetTick();
    motor2_smooth.is_running = 1;

    float max_velocity = (max_freq > 0) ? (float)max_freq : (2.0f * pulse_count * 1000.0f / time_ms);
    if (max_velocity > 10000.0f) max_velocity = 10000.0f;
    if (max_velocity < 50.0f) max_velocity = 50.0f;

    float max_acceleration = max_velocity * 1.5f;
    float max_jerk = max_acceleration * 1.5f;

    _scurve_calculate_profile(&motor2_smooth.profile, (float)pulse_count, max_velocity, max_acceleration, max_jerk);

    a4988_motor2_enable(true);
    _a4988_motor2_step_delay((uint16_t)max_velocity);
}

void motor2_move_degrees_smooth(int32_t degrees, uint32_t time_ms, uint16_t max_rpm)
{
    if (degrees == 0 || time_ms == 0) {
        _a4988_stop_motor2_internal();
        return;
    }

    if (max_rpm == 0 || max_rpm > A4988_MAX_RPM) max_rpm = A4988_MAX_RPM;

    a4988_motor2_set_direction(degrees > 0);
    if (degrees < 0) degrees = -degrees;

    uint32_t spr = a4988_get_steps_per_rev();
    uint32_t pulse_count = (uint32_t)((uint64_t)degrees * (uint64_t)spr / 360ULL);
    uint16_t max_freq = (uint16_t)(((uint32_t)max_rpm * spr) / 60U);
    motor2_smooth_pulses(pulse_count, time_ms, max_freq);
}

void motor2_smooth_update(void)
{
    if (!motor2_smooth.is_running) return;

    uint32_t current_time = Delay_GetTick();
    if (current_time < motor2_const_speed.next_tick) return;

    if (stepper2_pulse_count == motor2_const_speed.target_pulses)
    {
        _a4988_stop_motor2_internal();
        xEventGroupSetBits(xMotorEventGroup, MOTOR2_DONE_BIT);
        motor2_smooth.is_running = 0;
        return;
    }

    motor2_const_speed.next_tick = current_time + motor2_const_speed.pulse_period_ms;
    if (motor2_const_speed.pulse_high == 0U)
    {
        GPIO_SetBits(MOTOR2_PORT, STEP2_PIN);
        motor2_const_speed.pulse_high = 1U;
    }
    else
    {
        GPIO_ResetBits(MOTOR2_PORT, STEP2_PIN);
        motor2_const_speed.pulse_high = 0U;
        if (motor2_const_speed.direction)
            stepper2_pulse_count++;
        else
            stepper2_pulse_count--;
    }
}

uint8_t motor2_is_done(void)
{
    return (motor2_smooth.is_running == 0 && motor2_const_speed.is_running == 0);
}

void motor2_start_const_speed_scan(uint16_t rpm, int32_t max_degrees)
{
    if (rpm == 0 || max_degrees == 0) {
        motor2_const_speed_stop();
        return;
    }
    
    uint32_t spr = a4988_get_steps_per_rev();
    int32_t max_pulses = (int32_t)((int64_t)max_degrees * (int64_t)spr / 360LL);
    uint32_t steps = (max_pulses < 0) ? (uint32_t)(-max_pulses) : (uint32_t)max_pulses;
    bool direction = (max_degrees > 0);
    _a4988_motor2_begin_move(steps, direction, rpm);
}

void motor2_const_speed_stop(void)
{
    if (!motor2_const_speed.is_running) return;

    motor2_const_speed.is_running = 0;
    _a4988_stop_motor2_internal();

    xEventGroupSetBits(xMotorEventGroup, MOTOR2_DONE_BIT);
}

uint8_t motor2_const_speed_is_running(void)
{
    return motor2_const_speed.is_running;
}

void a4988_test_pwm_freq(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        TIM_Cmd(STEPPER_TIMER, DISABLE);
        _a4988_update_pwm_freq(0.0f);
        return;
    }

    a4988_enable(true);
    _a4988_update_pwm_freq((float)freq_hz);
    TIM_Cmd(STEPPER_TIMER, ENABLE);
}

void a4988_run_rpm(uint16_t rpm)
{
    if (rpm == 0)
    {
        a4988_stop_motor();
        return;
    }

    if (rpm > A4988_MAX_RPM) rpm = A4988_MAX_RPM;
    uint32_t spr = a4988_get_steps_per_rev();
    float freq_hz = ((float)rpm * (float)spr) / 60.0f;
    a4988_enable(true);
    _a4988_update_pwm_freq((float)freq_hz);
    TIM_Cmd(STEPPER_TIMER, ENABLE);
}

