#ifndef __A4988_H
#define __A4988_H

#if !defined(__cplusplus)
  #if !defined(bool)
    typedef unsigned char bool;
    #define true 1
    #define false 0
  #endif
#endif

#include <stdint.h>

// ================= 步进电机参数 =================
#define STEPS_PER_REV_FULL 200      // 电机物理步数/圈 (1.8°电机为200)
extern volatile uint8_t g_microstep_ratio; // 驱动器细分设置 (例如 16 代表 1/16 细分)
#define A4988_MAX_RPM 200           // 应用层最大安全转速限制 (RPM)

// ================= 电机引脚定义 =================
// 电机1：定时器驱动
#define STEP_PIN             GPIO_Pin_1 // STEP1 脉冲引脚 (PA1)
#define DIR_PIN              GPIO_Pin_6 // DIR1 方向引脚 (PA6)
#define ENABLE_PIN           GPIO_Pin_7 // EN1 使能引脚 (PA7)
#define MOTOR_PORT           GPIOA      // 电机1控制引脚所在端口

// 电机2：软件脉冲驱动
#define STEP2_PIN            GPIO_Pin_0 // STEP2 脉冲引脚 (PA0)
#define DIR2_PIN             GPIO_Pin_4 // DIR2 方向引脚 (PA4)
#define ENABLE2_PIN          GPIO_Pin_5 // EN2 使能引脚 (PA5)
#define MOTOR2_PORT          GPIOA      // 电机2控制引脚所在端口

// ================= 细分模式控制引脚 =================
#define MS1_PIN              GPIO_Pin_5 // MS1 细分控制引脚（两路电机共用）
#define MS1_PORT             GPIOB      // MS1 所在端口

// A4988 细分模式定义
typedef enum {
    MICROSTEP_FULL = 0,     // 全步
    MICROSTEP_1_16 = 1      // 1/16 细分
} MicrostepMode_t;

// ================= 电机状态结构体 =================
typedef struct {
    volatile bool direction;        // 运动方向 (true=正, false=反)
    volatile bool enabled;          // 使能状态
} Stepper_State_t;

extern Stepper_State_t stepper2_state; // 电机状态
extern volatile int32_t stepper2_pulse_count; // 全局脉冲计数 (位置)

extern Stepper_State_t stepper1_state;
extern volatile int32_t stepper1_pulse_count;

// ================= 核心 API =================
void a4988_init(void); // 初始化主电机驱动硬件 (GPIO, TIM2)
void a4988_motor2_init(void); // 初始化第二路电机驱动硬件 (GPIO, TIM3)
void a4988_set_microstep(MicrostepMode_t mode); // 设置细分
void a4988_set_direction(bool direction); // 设置主电机方向
void a4988_enable(bool enable); // 使能/禁用主电机
void a4988_set_hold_enable(bool enable); // 设置运动后是否保持力矩
void a4988_motor2_set_direction(bool direction); // 设置第二路方向
void a4988_motor2_enable(bool enable); // 使能/禁用第二路电机
void a4988_motor2_set_hold_enable(bool enable); // 设置第二路运动后是否保持力矩
void a4988_motor2_stop(void); // 停止第二路电机
void a4988_motor2_run_rpm(uint16_t rpm); // 第二路以恒定速度运行
void a4988_motor2_run_steps(uint32_t steps, bool direction); // 第二路按步数运行

// ================= 位置与状态查询 =================
int32_t a4988_get_angle(void); // 获取当前累计角度 (度)
int32_t a4988_get_pulse_count(void); // 获取当前累计脉冲数（可正可负）
void a4988_reset_angle(void); // 重置角度/脉冲计数为 0

// ================= 运动控制 API =================
// --- S曲线平滑运动 ---
void motor2_move_degrees_smooth(int32_t degrees, uint32_t time_ms, uint16_t max_rpm);
void motor2_smooth_update(void); // 需要在后台任务中周期性调用以更新速度
uint8_t motor2_is_done(void); // 检查平滑运动是否完成

// --- 恒速扫描 ---
void motor2_start_const_speed_scan(uint16_t rpm, int32_t max_degrees);
void motor2_const_speed_stop(void);
uint8_t motor2_const_speed_is_running(void);

// --- 停止 ---
void a4988_stop_motor(void); // 停止所有运动

// ================= 辅助与调试函数 =================
uint32_t a4988_get_steps_per_rev(void);
int32_t degrees_to_steps(int32_t degrees);
void a4988_test_pwm_freq(uint32_t freq_hz); // 输出指定频率的PWM用于测试

void a4988_run_rpm(uint16_t rpm);//以恒定速度运行

#endif /* __A4988_H */

