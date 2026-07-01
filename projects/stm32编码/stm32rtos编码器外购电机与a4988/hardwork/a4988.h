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

// ================= 细分模式控制引脚 =================
#define MS1_PIN              GPIO_Pin_5 // MS1 细分控制引脚
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

extern Stepper_State_t stepper1_state;
extern volatile int32_t stepper1_pulse_count;

// ================= 核心 API =================
void a4988_init(void);
void a4988_set_microstep(MicrostepMode_t mode);
void a4988_set_direction(bool direction);
void a4988_enable(bool enable);
void a4988_set_hold_enable(bool enable);
void a4988_stop_motor(void);
void a4988_run_rpm(uint16_t rpm);

// ================= 位置与状态查询 =================
int32_t a4988_get_angle(void);
int32_t a4988_get_pulse_count(void);
void a4988_reset_angle(void);

// ================= 辅助与调试函数 =================
uint32_t a4988_get_steps_per_rev(void);
int32_t degrees_to_steps(int32_t degrees);
void a4988_test_pwm_freq(uint32_t freq_hz);

#endif /* __A4988_H */
